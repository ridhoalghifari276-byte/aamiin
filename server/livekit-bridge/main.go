package main

import (
	"crypto/subtle"
	"encoding/binary"
	"io"
	"log"
	"net"
	"os"
	"os/exec"
	"sync"
	"time"

	"github.com/livekit/protocol/livekit"
	lksdk "github.com/livekit/server-sdk-go/v2"
	"github.com/pion/webrtc/v4"
	"github.com/pion/webrtc/v4/pkg/media"
	"github.com/hraban/opus"
)

const hdrLen = 48

func env(k, def string) string {
	if v := os.Getenv(k); v != "" {
		return v
	}
	return def
}

var (
	lkURL   = env("LIVEKIT_URL", "ws://livekit:7880")
	lkKey   = env("LIVEKIT_API_KEY", "bodycam")
	lkSec   = env("LIVEKIT_API_SECRET", "")
	token   = []byte(env("LIVEKIT_TOKEN", "X01040688"))
	listen  = env("LISTEN", ":50301")
)

type asm struct {
	n     int
	parts [][]byte
	at    time.Time
}

func (a *asm) ready() bool {
	if a == nil || a.n < 1 || len(a.parts) != a.n {
		return false
	}
	for _, p := range a.parts {
		if p == nil {
			return false
		}
	}
	return true
}

type session struct {
	device string
	video  chan []byte
	audio  chan []byte
}

var (
	sessionsMu sync.Mutex
	sessions   = map[string]*session{}
	asmMu      sync.Mutex
	asmBuf     = map[string]*asm{}
)

func main() {
	if lkSec == "" {
		log.Fatal("LIVEKIT_API_SECRET is empty")
	}
	pc, err := net.ListenPacket("udp", listen)
	if err != nil {
		log.Fatal(err)
	}
	log.Printf("[lk] listening %s → %s", listen, lkURL)
	buf := make([]byte, 8192)
	for {
		n, _, err := pc.ReadFrom(buf)
		if err != nil {
			log.Printf("[lk] read: %v", err)
			time.Sleep(200 * time.Millisecond)
			continue
		}
		pkt := append([]byte(nil), buf[:n]...)
		handle(pkt)
		sweep()
	}
}

func handle(pkt []byte) {
	if len(pkt) < hdrLen || string(pkt[:4]) != "BCRT" || pkt[4] != 1 {
		return
	}
	tok := cstr(pkt[36:48])
	if len(tok) != len(token) || subtle.ConstantTimeCompare(tok, token) != 1 {
		return
	}
	kind := pkt[5]
	if kind != 1 && kind != 2 {
		return
	}
	nchunks := int(pkt[7])
	idx := int(pkt[18])
	plen := int(binary.LittleEndian.Uint16(pkt[16:18]))
	frame := binary.LittleEndian.Uint16(pkt[14:16])
	device := string(cstr(pkt[20:36]))
	if !safeID(device) || nchunks < 1 || nchunks > 8 || idx >= nchunks || hdrLen+plen > len(pkt) {
		return
	}
	part := append([]byte(nil), pkt[hdrLen:hdrLen+plen]...)
	key := device + "|" + string(rune(kind)) + "|" + itoa(int(frame))
	asmMu.Lock()
	slot := asmBuf[key]
	if slot == nil || slot.n != nchunks {
		slot = &asm{n: nchunks, parts: make([][]byte, nchunks), at: time.Now()}
		asmBuf[key] = slot
	}
	slot.parts[idx] = part
	ready := slot.ready()
	var blob []byte
	if ready {
		for _, p := range slot.parts {
			blob = append(blob, p...)
		}
		delete(asmBuf, key)
	}
	asmMu.Unlock()
	if !ready {
		return
	}
	if kind == 1 {
		if len(blob) < 128 || blob[0] != 0xFF || blob[1] != 0xD8 || blob[len(blob)-2] != 0xFF || blob[len(blob)-1] != 0xD9 {
			return
		}
		offer(sessionFor(device).video, blob)
		return
	}
	if len(blob) >= 2 && len(blob)%2 == 0 {
		offer(sessionFor(device).audio, blob)
	}
}

func sweep() {
	asmMu.Lock()
	defer asmMu.Unlock()
	now := time.Now()
	for k, slot := range asmBuf {
		if now.Sub(slot.at) > 120*time.Millisecond {
			delete(asmBuf, k)
		}
	}
}

func sessionFor(device string) *session {
	sessionsMu.Lock()
	defer sessionsMu.Unlock()
	s := sessions[device]
	if s != nil {
		return s
	}
	s = &session{
		device: device,
		video:  make(chan []byte, 1),
		audio:  make(chan []byte, 16),
	}
	sessions[device] = s
	go s.run()
	return s
}

func (s *session) run() {
	backoff := time.Second
	for {
		err := s.publish()
		log.Printf("[lk] %s republish in %s: %v", s.device, backoff, err)
		time.Sleep(backoff)
		if backoff < 15*time.Second {
			backoff *= 2
		}
	}
}

func (s *session) publish() error {
	dead := make(chan struct{})
	var once sync.Once
	finish := func() { once.Do(func() { close(dead) }) }

	room, err := lksdk.ConnectToRoom(lkURL, lksdk.ConnectInfo{
		APIKey:              lkKey,
		APISecret:           lkSec,
		RoomName:            s.device,
		ParticipantIdentity: "cam-" + s.device,
		ParticipantName:     s.device,
	}, &lksdk.RoomCallback{OnDisconnected: func() { finish() }})
	if err != nil {
		return err
	}
	defer room.Disconnect()

	vtrack, err := lksdk.NewLocalSampleTrack(webrtc.RTPCodecCapability{
		MimeType:    webrtc.MimeTypeH264,
		ClockRate:   90000,
		SDPFmtpLine: "level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f",
	})
	if err != nil {
		return err
	}
	if _, err = room.LocalParticipant.PublishTrack(vtrack, &lksdk.TrackPublicationOptions{
		Name: "camera", Source: livekit.TrackSource_CAMERA,
	}); err != nil {
		return err
	}

	atrack, err := lksdk.NewLocalSampleTrack(webrtc.RTPCodecCapability{
		MimeType: webrtc.MimeTypeOpus, ClockRate: 48000, Channels: 2,
	})
	if err != nil {
		return err
	}
	if _, err = room.LocalParticipant.PublishTrack(atrack, &lksdk.TrackPublicationOptions{
		Name: "mic", Source: livekit.TrackSource_MICROPHONE,
	}); err != nil {
		return err
	}

	enc, err := opus.NewEncoder(48000, 2, opus.AppVoIP)
	if err != nil {
		return err
	}
	_ = enc.SetBitrate(32000)

	cmd, stdin, stdout, err := startFFmpeg()
	if err != nil {
		return err
	}
	defer func() {
		_ = stdin.Close()
		if cmd.Process != nil {
			_ = cmd.Process.Kill()
		}
		_ = cmd.Wait()
	}()
	log.Printf("[lk] publishing %s", s.device)

	go writeJPEG(stdin, s.video, dead, finish)
	go readH264(stdout, vtrack, finish)
	go writeOpus(enc, atrack, s.audio, dead)

	<-dead
	return io.EOF
}

func writeJPEG(w io.Writer, ch <-chan []byte, dead <-chan struct{}, finish func()) {
	defer finish()
	for {
		select {
		case <-dead:
			return
		case jpg := <-ch:
			if _, err := w.Write(jpg); err != nil {
				return
			}
		}
	}
}

func writeOpus(enc *opus.Encoder, track *lksdk.LocalTrack, ch <-chan []byte, dead <-chan struct{}) {
	var pcm []int16
	buf := make([]byte, 4000)
	for {
		select {
		case <-dead:
			return
		case chunk := <-ch:
			for i := 0; i+1 < len(chunk); i += 2 {
				pcm = append(pcm, int16(binary.LittleEndian.Uint16(chunk[i:i+2])))
			}
			for len(pcm) >= 320 {
				stereo := upsampleStereo(pcm[:320])
				pcm = append([]int16(nil), pcm[320:]...)
				n, err := enc.Encode(stereo, buf)
				if err != nil || n <= 0 {
					continue
				}
				_ = track.WriteSample(media.Sample{Data: append([]byte(nil), buf[:n]...), Duration: 20 * time.Millisecond}, nil)
			}
		}
	}
}

func upsampleStereo(in []int16) []int16 {
	out := make([]int16, 960*2)
	for i, s := range in {
		base := i * 3
		for k := 0; k < 3; k++ {
			out[(base+k)*2] = s
			out[(base+k)*2+1] = s
		}
	}
	return out
}

func readH264(r io.Reader, track *lksdk.LocalTrack, finish func()) {
	defer finish()
	buf := make([]byte, 32*1024)
	var rest []byte
	var au []byte
	saw := false
	var last time.Time
	flush := func() {
		if !saw || len(au) == 0 {
			au = nil
			saw = false
			return
		}
		d := time.Since(last)
		if last.IsZero() || d < 20*time.Millisecond || d > 200*time.Millisecond {
			d = 66 * time.Millisecond
		}
		last = time.Now()
		_ = track.WriteSample(media.Sample{Data: append([]byte(nil), au...), Duration: d}, nil)
		au = nil
		saw = false
	}
	for {
		n, err := r.Read(buf)
		if n > 0 {
			rest = append(rest, buf[:n]...)
			idxs := startCodes(rest)
			if len(idxs) >= 2 {
				for i := 0; i < len(idxs)-1; i++ {
					nal := rest[idxs[i]:idxs[i+1]]
					t := nalType(nal)
					if (t == 1 || t == 5 || t == 9) && saw {
						flush()
					}
					au = append(au, nal...)
					if t == 1 || t == 5 {
						saw = true
					}
				}
				rest = append([]byte(nil), rest[idxs[len(idxs)-1]:]...)
			}
			if len(rest) > 1024*1024 {
				rest = nil
			}
		}
		if err != nil {
			flush()
			return
		}
	}
}

func startCodes(b []byte) []int {
	var out []int
	for i := 0; i+3 < len(b); i++ {
		if b[i] == 0 && b[i+1] == 0 && b[i+2] == 1 {
			out = append(out, i)
			i += 2
			continue
		}
		if i+4 < len(b) && b[i] == 0 && b[i+1] == 0 && b[i+2] == 0 && b[i+3] == 1 {
			out = append(out, i)
			i += 3
		}
	}
	return out
}

func nalType(nal []byte) int {
	i := 0
	if len(nal) >= 4 && nal[0] == 0 && nal[1] == 0 && nal[2] == 0 && nal[3] == 1 {
		i = 4
	} else if len(nal) >= 3 && nal[0] == 0 && nal[1] == 0 && nal[2] == 1 {
		i = 3
	}
	if i >= len(nal) {
		return 0
	}
	return int(nal[i] & 0x1F)
}

func startFFmpeg() (*exec.Cmd, io.WriteCloser, io.ReadCloser, error) {
	cmd := exec.Command("ffmpeg",
		"-hide_banner", "-loglevel", "warning",
		"-fflags", "nobuffer",
		"-flags", "low_delay",
		"-probesize", "32",
		"-analyzeduration", "0",
		"-f", "mjpeg",
		"-i", "pipe:0",
		"-an",
		"-c:v", "libx264",
		"-preset", "ultrafast",
		"-tune", "zerolatency",
		"-profile:v", "baseline",
		"-pix_fmt", "yuv420p",
		"-g", "15",
		"-keyint_min", "15",
		"-bf", "0",
		"-x264-params", "repeat-headers=1:scenecut=0",
		"-f", "h264",
		"pipe:1",
	)
	stdin, err := cmd.StdinPipe()
	if err != nil {
		return nil, nil, nil, err
	}
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		return nil, nil, nil, err
	}
	cmd.Stderr = os.Stderr
	if err := cmd.Start(); err != nil {
		return nil, nil, nil, err
	}
	return cmd, stdin, stdout, nil
}

func offer(ch chan []byte, b []byte) {
	select {
	case ch <- b:
	default:
		select {
		case <-ch:
		default:
		}
		select {
		case ch <- b:
		default:
		}
	}
}

func cstr(b []byte) []byte {
	for i, c := range b {
		if c == 0 {
			return b[:i]
		}
	}
	return b
}

func safeID(s string) bool {
	if s == "" || len(s) > 32 {
		return false
	}
	for i, c := range s {
		ok := c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z' || c >= '0' && c <= '9' || c == '-' || c == '_'
		if i == 0 && (c < 'A' || (c > 'Z' && c < 'a') || c > 'z') && (c < '0' || c > '9') {
			return false
		}
		if !ok {
			return false
		}
	}
	return true
}

func itoa(n int) string {
	if n == 0 {
		return "0"
	}
	var b [8]byte
	i := len(b)
	for n > 0 {
		i--
		b[i] = byte('0' + n%10)
		n /= 10
	}
	return string(b[i:])
}
