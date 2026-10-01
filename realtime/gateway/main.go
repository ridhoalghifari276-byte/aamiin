package main

import (
	"bytes"
	"crypto/subtle"
	"encoding/binary"
	"encoding/json"
	"io"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"
	"regexp"
	"strings"
	"sync"
	"time"
)

const hdrLen = 48

const (
	typeVideo     = 1
	typeAudio     = 2
	typeHeartbeat = 3
	typeControl   = 4
)

var (
	token     = []byte(env("RT_TOKEN", "X01040688"))
	rtspBase  = env("RT_RTSP", "rtsp://127.0.0.1:8554")
	mtxHTTP   = env("RT_MEDIAMTX", "http://127.0.0.1:8889")
	listenUDP = env("RT_LISTEN", ":50310")
	listenHTTP = env("RT_HTTP", ":9090")
	webRoot   = env("RT_WEB", "web")
	devRe     = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_-]{0,31}$`)
)

func env(k, def string) string {
	if v := os.Getenv(k); v != "" {
		return v
	}
	return def
}

type asm struct {
	n     int
	ts    uint32
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

type devStat struct {
	Online      bool    `json:"online"`
	FPS         float64 `json:"fps"`
	BitrateKbps float64 `json:"bitrate_kbps"`
	LossPct     float64 `json:"loss_pct"`
	JitterMs    float64 `json:"jitter_ms"`
	RttMs       int     `json:"rtt_ms"`
	RSSI        int     `json:"rssi"`
	Level       int     `json:"level"`
	Frames      uint64  `json:"frames"`
	Dropped     uint64  `json:"dropped"`
	Audio       uint64  `json:"audio_packets"`
	HeapKB      int     `json:"heap_kb"`
	AgeMs       int64   `json:"age_ms"`
}

type device struct {
	mu       sync.Mutex
	asm      map[uint16]*asm
	lastDone uint16
	hasDone  bool
	lastSeq  uint16
	hasSeq   bool
	lost     uint64
	pkts     uint64
	bytes    uint64
	frames   uint64
	dropped  uint64
	audio    uint64
	jitter   float64
	lastArr  time.Time
	lastRx   time.Time
	rssi     int
	rtt      int
	level    int
	heap     int
	fpsN     int
	fpsAt    time.Time
	fps      float64
	bitN     int
	bitAt    time.Time
	bit      float64
	pub      *publisher
}

type hub struct {
	mu   sync.Mutex
	dev  map[string]*device
	ipN  map[string]int
	ipAt time.Time
}

func newHub() *hub {
	return &hub{dev: map[string]*device{}, ipN: map[string]int{}}
}

func (h *hub) allow(ip string, okToken bool) bool {
	h.mu.Lock()
	defer h.mu.Unlock()
	now := time.Now()
	if now.Sub(h.ipAt) > time.Second {
		h.ipN = map[string]int{}
		h.ipAt = now
	}
	h.ipN[ip]++
	limit := 30
	if okToken {
		limit = 2500
	}
	return h.ipN[ip] <= limit
}

func (h *hub) get(id string) *device {
	h.mu.Lock()
	defer h.mu.Unlock()
	d := h.dev[id]
	if d == nil {
		d = &device{asm: map[uint16]*asm{}, fpsAt: time.Now(), bitAt: time.Now()}
		h.dev[id] = d
	}
	return d
}

func newer(a, b uint16) bool { return int16(a-b) > 0 }

func (d *device) noteSeq(seq uint16, n int) {
	now := time.Now()
	if d.hasSeq {
		gap := int(int16(seq - d.lastSeq))
		if gap > 1 && gap < 200 {
			d.lost += uint64(gap - 1)
		}
		if !d.lastArr.IsZero() {
			dt := now.Sub(d.lastArr).Seconds() * 1000
			if dt > 0 && dt < 500 {
				inst := dt
				if d.jitter == 0 {
					d.jitter = inst
				} else {
					diff := inst - d.jitter
					if diff < 0 {
						diff = -diff
					}
					d.jitter += (diff - d.jitter) / 16
				}
			}
		}
	}
	d.hasSeq = true
	d.lastSeq = seq
	d.lastArr = now
	d.lastRx = now
	d.pkts++
	d.bytes += uint64(n)
	if now.Sub(d.bitAt) >= time.Second {
		d.bit = float64(d.bitN*8) / 1000
		d.bitN = 0
		d.bitAt = now
	}
	d.bitN += n
}

func (d *device) pushFrame(id string, jpeg []byte) {
	d.frames++
	d.fpsN++
	if time.Since(d.fpsAt) >= time.Second {
		d.fps = float64(d.fpsN) / time.Since(d.fpsAt).Seconds()
		d.fpsN = 0
		d.fpsAt = time.Now()
	}
	if d.pub == nil || d.pub.dead() {
		p, err := startPublisher(id)
		if err != nil {
			log.Printf("[pub] %s: %v", id, err)
			d.dropped++
			return
		}
		d.pub = p
	}
	if !d.pub.pushVideo(jpeg) {
		d.dropped++
	}
}

func (h *hub) onPacket(addr *net.UDPAddr, buf []byte) {
	if len(buf) < hdrLen || string(buf[:4]) != "BCRT" || buf[4] != 1 {
		return
	}
	tok := buf[36:48]
	padded := make([]byte, 12)
	copy(padded, token)
	if subtle.ConstantTimeCompare(tok, padded) != 1 {
		h.allow(addr.IP.String(), false)
		return
	}
	if !h.allow(addr.IP.String(), true) {
		return
	}
	id := cstr(buf[20:36])
	if !devRe.MatchString(id) {
		return
	}
	typ := buf[5]
	nchunks := int(buf[7])
	seq := binary.LittleEndian.Uint16(buf[8:10])
	ts := binary.LittleEndian.Uint32(buf[10:14])
	frame := binary.LittleEndian.Uint16(buf[14:16])
	plen := int(binary.LittleEndian.Uint16(buf[16:18]))
	idx := int(buf[18])
	if plen < 0 || hdrLen+plen > len(buf) || nchunks < 1 || nchunks > 8 || idx >= nchunks {
		return
	}
	payload := append([]byte(nil), buf[hdrLen:hdrLen+plen]...)
	d := h.get(id)
	d.mu.Lock()
	defer d.mu.Unlock()
	d.noteSeq(seq, len(buf))
	switch typ {
	case typeHeartbeat:
		if len(payload) >= 8 {
			d.rssi = int(int8(payload[0]))
			d.level = int(payload[2])
			d.heap = int(binary.LittleEndian.Uint16(payload[4:6]))
			d.rtt = int(binary.LittleEndian.Uint16(payload[6:8]))
		}
		hint := byte(0)
		loss := 0.0
		if d.pkts > 0 {
			loss = float64(d.lost) / float64(d.pkts+d.lost) * 100
		}
		if loss > 8 || d.rssi < -75 {
			hint = 2
		} else if loss > 3 || d.rssi < -67 {
			hint = 1
		}
		replyControl(addr, hint)
	case typeAudio:
		d.audio++
		if d.pub != nil && !d.pub.dead() {
			d.pub.pushAudio(payload)
		}
	case typeVideo:
		if d.hasDone && !newer(frame, d.lastDone) && frame != d.lastDone {
			d.dropped++
			return
		}
		for idFrame, old := range d.asm {
			if newer(frame, idFrame) {
				delete(d.asm, idFrame)
				d.dropped++
			} else if time.Since(old.at) > 100*time.Millisecond {
				delete(d.asm, idFrame)
				d.dropped++
			}
		}
		a := d.asm[frame]
		if a == nil || a.n != nchunks {
			a = &asm{n: nchunks, ts: ts, parts: make([][]byte, nchunks), at: time.Now()}
			d.asm[frame] = a
		}
		if a.parts[idx] == nil {
			a.parts[idx] = payload
		}
		if a.ready() {
			jpeg := bytes.Join(a.parts, nil)
			delete(d.asm, frame)
			if len(jpeg) < 128 || jpeg[0] != 0xFF || jpeg[1] != 0xD8 || jpeg[len(jpeg)-2] != 0xFF || jpeg[len(jpeg)-1] != 0xD9 {
				d.dropped++
				return
			}
			d.lastDone = frame
			d.hasDone = true
			d.pushFrame(id, jpeg)
		}
	}
}

func cstr(b []byte) string {
	if i := bytes.IndexByte(b, 0); i >= 0 {
		b = b[:i]
	}
	return string(b)
}

var ingest *net.UDPConn

func replyControl(addr *net.UDPAddr, hint byte) {
	if ingest == nil {
		return
	}
	pkt := make([]byte, hdrLen+1)
	copy(pkt, []byte("BCRT"))
	pkt[4] = 1
	pkt[5] = typeControl
	pkt[6] = 1
	pkt[7] = 1
	binary.LittleEndian.PutUint16(pkt[16:18], 1)
	pkt[hdrLen] = hint
	_, _ = ingest.WriteToUDP(pkt, addr)
}

type publisher struct {
	cmd   *exec.Cmd
	video chan []byte
	audio chan []byte
	gone  bool
	mu    sync.Mutex
}

func (p *publisher) dead() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.gone
}

func (p *publisher) pushVideo(b []byte) bool {
	select {
	case p.video <- b:
		return true
	default:
		select {
		case <-p.video:
		default:
		}
		select {
		case p.video <- b:
			return true
		default:
			return false
		}
	}
}

func (p *publisher) pushAudio(b []byte) {
	select {
	case p.audio <- b:
	default:
	}
}

func startPublisher(device string) (*publisher, error) {
	if _, err := exec.LookPath("ffmpeg"); err != nil {
		return nil, err
	}
	vr, vw, err := os.Pipe()
	if err != nil {
		return nil, err
	}
	ar, aw, err := os.Pipe()
	if err != nil {
		vr.Close()
		vw.Close()
		return nil, err
	}
	url := strings.TrimRight(rtspBase, "/") + "/" + device
	cmd := exec.Command("ffmpeg",
		"-hide_banner", "-loglevel", "warning",
		"-fflags", "nobuffer", "-flags", "low_delay",
		"-probesize", "32", "-analyzeduration", "0",
		"-f", "mjpeg", "-framerate", "15", "-i", "pipe:0",
		"-fflags", "nobuffer", "-f", "s16le", "-ar", "16000", "-ac", "1", "-i", "pipe:3",
		"-map", "0:v:0", "-map", "1:a:0",
		"-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
		"-profile:v", "baseline", "-pix_fmt", "yuv420p",
		"-bf", "0", "-g", "15", "-keyint_min", "15", "-sc_threshold", "0",
		"-c:a", "libopus", "-b:a", "32k", "-application", "lowdelay",
		"-af", "aresample=async=1000",
		"-muxdelay", "0", "-muxpreload", "0",
		"-f", "rtsp", "-rtsp_transport", "tcp", url,
	)
	cmd.Stdin = vr
	cmd.ExtraFiles = []*os.File{ar}
	cmd.Stderr = os.Stderr
	if err := cmd.Start(); err != nil {
		vr.Close()
		vw.Close()
		ar.Close()
		aw.Close()
		return nil, err
	}
	vr.Close()
	ar.Close()
	p := &publisher{cmd: cmd, video: make(chan []byte, 1), audio: make(chan []byte, 6)}
	go writeAll(vw, p.video)
	go writeAll(aw, p.audio)
	go func() {
		err := cmd.Wait()
		p.mu.Lock()
		p.gone = true
		p.mu.Unlock()
		log.Printf("[pub] %s ffmpeg exit: %v", device, err)
	}()
	p.pushAudio(make([]byte, 640))
	log.Printf("[pub] %s → %s", device, url)
	return p, nil
}

func writeAll(w *os.File, ch <-chan []byte) {
	defer w.Close()
	for b := range ch {
		if _, err := w.Write(b); err != nil {
			return
		}
	}
}

func (h *hub) stats() map[string]devStat {
	h.mu.Lock()
	ids := make([]string, 0, len(h.dev))
	for id := range h.dev {
		ids = append(ids, id)
	}
	h.mu.Unlock()
	out := map[string]devStat{}
	for _, id := range ids {
		d := h.get(id)
		d.mu.Lock()
		loss := 0.0
		if d.pkts+d.lost > 0 {
			loss = float64(d.lost) / float64(d.pkts+d.lost) * 100
		}
		age := int64(0)
		if !d.lastRx.IsZero() {
			age = time.Since(d.lastRx).Milliseconds()
		}
		out[id] = devStat{
			Online:      age >= 0 && age < 4000,
			FPS:         round1(d.fps),
			BitrateKbps: round1(d.bit),
			LossPct:     round1(loss),
			JitterMs:    round1(d.jitter),
			RttMs:       d.rtt,
			RSSI:        d.rssi,
			Level:       d.level,
			Frames:      d.frames,
			Dropped:     d.dropped,
			Audio:       d.audio,
			HeapKB:      d.heap,
			AgeMs:       age,
		}
		d.mu.Unlock()
	}
	return out
}

func round1(v float64) float64 { return float64(int(v*10+0.5)) / 10 }

func main() {
	h := newHub()
	udpAddr, err := net.ResolveUDPAddr("udp", listenUDP)
	if err != nil {
		log.Fatal(err)
	}
	sock, err := net.ListenUDP("udp", udpAddr)
	if err != nil {
		log.Fatal(err)
	}
	ingest = sock
	log.Printf("[udp] %s", listenUDP)
	go func() {
		buf := make([]byte, 2048)
		for {
			n, addr, err := sock.ReadFromUDP(buf)
			if err != nil {
				log.Printf("[udp] %v", err)
				time.Sleep(200 * time.Millisecond)
				continue
			}
			h.onPacket(addr, buf[:n])
		}
	}()

	mux := http.NewServeMux()
	mux.HandleFunc("/health", func(w http.ResponseWriter, r *http.Request) {
		w.Write([]byte("ok"))
	})
	mux.HandleFunc("/api/stats", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		_ = json.NewEncoder(w).Encode(map[string]any{"devices": h.stats()})
	})
	mux.HandleFunc("/api/whep/", func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost {
			http.Error(w, "post", http.StatusMethodNotAllowed)
			return
		}
		device := strings.TrimPrefix(r.URL.Path, "/api/whep/")
		if !devRe.MatchString(device) {
			http.Error(w, "device", http.StatusBadRequest)
			return
		}
		body, err := io.ReadAll(io.LimitReader(r.Body, 1<<16))
		if err != nil {
			http.Error(w, "body", http.StatusBadRequest)
			return
		}
		req, err := http.NewRequest(http.MethodPost, strings.TrimRight(mtxHTTP, "/")+"/"+device+"/whep", bytes.NewReader(body))
		if err != nil {
			http.Error(w, "req", http.StatusBadGateway)
			return
		}
		req.Header.Set("Content-Type", "application/sdp")
		resp, err := http.DefaultClient.Do(req)
		if err != nil {
			http.Error(w, "mediamtx", http.StatusBadGateway)
			return
		}
		defer resp.Body.Close()
		w.Header().Set("Content-Type", "application/sdp")
		w.WriteHeader(resp.StatusCode)
		_, _ = io.Copy(w, resp.Body)
	})
	mux.Handle("/", http.FileServer(http.Dir(webRoot)))
	log.Printf("[http] %s", listenHTTP)
	log.Fatal(http.ListenAndServe(listenHTTP, mux))
}
