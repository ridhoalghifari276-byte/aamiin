from fastapi import FastAPI, Request, Header, HTTPException, WebSocket, WebSocketDisconnect, File, Form, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse, JSONResponse, FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pathlib import Path
from collections import defaultdict, deque
from dataclasses import dataclass, field
import asyncio, time, os, wave, struct, subprocess, threading, uuid, json, queue
import shutil

import cv2
import numpy as np

from faces import FaceEngine
import pqtalkie

app = FastAPI(title="ESP32-S3 Bodycam Gateway")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["GET", "HEAD", "OPTIONS"],
    allow_headers=["Authorization", "X-Api-Token", "Content-Type"],
)
BASE = Path(os.getenv("DATA_DIR", "/data"))
BASE.mkdir(parents=True, exist_ok=True)
STATIC = Path(__file__).resolve().parent / "static"
TOKEN = os.getenv("BODYCAM_TOKEN", "CHANGE_ME")
# Separate from the device ingest token. Other apps use this to read live A/V.
API_TOKEN = os.getenv("BODYCAM_API_TOKEN", "").strip()


def _parse_device_api_tokens() -> dict[str, str]:
    raw = os.getenv("BODYCAM_API_TOKENS", "").strip()
    if not raw:
        return {}
    try:
        if raw.startswith("{"):
            data = json.loads(raw)
            return {
                str(k).strip(): str(v).strip()
                for k, v in data.items()
                if str(k).strip() and str(v).strip()
            }
    except Exception:
        pass
    out: dict[str, str] = {}
    for part in raw.split(","):
        part = part.strip()
        if ":" not in part:
            continue
        key, val = part.split(":", 1)
        key, val = key.strip(), val.strip()
        if key and val:
            out[key] = val
    return out


DEVICE_API_TOKENS = _parse_device_api_tokens()
SAMPLE_RATE = 16000
# Bodycam mounted horizontally: rotate JPEG so upright (0 / 90 / 180 / 270).
CAMERA_ROTATE = int(os.getenv("CAMERA_ROTATE", "270")) % 360
if CAMERA_ROTATE not in (0, 90, 180, 270):
    CAMERA_ROTATE = 270

latest = {}
latest_ts = {}
# Live PCM in RAM: 125 ms chunks. 40 of them ≈ 5 s of slack so a browser that
# stalls for a moment resumes from real audio instead of finding it overwritten.
audio_live = defaultdict(lambda: deque(maxlen=40))
audio_raw = defaultdict(lambda: deque(maxlen=40))
audio_seq = defaultdict(int)
# Samples actually received per device, so /health can report the effective
# capture rate. A live stream well under SAMPLE_RATE means the device is
# dropping voice, which is invisible in every other measurement.
audio_count: dict[str, int] = defaultdict(int)
audio_since: dict[str, float] = {}
# Same idea for video, in bytes: the two streams share one radio, so the only
# way to judge whether video is starving voice is to know what it actually uses.
video_bytes: dict[str, int] = defaultdict(int)
video_frames: dict[str, int] = defaultdict(int)
_loop_beat = time.monotonic()


def _event_loop_watchdog():
    time.sleep(30)
    while True:
        time.sleep(5)
        if time.monotonic() - _loop_beat > 25:
            print("[watchdog] event loop stuck — exit so Docker restarts", flush=True)
            os._exit(1)


async def _arm_watchdog():
    async def beat():
        global _loop_beat
        while True:
            _loop_beat = time.monotonic()
            await asyncio.sleep(1)

    asyncio.create_task(beat())
    threading.Thread(target=_event_loop_watchdog, name="watchdog", daemon=True).start()


ingest_q: queue.Queue = queue.Queue(maxsize=48)
audio_q: queue.Queue = queue.Queue(maxsize=96)
INGEST_WORKERS = 3
lock = threading.RLock()
face_engine: FaceEngine | None = None
device_state: dict[str, dict] = {}
ONLINE_TTL = 8.0
TARGET_FPS = 20
LIVE_FPS = 20
LIVE_MAX_SIDE = 480  # 360p, keep native 4:3 / 3:4 — never crop
FACE_INTERVAL_SEC = 0.75
MAX_RECORDINGS_PER_DEVICE = 80
last_face_submit: dict[str, float] = defaultdict(float)
WS_IDLE_SEC = 45.0
MJPEG_MAX = 6
_mjpeg_live = 0
_mjpeg_gate = threading.Lock()


def ingest_loop():
    while True:
        device, data, rec_path, do_face = ingest_q.get()
        try:
            process_frame_work(device, data, rec_path, do_face)
        except Exception as e:
            print(f"[frame] ingest error: {e}", flush=True)


def audio_loop():
    """PCM voice-chain off the asyncio loop — multi-cam must not stall WS accept."""
    while True:
        device, data, sid, mode = audio_q.get()
        try:
            append_pcm(device, data, sid, mode)
        except Exception as e:
            print(f"[audio] ingest error: {e}", flush=True)


def enqueue_pcm(
    device: str,
    data: bytes,
    session_id: str | None = None,
    session_mode: str | None = None,
):
    item = (device, data, session_id, session_mode)
    try:
        audio_q.put_nowait(item)
    except queue.Full:
        try:
            audio_q.get_nowait()
        except queue.Empty:
            pass
        try:
            audio_q.put_nowait(item)
        except queue.Full:
            pass


def _safe_standby(device: str):
    try:
        pqtalkie.ensure_standby(device)
    except Exception as e:
        print(f"[ptt] standby {device}: {e}", flush=True)


async def _ws_until_close(websocket: WebSocket):
    try:
        while True:
            msg = await websocket.receive()
            if msg.get("type") == "websocket.disconnect":
                return
    except Exception:
        return


async def _ws_send_until_close(websocket: WebSocket, sender):
    send_task = asyncio.create_task(sender())
    watch_task = asyncio.create_task(_ws_until_close(websocket))
    try:
        await asyncio.wait(
            {send_task, watch_task},
            return_when=asyncio.FIRST_COMPLETED,
        )
    finally:
        send_task.cancel()
        watch_task.cancel()
        for t in (send_task, watch_task):
            try:
                await t
            except Exception:
                pass


def _encode_jpg(img, quality: int) -> bytes | None:
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return buf.tobytes() if ok else None


def _rotate_bgr(img, deg: int):
    deg = deg % 360
    if deg == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img


def _scale_fit(img, max_side: int):
    """Shrink so the long edge is max_side. Never crop; keep aspect ratio."""
    h, w = img.shape[:2]
    long_side = max(h, w)
    if long_side <= max_side:
        return img
    scale = max_side / float(long_side)
    nw = max(2, int(round(w * scale)) & ~1)
    nh = max(2, int(round(h * scale)) & ~1)
    return cv2.resize(img, (nw, nh), interpolation=cv2.INTER_AREA)


def rotate_jpeg(data: bytes, deg: int = CAMERA_ROTATE) -> bytes:
    """Rotate JPEG in-place for mount orientation. Returns original on failure."""
    deg = deg % 360
    if deg == 0 or not data:
        return data
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return data
    img = _rotate_bgr(img, deg)
    return _encode_jpg(img, 78) or data


_nv_clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))


def nightvision_bgr(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    boosted = _nv_clahe.apply(gray)
    nv = np.zeros_like(img)
    nv[:, :, 1] = boosted
    nv[:, :, 0] = (boosted.astype(np.uint16) * 40 // 255).astype(np.uint8)
    return nv


def nightvision_jpeg(data: bytes) -> bytes:
    """Boost dark frames (CLAHE) and tint green — night-scene visibility."""
    if not data:
        return data
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return data
    return _encode_jpg(nightvision_bgr(img), 70) or data


@dataclass
class Session:
    session_id: str
    device: str
    mode: str  # audio | video
    started: float = field(default_factory=time.time)
    ended: float | None = None
    pcm: bytearray = field(default_factory=bytearray)
    frame_count: int = 0
    frames_dir: Path | None = None
    max_pcm_bytes: int = SAMPLE_RATE * 2 * 60 * 10  # 10 min mono s16le
    max_frames: int = TARGET_FPS * 60 * 8  # ~8 min @ target fps on disk


# Active + finished metadata (finished keep file path)
sessions: dict[str, Session] = {}
device_active: dict[str, str] = {}  # device -> session_id
recordings_index: list[dict] = []


def persist_recordings_index():
    idx_path = BASE / "recordings.json"
    idx_path.write_text(json.dumps(recordings_index, indent=2))


def trim_recordings_index():
    by_dev: dict[str, list] = defaultdict(list)
    for r in recordings_index:
        by_dev[str(r.get("device") or "_")].append(r)
    out: list[dict] = []
    for items in by_dev.values():
        items.sort(key=lambda r: float(r.get("ended") or r.get("started") or 0), reverse=True)
        out.extend(items[:MAX_RECORDINGS_PER_DEVICE])
    out.sort(key=lambda r: float(r.get("ended") or r.get("started") or 0), reverse=True)
    recordings_index[:] = out


def recover_recordings_from_disk():
    """Rebuild missing index rows from per-device recording folders."""
    def rel_data(p: Path) -> str:
        return str(p.relative_to(BASE)).replace("\\", "/")

    by_sid: dict[str, dict] = {}
    for item in recordings_index:
        sid = item.get("session_id")
        if sid:
            by_sid[str(sid)] = item
    for rec_dir in BASE.glob("*/recordings"):
        if not rec_dir.is_dir():
            continue
        device = rec_dir.parent.name
        grouped: dict[str, dict] = {}
        for f in rec_dir.iterdir():
            if not f.is_file():
                continue
            suf = f.suffix.lower()
            if suf not in (".mp4", ".m4a", ".wav", ".jpg", ".jpeg"):
                continue
            grouped.setdefault(f.stem, {})[suf] = f
        for sid, parts in grouped.items():
            item = by_sid.get(sid)
            if not item:
                src = parts.get(".mp4") or parts.get(".m4a") or parts.get(".wav")
                if not src:
                    continue
                st = src.stat()
                item = {
                    "session_id": sid,
                    "device": device,
                    "mode": "video" if ".mp4" in parts else "audio",
                    "started": st.st_mtime,
                    "ended": st.st_mtime,
                    "duration_sec": 0,
                    "pcm_bytes": 0,
                    "frames": 0,
                    "wav": None,
                    "m4a": None,
                    "mp4": None,
                    "thumb": None,
                }
                by_sid[sid] = item
            if ".mp4" in parts:
                item["mp4"] = rel_data(parts[".mp4"])
                item["mode"] = "video"
            if ".m4a" in parts:
                item["m4a"] = rel_data(parts[".m4a"])
            if ".wav" in parts:
                item["wav"] = rel_data(parts[".wav"])
            thumb = parts.get(".jpg") or parts.get(".jpeg")
            if thumb:
                item["thumb"] = rel_data(thumb)
            if not item.get("device"):
                item["device"] = device
    recordings_index[:] = sorted(
        by_sid.values(),
        key=lambda r: float(r.get("ended") or r.get("started") or 0),
        reverse=True,
    )
    trim_recordings_index()


def recording_thumb_file(item: dict) -> Path | None:
    sid = str(item.get("session_id") or "").replace("/", "_")
    device = item.get("device")
    if not sid or not device:
        return None
    return BASE / device / "recordings" / f"{sid}.jpg"


def attach_existing_thumb(item: dict) -> str | None:
    rel = item.get("thumb")
    if rel:
        p = BASE / rel
        if p.is_file() and p.stat().st_size > 32:
            return str(p.relative_to(BASE)).replace("\\", "/")
    p = recording_thumb_file(item)
    if p and p.is_file() and p.stat().st_size > 32:
        rel = str(p.relative_to(BASE)).replace("\\", "/")
        item["thumb"] = rel
        return rel
    return None


def extract_thumb_from_mp4(item: dict) -> str | None:
    got = attach_existing_thumb(item)
    if got:
        return got
    mp4_rel = item.get("mp4")
    dest = recording_thumb_file(item)
    if not mp4_rel or not dest:
        return None
    mp4 = BASE / mp4_rel
    if not mp4.is_file():
        return None
    dest.parent.mkdir(parents=True, exist_ok=True)
    dur = float(item.get("duration_sec") or 1.0)
    ss = max(0.0, min(2.0, dur * 0.15))
    try:
        r = subprocess.run(
            [
                "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
                "-ss", f"{ss:.3f}",
                "-i", str(mp4),
                "-frames:v", "1",
                "-q:v", "4",
                str(dest),
            ],
            check=False,
            timeout=30,
            capture_output=True,
            text=True,
        )
        if r.returncode == 0 and dest.is_file() and dest.stat().st_size > 32:
            rel = str(dest.relative_to(BASE)).replace("\\", "/")
            item["thumb"] = rel
            persist_recordings_index()
            return rel
    except Exception as e:
        print(f"[thumb] extract fail: {e}", flush=True)
    return None


def auth(device, token):
    if not device or token != TOKEN:
        raise HTTPException(401, "unauthorized")


def _bearer(authorization: str | None) -> str:
    if authorization and authorization.lower().startswith("bearer "):
        return authorization[7:].strip()
    return ""


def _incoming_api_token(
    token: str | None = None,
    x_api_token: str | None = None,
    authorization: str | None = None,
) -> str:
    return (token or x_api_token or _bearer(authorization) or "").strip()


def resolve_api_scope(token: str, device: str | None = None) -> str:
    """Return 'all' or a device id. External apps only — never the ESP ingest token."""
    token = (token or "").strip()
    if not token:
        raise HTTPException(401, "missing api token")
    if API_TOKEN and token == API_TOKEN:
        return "all"
    bound = next((d for d, t in DEVICE_API_TOKENS.items() if t == token), None)
    if not bound:
        raise HTTPException(401, "unauthorized")
    if device and device != bound:
        raise HTTPException(403, "token not valid for this device")
    return bound


def ext_unit(scope: str, device: str | None) -> str:
    if scope != "all":
        return scope
    unit = (device or "").strip()
    if not unit:
        raise HTTPException(400, "device required")
    return unit


def write_wav(path: Path, pcm: bytes, rate: int = SAMPLE_RATE):
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(pcm)


def encode_m4a(pcm: bytes, dest: Path) -> str | None:
    """Mux s16le mono PCM to AAC M4A so browsers can play/download audio recs."""
    if not pcm or len(pcm) < 256:
        return None
    tmp = dest.with_suffix(".pcm")
    try:
        tmp.write_bytes(pcm)
        cmd = [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-f", "s16le", "-ar", str(SAMPLE_RATE), "-ac", "1",
            "-i", str(tmp),
            "-c:a", "aac", "-b:a", "96k", "-ar", str(SAMPLE_RATE), "-ac", "1",
            "-movflags", "+faststart",
            str(dest),
        ]
        r = subprocess.run(cmd, check=False, timeout=60, capture_output=True, text=True)
        if r.returncode == 0 and dest.is_file() and dest.stat().st_size > 64:
            return str(dest.relative_to(BASE)).replace("\\", "/")
        print(f"[m4a] fail {dest.name}: {(r.stderr or r.stdout or r.returncode)}", flush=True)
    except Exception as e:
        print(f"[m4a] exception {dest.name}: {e}", flush=True)
    finally:
        try:
            tmp.unlink(missing_ok=True)
        except Exception:
            pass
    return None


def finalize_session(sess: Session) -> dict:
    """Flush in-memory session to disk (WAV / M4A and optional MP4)."""
    if sess.ended is not None:
        for item in recordings_index:
            if item.get("session_id") == sess.session_id:
                return item
        return {
            "session_id": sess.session_id,
            "device": sess.device,
            "mode": sess.mode,
            "ended": sess.ended,
            "wav": None,
            "m4a": None,
            "mp4": None,
        }
    sess.ended = time.time()
    out_dir = BASE / sess.device / "recordings"
    out_dir.mkdir(parents=True, exist_ok=True)
    safe = sess.session_id.replace("/", "_")
    pcm = bytes(sess.pcm) if sess.pcm else b""
    wall = max(0.01, (sess.ended or time.time()) - sess.started)
    pcm_dur = len(pcm) / (SAMPLE_RATE * 2.0) if pcm else 0.0
    duration = pcm_dur if sess.mode == "audio" and pcm_dur > 0.05 else wall
    meta = {
        "session_id": sess.session_id,
        "device": sess.device,
        "mode": sess.mode,
        "started": sess.started,
        "ended": sess.ended,
        "duration_sec": round(duration, 2),
        "pcm_bytes": len(pcm),
        "frames": sess.frame_count,
        "wav": None,
        "m4a": None,
        "mp4": None,
        "thumb": None,
    }

    wav_path = out_dir / f"{safe}.wav"
    write_wav(wav_path, pcm if pcm else b"\x00\x00")
    meta["wav"] = str(wav_path.relative_to(BASE)).replace("\\", "/")
    m4a_rel = encode_m4a(pcm, out_dir / f"{safe}.m4a")
    if m4a_rel:
        meta["m4a"] = m4a_rel

    thumb_path = out_dir / f"{safe}.jpg"

    def _copy_mid_frame():
        if not sess.frames_dir or not sess.frames_dir.is_dir():
            return False
        frames = sorted(sess.frames_dir.glob("*.jpg"))
        if not frames:
            return False
        src = frames[min(len(frames) // 2, len(frames) - 1)]
        try:
            shutil.copyfile(src, thumb_path)
            return thumb_path.is_file() and thumb_path.stat().st_size > 32
        except Exception as e:
            print(f"[thumb] copy fail {safe}: {e}", flush=True)
            return False

    def _thumb_from_mp4(mp4_file: Path):
        if not mp4_file.is_file():
            return False
        ss = max(0.0, min(2.0, duration * 0.15))
        cmd = [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-ss", f"{ss:.3f}",
            "-i", str(mp4_file),
            "-frames:v", "1",
            "-q:v", "4",
            str(thumb_path),
        ]
        try:
            r = subprocess.run(cmd, check=False, timeout=30, capture_output=True, text=True)
            return r.returncode == 0 and thumb_path.is_file() and thumb_path.stat().st_size > 32
        except Exception as e:
            print(f"[thumb] ffmpeg fail {safe}: {e}", flush=True)
            return False

    if sess.mode == "video":
        _copy_mid_frame()

    if sess.mode == "video" and sess.frames_dir and sess.frame_count:
        pcm_path = out_dir / f"{safe}.pcm"
        need = int(duration * SAMPLE_RATE * 2)
        pcm_mux = bytes(sess.pcm)
        if len(pcm_mux) < need:
            pcm_mux += b"\x00" * (need - len(pcm_mux))
        pcm_path.write_bytes(pcm_mux if pcm_mux else b"\x00\x00")
        mp4_path = out_dir / f"{safe}.mp4"
        fps = max(5.0, min(30.0, sess.frame_count / duration))
        cmd = [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-fflags", "+genpts",
            "-f", "image2",
            "-framerate", f"{fps:.4f}",
            "-start_number", "0",
            "-i", str(sess.frames_dir / "%06d.jpg"),
            "-f", "s16le", "-ar", str(SAMPLE_RATE), "-ac", "1",
            "-i", str(pcm_path),
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-pix_fmt", "yuv420p",
            "-profile:v", "baseline",
            "-level", "3.1",
            "-c:a", "aac",
            "-b:a", "96k",
            "-ar", "16000",
            "-ac", "1",
            "-t", f"{duration:.3f}",
            "-movflags", "+faststart",
            str(mp4_path),
        ]
        try:
            r = subprocess.run(cmd, check=False, timeout=180, capture_output=True, text=True)
            if r.returncode == 0 and mp4_path.is_file() and mp4_path.stat().st_size > 64:
                meta["mp4"] = str(mp4_path.relative_to(BASE)).replace("\\", "/")
            else:
                meta["mp4_error"] = (r.stderr or r.stdout or f"ffmpeg exit {r.returncode}")[-800:]
                print(f"[mux] fail {safe}: {meta['mp4_error']}", flush=True)
        except Exception as e:
            meta["mp4_error"] = str(e)
            print(f"[mux] exception {safe}: {e}", flush=True)
        try:
            pcm_path.unlink(missing_ok=True)
        except Exception:
            pass
        if not (thumb_path.is_file() and thumb_path.stat().st_size > 32) and mp4_path.is_file():
            _thumb_from_mp4(mp4_path)
        try:
            shutil.rmtree(sess.frames_dir, ignore_errors=True)
        except Exception:
            pass

    if thumb_path.is_file() and thumb_path.stat().st_size > 32:
        meta["thumb"] = str(thumb_path.relative_to(BASE)).replace("\\", "/")

    sess.pcm = bytearray()
    sess.frame_count = 0
    sess.frames_dir = None
    recordings_index[:] = [r for r in recordings_index if r.get("session_id") != sess.session_id]
    recordings_index.insert(0, meta)
    trim_recordings_index()
    persist_recordings_index()
    return meta


def touch_device(device: str):
    latest_ts[device] = time.time()


def online_ids() -> list[str]:
    now = time.time()
    ids = set()
    for d, t in latest_ts.items():
        if now - t <= ONLINE_TTL:
            ids.add(d)
    for d, st in device_state.items():
        if now - float(st.get("ts") or 0) <= ONLINE_TTL:
            ids.add(d)
    return sorted(ids)


def _session_active(sid: str | None) -> Session | None:
    if not sid:
        return None
    sess = sessions.get(sid)
    if sess and sess.ended is None:
        return sess
    return None


def ensure_record_session(device: str, session_id: str | None, mode: str | None) -> Session | None:
    """Keep PCM even if session/start HTTP is late. Prefer the UI/device active session."""
    sess = _session_active(device_active.get(device))
    if sess:
        return sess
    if not session_id:
        return None
    sess = _session_active(session_id)
    if sess:
        device_active[device] = session_id
        return sess
    if session_id in sessions:
        return None
    mode = (mode or "audio").lower()
    if mode not in ("audio", "video"):
        mode = "audio"
    sess = Session(session_id=session_id, device=device, mode=mode)
    if mode == "video":
        frames_dir = BASE / device / "recordings" / f"{session_id.replace('/', '_')}_frames"
        frames_dir.mkdir(parents=True, exist_ok=True)
        sess.frames_dir = frames_dir
    sessions[session_id] = sess
    device_active[device] = session_id
    return sess


class VoiceChain:
    """Outdoor bodycam voice: cut wind rumble, keep speech, duck noise.

    Hard-zero + OPEN_RMS=1000 swallowed quiet speech and chopped words.
    Outdoor running needs the opposite: a ~100 Hz high-pass, a low open
    threshold, hang between syllables, and a floor duck (not digital mute).
    """

    BLOCK = 320
    OPEN_RMS = 220.0
    HANG_BLOCKS = 12  # 240 ms covers gaps between words
    TARGET = 4800.0
    HP_R = 0.961  # ~100 Hz at 16 kHz
    CEILING = 26000.0
    NOISE_DUCK = 0.12

    def __init__(self, rate: int = SAMPLE_RATE):
        self.pending = np.zeros(0, dtype=np.float32)
        self.hp_x = 0.0
        self.hp_y = 0.0
        self.peak = 0.0
        self.hang = 0
        self.gate = 1.0
        self.gain = 2.2
        self.duck = 1.0
        self.speech = 0.0
        self.floor = 80.0

    def process(self, pcm: bytes) -> bytes:
        if not pcm or len(pcm) < 4:
            return pcm
        x = np.frombuffer(pcm, dtype="<i2")
        if x.size == 0:
            return pcm
        y = self._hpf(x.astype(np.float32))
        if self.pending.size:
            y = np.concatenate((self.pending, y))
        full = (y.size // self.BLOCK) * self.BLOCK
        self.pending = y[full:].copy()
        if full == 0:
            return b""
        out = np.empty(full, dtype=np.float32)
        for i in range(0, full, self.BLOCK):
            out[i : i + self.BLOCK] = self._gate(y[i : i + self.BLOCK])
        return np.clip(out, -self.CEILING, self.CEILING).astype("<i2").tobytes()

    def _hpf(self, x: np.ndarray) -> np.ndarray:
        # First-order DC blocker. Keep state across packets.
        x = np.asarray(x, dtype=np.float32)
        y = np.empty_like(x)
        px = float(self.hp_x)
        py = float(self.hp_y)
        r = float(self.HP_R)
        # Local bindings — still O(n) but much cheaper than attribute hits per sample.
        for i in range(x.size):
            v = float(x[i])
            ny = v - px + r * py
            y[i] = ny
            px = v
            py = ny
        self.hp_x = px
        self.hp_y = py
        return y

    def _gate(self, y: np.ndarray) -> np.ndarray:
        n = y.size
        rms = float(np.sqrt(np.mean(y * y))) + 1e-6
        if rms < self.floor:
            self.floor += (rms - self.floor) * 0.18
        else:
            self.floor += (rms - self.floor) * 0.012
        self.floor = max(40.0, min(self.floor, 900.0))

        if rms > self.peak:
            self.peak = rms
        else:
            self.peak *= 0.96

        open_at = max(self.OPEN_RMS, self.floor * 2.4)
        is_speech = rms >= open_at
        if is_speech:
            self.hang = self.HANG_BLOCKS
            self.speech = rms
            want = 1.0
            want_g = min(5.5, max(1.6, self.TARGET / rms))
        elif self.hang > 0:
            self.hang -= 1
            want = 1.0
            want_g = self.gain
        else:
            want = self.NOISE_DUCK
            want_g = 1.3

        alpha = 0.55 if want > self.gate else 0.22
        gate_n = self.gate + (want - self.gate) * alpha
        self.gain += (want_g - self.gain) * 0.08
        self.duck = gate_n
        ramp = np.linspace(self.gate, gate_n, n, dtype=np.float32)
        self.gate = gate_n
        return y * ramp * self.gain


class GapFiller:
    """Replace audio the device never sent with a smooth continuation.

    The mic captures 16 kHz but only ~73 % of it reaches us, so two packets in
    a row are usually *not* adjacent audio. Splicing them end to end puts a
    step discontinuity every 320 samples, and a train of steps at the 50 Hz
    packet rate is a harmonic comb — 2350 Hz, 4700 Hz, 7050 Hz are all exact
    multiples of 50. That comb is the whine, and no filter can remove it
    because it is not something the microphone heard: it is the seam.

    Filling the hole instead keeps the stream aligned to real time, which also
    stops the browser's jitter buffer from starving permanently and coasting
    toward silence between bursts.
    """

    MIN_GAP = 0.150
    MAX_FILL = 0.150  # never manufacture more than this in one go
    RESYNC = 0.500  # clock this far ahead means a stall, not a gap
    TAIL = 160  # 10 ms of history to repeat from
    TILE_MAX = 0.060  # never repeat for longer than this, even in speech
    SPEECH_SNR = 2.5  # tail this far above the floor counts as speech
    FLOOR_WINDOW = 100  # packets, ≈ 2 s of minimum statistics
    SMOOTH = 4  # moving-average taps applied to the fill noise

    def __init__(self, rate: int = SAMPLE_RATE):
        self.rate = rate
        self.clock: float | None = None
        self.tail = np.zeros(self.TAIL, dtype=np.float32)
        self.rng = np.random.default_rng(7)
        self.smooth_k = np.ones(self.SMOOTH, dtype=np.float32) / self.SMOOTH
        self.recent: deque = deque(maxlen=self.FLOOR_WINDOW)
        self.floor = 50.0
        self.filled = 0
        self.received = 0

    def feed(self, pcm: bytes, now: float) -> bytes:
        """Pass the packet through.

        This class used to invent up to 150 ms of audio whenever a packet
        arrived a little late. After a sentence the tail is still speech, so
        that invention was a pitched tail — the ngiung that started the moment
        talking stopped. The device now sends 125 ms packets under its send
        cap, so the 20 ms-era holes this was written for are gone.
        """
        x = np.frombuffer(pcm, dtype="<i2")
        if x.size == 0:
            return pcm
        self.received += x.size
        return pcm

    def _extrapolate(self, n: int) -> np.ndarray:
        """Fill a hole with decaying room noise, never a repeated waveform.

        Tiling the last 10 ms was tried because codecs do it through speech.
        After a sentence the tail *is* speech, so the next hole became a
        pitched buzz at the talker's formant — the hum that only appeared
        after speaking. Noise at the room-floor level cannot do that.
        """
        rms = float(np.sqrt(np.mean(self.tail * self.tail)))
        if rms > self.floor * self.SPEECH_SNR:
            rms = self.floor
        noise = self.rng.standard_normal(n + self.SMOOTH - 1).astype(np.float32)
        fill = np.convolve(noise, self.smooth_k, mode="valid")[:n]
        cur = float(np.sqrt(np.mean(fill * fill))) or 1.0
        fill = (fill * (rms / cur)).astype(np.float32)
        fill *= np.linspace(1.0, 0.45, n, dtype=np.float32)
        k = min(32, n)
        if k > 1:
            w = np.linspace(0.0, 1.0, k, dtype=np.float32)
            fill[:k] = fill[:k] * w + float(self.tail[-1]) * (1.0 - w)
        return fill

    def _remember(self, x: np.ndarray):
        if x.size >= self.TAIL:
            self.tail = x[-self.TAIL :].astype(np.float32).copy()
        else:
            self.tail = np.concatenate((self.tail[x.size :], x.astype(np.float32)))
        # Minimum statistics, same reasoning as the voice chain: room noise is
        # continuous and speech is not, so the quietest packet in the last ~2 s
        # is the floor. Needed here only to tell speech from silence.
        self.recent.append(float(np.sqrt(np.mean(x.astype(np.float32) ** 2))))
        quiet = min(self.recent)
        self.floor += (quiet - self.floor) * (0.5 if quiet < self.floor else 0.05)
        if self.floor < 4.0:
            self.floor = 4.0


_voice_chains: dict[str, VoiceChain] = {}
_gap_fillers: dict[str, GapFiller] = {}


def clean_pcm(device: str, data: bytes) -> bytes:
    chain = _voice_chains.get(device)
    if chain is None:
        chain = VoiceChain()
        _voice_chains[device] = chain
    return chain.process(data)


def append_pcm(device: str, data: bytes, session_id: str | None, session_mode: str | None = None):
    now = time.time()
    got = len(data) // 2
    filler = _gap_fillers.get(device)
    if filler is None:
        filler = GapFiller()
        _gap_fillers[device] = filler
    # Conceal what never arrived before anything else looks at the stream, so
    # the voice chain and the browser both see continuous, real-time audio.
    raw = filler.feed(data, now)
    data = clean_pcm(device, raw)
    with lock:
        touch_device(device)
        audio_since.setdefault(device, now)
        audio_count[device] += got
        audio_seq[device] += 1
        audio_live[device].append((audio_seq[device], data))
        # Untouched mic samples, for telling "the mic picked this up" apart from
        # "the voice chain produced this" when diagnosing a noise complaint.
        audio_raw[device].append((audio_seq[device], raw))
        sess = ensure_record_session(device, session_id, session_mode)
        if sess and len(sess.pcm) < sess.max_pcm_bytes:
            remain = sess.max_pcm_bytes - len(sess.pcm)
            sess.pcm.extend(data[:remain])
    pqtalkie.feed_device_pcm(device, data)


def store_live_frame(device: str, data: bytes) -> float:
    """Hot path: publish the device JPEG untouched. No decode, no re-encode.

    The browser rotates with a canvas transform, so live latency is pure network.
    """
    now = time.time()
    with lock:
        latest[device] = data
        latest_ts[device] = now
        audio_since.setdefault(device, now)
        video_bytes[device] += len(data)
        video_frames[device] += 1
    return now


def claim_frame_work(device: str, now: float, session_id: str | None, session_mode: str | None):
    """Decide whether this frame needs CPU (recording to disk / face detect)."""
    rec_path = None
    need_mkdir = None
    with lock:
        sess = ensure_record_session(device, session_id, session_mode or "video")
        if sess and sess.mode == "video" and sess.frame_count < sess.max_frames:
            if not sess.frames_dir:
                frames_dir = BASE / device / "recordings" / f"{sess.session_id.replace('/', '_')}_frames"
                sess.frames_dir = frames_dir
                need_mkdir = frames_dir
            rec_path = sess.frames_dir / f"{sess.frame_count:06d}.jpg"
            sess.frame_count += 1
        do_face = face_engine is not None and (now - last_face_submit[device]) >= FACE_INTERVAL_SEC
        if do_face:
            last_face_submit[device] = now
    if need_mkdir is not None:
        try:
            need_mkdir.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            print(f"[frame] mkdir {need_mkdir}: {e}", flush=True)
            rec_path = None
    return rec_path, do_face


def process_frame_work(device: str, data: bytes, rec_path, do_face: bool):
    """Cold path: only runs while recording or when a face check is due."""
    if rec_path is None and not do_face:
        return
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return
    if CAMERA_ROTATE:
        img = _rotate_bgr(img, CAMERA_ROTATE)
    if rec_path is not None:
        rec_bytes = _encode_jpg(img, 75)
        if rec_bytes:
            rec_path.write_bytes(rec_bytes)
    if do_face and face_engine is not None:
        small = _encode_jpg(_scale_fit(img, LIVE_MAX_SIDE), 68)
        if small:
            face_engine.submit(device, small)


def append_frame(device: str, data: bytes, session_id: str | None, session_mode: str | None = None):
    now = store_live_frame(device, data)
    rec_path, do_face = claim_frame_work(device, now, session_id, session_mode)
    process_frame_work(device, data, rec_path, do_face)


@app.on_event("startup")
def load_index():
    global face_engine
    idx = BASE / "recordings.json"
    if idx.exists():
        try:
            recordings_index[:] = json.loads(idx.read_text())
        except Exception:
            pass
    try:
        recover_recordings_from_disk()
        persist_recordings_index()
    except Exception as e:
        print(f"[recordings] recover: {e}", flush=True)
    face_engine = FaceEngine(BASE)
    for i in range(INGEST_WORKERS):
        threading.Thread(target=ingest_loop, name=f"frame-ingest-{i}", daemon=True).start()
    threading.Thread(target=audio_loop, name="audio-ingest", daemon=True).start()
    print(f"[face] engine ready · frame workers={INGEST_WORKERS} · audio worker=1", flush=True)
    if API_TOKEN or DEVICE_API_TOKENS:
        print("[api] external live API enabled at /api/v1/ext", flush=True)
    else:
        print("[api] BODYCAM_API_TOKEN not set — /api/v1/ext is closed", flush=True)
    print(f"[ptt] PQTALKIE {pqtalkie.PQTALKIE_URL}", flush=True)
    pqtalkie.warm_default()


@app.on_event("startup")
async def arm_watchdog():
    await _arm_watchdog()


@app.get("/health")
def health():
    with lock:
        active = {
            d: sessions[s].mode
            for d, s in device_active.items()
            if s in sessions and sessions[s].ended is None
        }
        states = dict(device_state)
        devices = online_ids()
        now = time.time()
        audio_hz = {
            d: round(audio_count[d] / max(now - t, 1e-3), 1)
            for d, t in audio_since.items()
        }
        video_kbps = {
            d: round(video_bytes[d] * 8 / 1000.0 / max(now - t, 1e-3), 1)
            for d, t in audio_since.items()
            if video_bytes[d]
        }
        video_fps = {
            d: round(video_frames[d] / max(now - t, 1e-3), 1)
            for d, t in audio_since.items()
            if video_frames[d]
        }
    faces_n = len(face_engine.captures) if face_engine else 0
    radios = pqtalkie.status()
    for d, st in list(states.items()):
        copy = dict(st)
        r = radios.get(d) or {}
        copy["radio"] = bool(r.get("ready"))
        copy["radio_tx"] = bool(r.get("tx"))
        copy["radio_err"] = r.get("error") or ""
        copy["radio_ch"] = r.get("channelId")
        copy["radio_label"] = r.get("label") or ""
        copy["radio_token"] = bool(r.get("has_token"))
        copy["radio_rx_chunks"] = int(r.get("rx_chunks") or 0)
        copy["radio_rx_bytes"] = int(r.get("rx_bytes") or 0)
        copy["radio_rx_age_ms"] = r.get("rx_age_ms")
        copy["radio_tx_chunks"] = int(r.get("tx_chunks") or 0)
        copy["sos_err"] = r.get("sos_err") or ""
        states[d] = copy
    return {
        "ok": True,
        "devices": devices,
        "active_sessions": active,
        "device_state": states,
        "recordings": len(recordings_index),
        "faces": faces_n,
        "camera_rotate": CAMERA_ROTATE,
        "pqtalkie_url": pqtalkie.PQTALKIE_URL,
        "target_fps": TARGET_FPS,
        "live_fps": LIVE_FPS,
        "last_frame_age": {
            d: round(time.time() - t, 2) for d, t in latest_ts.items()
        },
        # Should sit at SAMPLE_RATE. Anything less is voice never captured.
        "audio_rate_hz": audio_hz,
        "audio_rate_target": SAMPLE_RATE,
        "video_kbps": video_kbps,
        "video_fps_actual": video_fps,
        # Cumulative, so a caller can difference two polls. The averages above
        # are since-connect and get meaningless once the device idles.
        "counters": {
            "t": now,
            "audio_samples": dict(audio_count),
            "video_bytes": dict(video_bytes),
            "video_frames": dict(video_frames),
            # How much of the outgoing stream had to be invented.
            "audio_filled": {d: f.filled for d, f in _gap_fillers.items()},
            "audio_received": {d: f.received for d, f in _gap_fillers.items()},
        },
    }


@app.post("/api/v1/device/state")
async def post_device_state(
    req: Request,
    x_device_id: str = Header(None),
    x_token: str = Header(None),
):
    """ESP reports stream/audio/video button state for dashboard sync."""
    auth(x_device_id, x_token)
    body = {}
    try:
        body = await req.json()
    except Exception:
        pass
    with lock:
        prev = device_state.get(x_device_id) or {}
        lat = body.get("lat")
        lon = body.get("lon")
        last_lat = lat if lat is not None else prev.get("last_lat")
        last_lon = lon if lon is not None else prev.get("last_lon")
        device_state[x_device_id] = {
            "stream": bool(body.get("stream")),
            "audio": bool(body.get("audio")),
            "video": bool(body.get("video")),
            "visual": bool(body.get("visual", body.get("stream") or body.get("video"))),
            "nightvision": bool(body.get("nightvision")),
            "ptt": bool(body.get("ptt")),
            "sos": bool(body.get("sos")),
            "rssi": body.get("rssi"),
            "ip": body.get("ip"),
            "gps": bool(body.get("gps")),
            "gps_on": bool(body.get("gps_on", True)),
            "gps_rx": bool(body.get("gps_rx")),
            "lat": lat,
            "lon": lon,
            "last_lat": last_lat,
            "last_lon": last_lon,
            "alt": body.get("alt"),
            "spd": body.get("spd"),
            "crs": body.get("crs"),
            "sats": body.get("sats"),
            "gps_age_ms": body.get("gps_age_ms"),
            "radio": False,
            "ts": time.time(),
        }
        touch_device(x_device_id)
    try:
        pqtalkie.ensure_standby(x_device_id)
        pqtalkie.set_device_tx(x_device_id, bool(body.get("ptt")))
        pqtalkie.set_device_sos(x_device_id, bool(body.get("sos")))
        if body.get("lat") is not None and body.get("lon") is not None:
            pqtalkie.set_device_location(x_device_id, body.get("lat"), body.get("lon"))
        tok = (body.get("ptt_token") or "").strip()
        if tok:
            pqtalkie.set_device_token(x_device_id, tok)
        radio_ready = bool((pqtalkie.status(x_device_id) or {}).get("ready"))
        with lock:
            if x_device_id in device_state:
                device_state[x_device_id]["radio"] = radio_ready
    except Exception as e:
        print(f"[ptt] state hook {x_device_id}: {e}", flush=True)
    return {"ok": True, "device": x_device_id, "state": device_state[x_device_id]}


@app.get("/api/v1/device/state")
def get_device_state(device: str = "bodycam-01"):
    with lock:
        st = device_state.get(device)
    radio = pqtalkie.status(device) or {}
    if not st:
        return {
            "stream": False,
            "audio": False,
            "video": False,
            "visual": False,
            "nightvision": False,
            "ptt": False,
            "sos": False,
            "gps": False,
            "gps_rx": False,
            "lat": None,
            "lon": None,
            "radio": bool(radio.get("ready")),
            "radio_tx": bool(radio.get("tx")),
            "radio_err": radio.get("error") or "",
            "radio_ch": radio.get("channelId"),
            "radio_label": radio.get("label") or "",
            "radio_token": bool(radio.get("has_token")),
            "device": device,
            "ptt_radio": radio,
        }
    return {
        **st,
        "device": device,
        "radio": bool(radio.get("ready")),
        "radio_tx": bool(radio.get("tx")),
        "radio_err": radio.get("error") or "",
        "radio_ch": radio.get("channelId"),
        "radio_label": radio.get("label") or "",
        "radio_token": bool(radio.get("has_token")),
        "ptt_radio": radio,
    }


@app.post("/api/v1/ptt-token")
async def set_ptt_token(req: Request):
    """Save the PQTALKIE kiosk token from the radio web for this bodycam."""
    body = await req.json()
    device = (body.get("device") or "").strip()
    token = (body.get("token") or "").strip()
    if not device:
        raise HTTPException(400, "device required")
    pqtalkie.set_device_token(device, token)
    return {"ok": True, "device": device, "ptt_radio": pqtalkie.status(device)}


@app.get("/api/v1/ptt")
def get_ptt(device: str | None = None):
    return {"ok": True, "ptt": pqtalkie.status(device)}


@app.post("/api/v1/frame")
async def frame(
    req: Request,
    x_device_id: str = Header(None),
    x_token: str = Header(None),
    x_session_id: str = Header(None),
    x_session_mode: str = Header(None),
):
    # Compatibility fallback. New firmware uses /ws/device instead.
    auth(x_device_id, x_token)
    data = await req.body()
    now = store_live_frame(x_device_id, data)
    rec_path, do_face = claim_frame_work(x_device_id, now, x_session_id, x_session_mode)
    if rec_path is not None or do_face:
        item = (x_device_id, data, rec_path, do_face)
        try:
            ingest_q.put_nowait(item)
        except queue.Full:
            try:
                ingest_q.get_nowait()
            except queue.Empty:
                pass
            try:
                ingest_q.put_nowait(item)
            except queue.Full:
                pass
    return {"ok": True, "bytes": len(data), "transport": "http-fallback"}


@app.post("/api/v1/audio")
async def audio(
    req: Request,
    x_device_id: str = Header(None),
    x_token: str = Header(None),
    x_session_id: str = Header(None),
    x_session_mode: str = Header(None),
):
    auth(x_device_id, x_token)
    data = await req.body()
    enqueue_pcm(x_device_id, data, x_session_id, x_session_mode)
    return {"ok": True, "bytes": len(data)}


@app.post("/api/v1/session/start")
async def session_start(
    req: Request,
    x_device_id: str = Header(None),
    x_token: str = Header(None),
    x_session_id: str = Header(None),
):
    auth(x_device_id, x_token)
    body = {}
    try:
        body = await req.json()
    except Exception:
        pass
    mode = (body.get("mode") or "audio").lower()
    if mode not in ("audio", "video"):
        raise HTTPException(400, "mode must be audio|video")
    sid = body.get("session_id") or x_session_id or f"{x_device_id}-{mode}-{uuid.uuid4().hex[:8]}"
    with lock:
        existing = sessions.get(sid)
        if existing and existing.ended is None:
            existing.mode = mode
            if mode == "video" and not existing.frames_dir:
                frames_dir = BASE / x_device_id / "recordings" / f"{sid.replace('/', '_')}_frames"
                frames_dir.mkdir(parents=True, exist_ok=True)
                existing.frames_dir = frames_dir
            device_active[x_device_id] = sid
            return {"ok": True, "session_id": sid, "mode": mode}
        old = device_active.get(x_device_id)
        if old and old != sid and old in sessions and sessions[old].ended is None:
            finalize_session(sessions[old])
        sess = Session(session_id=sid, device=x_device_id, mode=mode)
        if mode == "video":
            frames_dir = BASE / x_device_id / "recordings" / f"{sid.replace('/', '_')}_frames"
            frames_dir.mkdir(parents=True, exist_ok=True)
            sess.frames_dir = frames_dir
        sessions[sid] = sess
        device_active[x_device_id] = sid
    return {"ok": True, "session_id": sid, "mode": mode}


@app.post("/api/v1/session/stop")
async def session_stop(
    req: Request,
    x_device_id: str = Header(None),
    x_token: str = Header(None),
    x_session_id: str = Header(None),
):
    auth(x_device_id, x_token)
    body = {}
    try:
        body = await req.json()
    except Exception:
        pass
    sid = body.get("session_id") or x_session_id or device_active.get(x_device_id)
    if not sid or sid not in sessions:
        return {"ok": False, "error": "no session"}
    with lock:
        meta = finalize_session(sessions[sid])
        if device_active.get(x_device_id) == sid:
            device_active.pop(x_device_id, None)
    return {"ok": True, "recording": meta}


@app.post("/api/v1/record/start")
async def ui_record_start(device: str = "bodycam-01", mode: str = "audio"):
    """Start in-memory recording from UI for a live device (no device token)."""
    mode = mode.lower()
    if mode not in ("audio", "video"):
        raise HTTPException(400, "mode must be audio|video")
    sid = f"{device}-ui-{mode}-{uuid.uuid4().hex[:8]}"
    with lock:
        old = device_active.get(device)
        if old and old in sessions and sessions[old].ended is None:
            finalize_session(sessions[old])
        sess = Session(session_id=sid, device=device, mode=mode)
        if mode == "video":
            frames_dir = BASE / device / "recordings" / f"{sid.replace('/', '_')}_frames"
            frames_dir.mkdir(parents=True, exist_ok=True)
            sess.frames_dir = frames_dir
        sessions[sid] = sess
        device_active[device] = sid
    return {"ok": True, "session_id": sid, "mode": mode}


@app.post("/api/v1/record/stop")
async def ui_record_stop(device: str = "bodycam-01"):
    with lock:
        sid = device_active.get(device)
        if not sid or sid not in sessions:
            return {"ok": False, "error": "no active recording"}
        meta = finalize_session(sessions[sid])
        device_active.pop(device, None)
    return {"ok": True, "recording": meta}


@app.get("/api/v1/recordings")
def list_recordings(device: str | None = None):
    with lock:
        items = list(recordings_index)
    devices = sorted({str(r.get("device")) for r in items if r.get("device")})
    if device:
        items = [r for r in items if r.get("device") == device]
    for r in items:
        if r.get("mode") == "video":
            attach_existing_thumb(r)
    with lock:
        active = None
        sid = device_active.get(device) if device else None
        if sid and sid in sessions and sessions[sid].ended is None:
            s = sessions[sid]
            active = {
                "session_id": s.session_id,
                "mode": s.mode,
                "elapsed_sec": round(time.time() - s.started, 1),
                "pcm_bytes": len(s.pcm),
                "frames": s.frame_count,
            }
    return {"ok": True, "active": active, "items": items, "devices": devices}


@app.get("/api/v1/recordings/file")
def get_recording_file(path: str, download: int = 0):
    """Serve a recording file. download=1 forces attachment + filename."""
    rel = Path(path)
    if rel.is_absolute() or ".." in rel.parts:
        raise HTTPException(400, "invalid path")
    full = (BASE / rel).resolve()
    if not str(full).startswith(str(BASE.resolve())) or not full.exists():
        raise HTTPException(404, "not found")
    suffix = full.suffix.lower()
    media = {
        ".wav": "audio/wav",
        ".m4a": "audio/mp4",
        ".aac": "audio/aac",
        ".mp4": "video/mp4",
        ".jpg": "image/jpeg",
        ".jpeg": "image/jpeg",
    }.get(suffix, "application/octet-stream")
    headers = {
        "Accept-Ranges": "bytes",
        "Cache-Control": "no-store",
    }
    if download:
        return FileResponse(
            full,
            media_type=media,
            filename=full.name,
            content_disposition_type="attachment",
            headers=headers,
        )
    return FileResponse(full, media_type=media, headers=headers)


@app.get("/api/v1/recordings/thumb")
def get_recording_thumb(session_id: str):
    """JPEG still from a video recording (YouTube-style thumbnail)."""
    with lock:
        item = next((r for r in recordings_index if r.get("session_id") == session_id), None)
    if not item:
        raise HTTPException(404, "not found")
    rel = extract_thumb_from_mp4(item)
    if not rel:
        raise HTTPException(404, "no thumbnail")
    full = (BASE / rel).resolve()
    if not str(full).startswith(str(BASE.resolve())) or not full.exists():
        raise HTTPException(404, "not found")
    return FileResponse(
        full,
        media_type="image/jpeg",
        headers={"Cache-Control": "public, max-age=86400"},
    )


def _unlink_rel(rel) -> None:
    if not rel:
        return
    path = (BASE / str(rel)).resolve()
    if not str(path).startswith(str(BASE.resolve())):
        return
    try:
        if path.is_dir():
            shutil.rmtree(path, ignore_errors=True)
        elif path.exists():
            path.unlink(missing_ok=True)
    except Exception:
        pass


def delete_recording(session_id: str) -> bool:
    with lock:
        item = next((r for r in recordings_index if r.get("session_id") == session_id), None)
        if not item:
            return False
        recordings_index[:] = [r for r in recordings_index if r.get("session_id") != session_id]
        persist_recordings_index()
    for key in ("wav", "m4a", "mp4", "thumb"):
        _unlink_rel(item.get(key))
    device = item.get("device") or ""
    safe = str(session_id).replace("/", "_")
    if device:
        _unlink_rel(Path(device) / "recordings" / f"{safe}_frames")
        _unlink_rel(Path(device) / "recordings" / f"{safe}.jpg")
    return True


@app.delete("/api/v1/recordings")
def api_delete_recording(session_id: str):
    if not delete_recording(session_id):
        raise HTTPException(404, "not found")
    return {"ok": True, "session_id": session_id}


@app.post("/api/v1/recordings/bulk-delete")
async def api_bulk_delete_recordings(req: Request):
    body = await req.json()
    ids = body.get("session_ids") or []
    device = (body.get("device") or "").strip() or None
    mode = (body.get("mode") or "").strip() or None
    with lock:
        items = list(recordings_index)
    if not ids and device:
        ids = [
            r.get("session_id")
            for r in items
            if r.get("device") == device and (not mode or r.get("mode") == mode)
        ]
    n = 0
    for sid in ids:
        if sid and delete_recording(str(sid)):
            n += 1
    return {"ok": True, "deleted": n}


_snap_cache: dict[str, tuple[float, bytes]] = {}


def upright_latest(device: str) -> bytes | None:
    """Rotated snapshot for still images. Cached per frame so it costs nothing
    on the live path, which now ships the raw device JPEG."""
    with lock:
        data = latest.get(device)
        ts = latest_ts.get(device, 0.0)
        nv_on = bool(device_state.get(device, {}).get("nightvision"))
    if not data:
        return None
    key = f"{device}|{1 if nv_on else 0}"
    hit = _snap_cache.get(key)
    if hit and hit[0] == ts:
        return hit[1]
    out = rotate_jpeg(data, CAMERA_ROTATE)
    if nv_on:
        out = nightvision_jpeg(out)
    _snap_cache[key] = (ts, out)
    return out


@app.get("/api/v1/latest.jpg")
def latest_jpg(device: str = "bodycam-01", annotate: int = 0):
    data = upright_latest(device)
    if data is None:
        return JSONResponse({"error": "no frame"}, 404)
    if annotate and face_engine:
        data = face_engine.annotate_jpeg(device, data)
    return Response(
        data,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )


@app.get("/api/v1/mjpeg")
async def mjpeg(request: Request, device: str = "bodycam-01", annotate: int = 0):
    global _mjpeg_live
    with _mjpeg_gate:
        if _mjpeg_live >= MJPEG_MAX:
            return JSONResponse({"error": "too many viewers"}, status_code=503)
        _mjpeg_live += 1
    period = 1.0 / LIVE_FPS

    async def gen():
        global _mjpeg_live
        last = b""
        last_sent = 0.0
        try:
            while True:
                if await request.is_disconnected():
                    return
                f = upright_latest(device)
                now = time.monotonic()
                if f and f != last and (now - last_sent) >= period:
                    last = f
                    last_sent = now
                    yield (
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(f)).encode()
                        + b"\r\n\r\n"
                        + f
                        + b"\r\n"
                    )
                    await asyncio.sleep(0)
                else:
                    await asyncio.sleep(0.05)
        finally:
            with _mjpeg_gate:
                _mjpeg_live = max(0, _mjpeg_live - 1)

    return StreamingResponse(
        gen(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
    )


@app.websocket("/ws/device")
async def ws_device(websocket: WebSocket):
    """Persistent ESP32 ingest socket. Binary packet: 0x01 JPEG, 0x02 PCM."""
    await websocket.accept()
    device = websocket.query_params.get("device") or ""
    token = websocket.query_params.get("token") or ""
    if not device or token != TOKEN:
        await websocket.close(code=1008)
        return
    touch_device(device)
    print(f"[ws-device] connected {device}", flush=True)
    threading.Thread(
        target=lambda d=device: _safe_standby(d),
        name=f"ptt-standby-{device}",
        daemon=True,
    ).start()

    async def ingest():
        while True:
            try:
                msg = await websocket.receive()
            except WebSocketDisconnect:
                return
            except Exception:
                return
            if msg.get("type") == "websocket.disconnect":
                return
            data = msg.get("bytes")
            if not data or len(data) < 2:
                continue
            kind = data[0]
            payload = bytes(data[1:])
            try:
                if kind == 0x02:
                    enqueue_pcm(device, payload, None, None)
                elif kind == 0x01:
                    now = store_live_frame(device, payload)
                    rec_path, do_face = claim_frame_work(device, now, None, "video")
                    if rec_path is not None or do_face:
                        item = (device, payload, rec_path, do_face)
                        try:
                            ingest_q.put_nowait(item)
                        except queue.Full:
                            try:
                                ingest_q.get_nowait()
                            except queue.Empty:
                                pass
                            try:
                                ingest_q.put_nowait(item)
                            except queue.Full:
                                pass
                touch_device(device)
            except Exception as e:
                print(f"[ws-device] ingest {device}: {e}", flush=True)

    async def speaker_down():
        # Radio downlink used to share /ws/device with JPEG; the cam rarely
        # processed inbound BIN while TX was busy. Speaker audio is idle here.
        while True:
            await asyncio.sleep(1.0)

    ingest_task = asyncio.create_task(ingest())
    speaker_task = asyncio.create_task(speaker_down())
    try:
        await ingest_task
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[ws-device] {device} error: {e}", flush=True)
    finally:
        speaker_task.cancel()
        try:
            await speaker_task
        except Exception:
            pass
        print(f"[ws-device] disconnected {device}", flush=True)


@app.websocket("/ws/audio")
async def ws_audio(websocket: WebSocket):
    """ESP32 mic uplink + HT radio downlink on one socket (no JPEG HOL).

    Uplink: raw s16le PCM (unchanged mic path).
    Downlink: 0x03 + s16le → bodycam speaker.
    """
    await websocket.accept()
    device = websocket.query_params.get("device") or ""
    token = websocket.query_params.get("token") or ""
    if not device or token != TOKEN:
        await websocket.close(code=4401)
        return
    print(f"[ws-audio] connected {device}", flush=True)
    sent = 0

    async def recv_mic():
        while True:
            msg = await websocket.receive()
            if msg.get("type") == "websocket.disconnect":
                return
            data = msg.get("bytes")
            if not data:
                continue
            enqueue_pcm(device, bytes(data), None, None)

    async def speaker_down():
        nonlocal sent
        while True:
            try:
                pcm = pqtalkie.pop_radio_pcm(device)
                if pcm:
                    # Opus chunks can be >>4 KB; send in pieces — never drop the head.
                    off = 0
                    while off < len(pcm):
                        piece = pcm[off : off + 4096]
                        off += len(piece)
                        await websocket.send_bytes(b"\x03" + piece)
                        sent += 1
                        if sent <= 8 or sent % 50 == 0:
                            print(
                                f"[ptt] {device} speaker out #{sent} bytes={len(piece)} "
                                f"qleft={len(pcm) - off}",
                                flush=True,
                            )
                        # Yield so other device sockets can accept/read.
                        await asyncio.sleep(0)
                else:
                    await asyncio.sleep(0.02)
            except WebSocketDisconnect:
                return
            except Exception as e:
                print(f"[ws-audio] speaker {device}: {e}", flush=True)
                await asyncio.sleep(0.25)

    recv_task = asyncio.create_task(recv_mic())
    spk_task = asyncio.create_task(speaker_down())
    try:
        done, pending = await asyncio.wait(
            {recv_task, spk_task}, return_when=asyncio.FIRST_COMPLETED
        )
        for t in pending:
            t.cancel()
        for t in done:
            exc = t.exception() if not t.cancelled() else None
            if exc:
                raise exc
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"[ws-audio] {device} error: {e}", flush=True)
    finally:
        recv_task.cancel()
        spk_task.cancel()
        print(f"[ws-audio] disconnected {device}", flush=True)


@app.websocket("/ws/liveaudio")
async def ws_live_audio(websocket: WebSocket, device: str = "bodycam-01", raw: int = 0):
    """Browser voice socket — audio only, on its own TCP connection.

    Voice used to share /ws/live with the JPEGs, so every 20 ms packet queued
    behind a 25 KB frame on the same stream. That head-of-line stall was the
    delay, and the bursty arrival that followed was the crackle. Alone on a
    socket, a packet is on the wire the moment it exists.

    raw=1 taps the mic before the voice chain. Diagnostics use this socket
    rather than /api/v1/pcm because the chunked HTTP stream silently loses
    chunks when the reader falls behind, which corrupts any measurement.
    """
    await websocket.accept()
    ring = audio_raw if raw else audio_live
    last_seq = 0
    primed = False

    async def sender():
        nonlocal last_seq, primed
        while True:
            with lock:
                buf = list(ring.get(device, ()))
            if buf:
                if not primed:
                    # Join at the live edge. Replaying the backlog would start
                    # the session already a second behind.
                    last_seq = buf[-1][0] - 1
                    primed = True
                elif buf[-1][0] - last_seq > 5:
                    # Chunks are 125 ms, so five of them is ~625 ms behind.
                    last_seq = buf[-1][0] - 1
                for seq, chunk in buf:
                    if seq > last_seq:
                        await websocket.send_bytes(chunk)
                        last_seq = seq
            await asyncio.sleep(0.012)

    try:
        await _ws_send_until_close(websocket, sender)
    except WebSocketDisconnect:
        return
    except Exception:
        return


@app.websocket("/ws/live")
async def ws_live(websocket: WebSocket, device: str = "bodycam-01"):
    """Browser video socket: newest JPEG as sent by the cam, plus face overlay.

    Audio deliberately lives on /ws/liveaudio instead of here.
    """
    await websocket.accept()
    await websocket.send_text(
        json.dumps({"t": "cfg", "rotate": CAMERA_ROTATE}, separators=(",", ":"))
    )
    last_jpeg_ts = -1.0
    last_overlay = ""

    async def sender():
        nonlocal last_jpeg_ts, last_overlay
        while True:
            t0 = time.monotonic()
            with lock:
                jpeg = latest.get(device)
                jpeg_ts = latest_ts.get(device, 0.0)

            if jpeg is not None and jpeg_ts > 0 and jpeg_ts != last_jpeg_ts:
                await websocket.send_bytes(b"\x01" + jpeg)
                last_jpeg_ts = jpeg_ts
                if face_engine:
                    ov = face_engine.live_overlay(device)
                    blob = json.dumps({"t": "faces", **ov}, separators=(",", ":"))
                    if blob != last_overlay:
                        last_overlay = blob
                        await websocket.send_text(blob)

            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.001, (1.0 / LIVE_FPS) - elapsed))

    try:
        await _ws_send_until_close(websocket, sender)
    except WebSocketDisconnect:
        return
    except Exception:
        return


@app.get("/api/v1/faces")
def list_faces(device: str | None = None, limit: int = 40):
    if not face_engine:
        return {"ok": False, "items": [], "status": {}, "devices": []}
    status = face_engine.status(device) if device else {}
    items = face_engine.list_captures(device, limit=limit) if limit else []
    return {
        "ok": True,
        "status": status,
        "items": items,
        "devices": face_engine.capture_devices(),
    }


@app.get("/api/v1/faces/registered")
def list_registered_faces(device: str | None = None):
    if not face_engine:
        return {"ok": False, "items": []}
    # Registered faces are a single global gallery, not split per device.
    return {"ok": True, "items": face_engine.list_registered(None)}


@app.post("/api/v1/faces/enroll")
async def enroll_face_upload(
    file: UploadFile = File(...),
    label: str = Form(...),
    device: str = Form("bodycam-01"),
):
    """Register a name + photo so live recognition can match this person."""
    if not face_engine:
        raise HTTPException(503, "face engine not ready")
    label = (label or "").strip()
    if not label:
        raise HTTPException(400, "nama wajib diisi")
    raw = await file.read()
    if not raw or len(raw) < 32:
        raise HTTPException(400, "file kosong")
    if len(raw) > 12 * 1024 * 1024:
        raise HTTPException(400, "file terlalu besar")
    try:
        item = face_engine.enroll_from_image(device, raw, label)
    except ValueError as e:
        raise HTTPException(400, str(e)) from e
    return {"ok": True, "item": item}


@app.post("/api/v1/faces/bulk-delete")
async def bulk_delete_faces(req: Request):
    if not face_engine:
        raise HTTPException(503, "face engine not ready")
    body = await req.json()
    ids = body.get("ids") or []
    device = (body.get("device") or "").strip() or None
    if ids:
        n = face_engine.delete_captures([str(i) for i in ids])
    elif device:
        n = face_engine.delete_unlabeled_for_device(device)
    else:
        raise HTTPException(400, "ids or device required")
    return {"ok": True, "deleted": n}


@app.delete("/api/v1/faces/{face_id}")
def delete_face(face_id: str):
    if not face_engine:
        raise HTTPException(503, "face engine not ready")
    if not face_engine.delete_capture(face_id):
        raise HTTPException(404, "face not found")
    return {"ok": True, "id": face_id}


@app.post("/api/v1/faces/{face_id}/register")
async def register_face(face_id: str, req: Request):
    if not face_engine:
        raise HTTPException(503, "face engine not ready")
    body = await req.json()
    label = (body.get("label") or "").strip()
    if not label:
        raise HTTPException(400, "label required")
    item = face_engine.set_label(face_id, label)
    if not item:
        raise HTTPException(404, "face not found")
    return {"ok": True, "item": item}


@app.delete("/api/v1/faces/registered/{label}")
def unregister_face(label: str):
    if not face_engine:
        raise HTTPException(503, "face engine not ready")
    n = face_engine.unregister_label(label)
    return {"ok": True, "cleared": n}


@app.post("/api/v1/faces/enable")
def faces_enable(device: str = "bodycam-01", enabled: int = 1):
    if not face_engine:
        raise HTTPException(503, "face engine not ready")
    face_engine.set_enabled(device, bool(enabled))
    return {"ok": True, "device": device, "enabled": bool(enabled)}


@app.patch("/api/v1/faces/{face_id}")
async def label_face(face_id: str, req: Request):
    """Set label for recognition (registers the face under that name)."""
    if not face_engine:
        raise HTTPException(503, "face engine not ready")
    body = await req.json()
    label = body.get("label")
    item = face_engine.set_label(face_id, label)
    if not item:
        raise HTTPException(404, "face not found")
    return {"ok": True, "item": item}


@app.get("/api/v1/pcm")
async def pcm_stream(device: str = "bodycam-01", raw: int = 0):
    """Live s16le mono PCM — only real samples, no fake silence (avoids stutter).

    raw=1 taps the mic before the voice chain, for diagnosing noise.
    """
    ring = audio_raw if raw else audio_live

    async def gen():
        last_seq = 0
        buf = ring.get(device)
        if buf:
            last_seq = buf[-1][0]
        while True:
            sent = False
            buf = ring.get(device)
            if buf:
                for seq, chunk in list(buf):
                    if seq > last_seq:
                        last_seq = seq
                        sent = True
                        yield chunk
            if not sent:
                await asyncio.sleep(0.015)
            else:
                await asyncio.sleep(0.001)

    return StreamingResponse(
        gen(),
        media_type="application/octet-stream",
        headers={
            "Cache-Control": "no-store",
            "X-Audio-Format": "s16le",
            "X-Audio-Rate": str(SAMPLE_RATE),
            "X-Audio-Channels": "1",
        },
    )


@app.get("/api/v1/ext")
def ext_catalog():
    """How other apps pull live bodycam data. Does not reveal tokens."""
    host = "http://45.250.101.17:7890"
    return {
        "ok": True,
        "auth": {
            "header": "X-Api-Token: <token>",
            "bearer": "Authorization: Bearer <token>",
            "query": "?token=<token>  (required for <img>, MJPEG, WebSocket)",
        },
        "tokens": {
            "master": "BODYCAM_API_TOKEN — all bodycams",
            "per_device": "BODYCAM_API_TOKENS — one token per bodycam-01/02/03",
        },
        "endpoints": {
            "devices": "GET /api/v1/ext/devices",
            "state": "GET /api/v1/ext/state?device=bodycam-01",
            "snapshot": "GET /api/v1/ext/latest.jpg?device=bodycam-01",
            "mjpeg": "GET /api/v1/ext/mjpeg?device=bodycam-01",
            "pcm": "GET /api/v1/ext/pcm?device=bodycam-01",
            "video_ws": "WS /ws/ext/live?device=bodycam-01&token=<token>",
            "audio_ws": "WS /ws/ext/audio?device=bodycam-01&token=<token>",
        },
        "gps": {
            "modules": "NEO-6M / NEO-7M / NEO-M8N NMEA 9600 on ESP UART1 (GPIO 38 RX, GPIO 39 TX)",
            "fields": "lat, lon, alt, spd (km/h), crs, sats, gps (fix), gps_age_ms",
            "where": "GET /api/v1/ext/devices  and  GET /api/v1/ext/state?device=bodycam-01",
        },
        "examples": {
            "list": f"curl -H 'X-Api-Token: TOKEN' {host}/api/v1/ext/devices",
            "snapshot": f"curl -H 'X-Api-Token: TOKEN' -o live.jpg '{host}/api/v1/ext/latest.jpg?device=bodycam-01'",
            "mjpeg": f"{host}/api/v1/ext/mjpeg?device=bodycam-01&token=TOKEN",
            "video_ws": f"ws://45.250.101.17:7890/ws/ext/live?device=bodycam-01&token=TOKEN",
            "audio_ws": f"ws://45.250.101.17:7890/ws/ext/audio?device=bodycam-01&token=TOKEN",
        },
        "video_ws_format": "binary 0x01 + JPEG bytes; text JSON {t:cfg|faces,...}",
        "audio_format": "raw PCM s16le mono 16000 Hz",
    }


@app.get("/api/v1/ext/devices")
def ext_devices(
    token: str | None = None,
    x_api_token: str | None = Header(None),
    authorization: str | None = Header(None),
):
    scope = resolve_api_scope(_incoming_api_token(token, x_api_token, authorization))
    with lock:
        known = set(online_ids()) | set(latest.keys()) | set(device_state.keys())
        if scope != "all":
            known = {scope} if scope in known else {scope}
        now = time.time()
        items = []
        for device in sorted(known):
            ts = latest_ts.get(device, 0.0)
            st = device_state.get(device) or {}
            age = round(now - ts, 2) if ts else None
            online = bool(ts and (now - ts) <= ONLINE_TTL)
            if not online and st.get("ts"):
                online = (now - float(st["ts"])) <= ONLINE_TTL
            items.append({
                "id": device,
                "online": online,
                "has_video": bool(latest.get(device) and ts and (now - ts) <= ONLINE_TTL),
                "age_s": age,
                "state": {
                    "stream": bool(st.get("stream")),
                    "audio": bool(st.get("audio")),
                    "video": bool(st.get("video")),
                    "nightvision": bool(st.get("nightvision")),
                    "ptt": bool(st.get("ptt")),
                    "sos": bool(st.get("sos")),
                    "rssi": st.get("rssi"),
                    "gps": bool(st.get("gps")),
                    "lat": st.get("lat"),
                    "lon": st.get("lon"),
                    "alt": st.get("alt"),
                    "spd": st.get("spd"),
                    "crs": st.get("crs"),
                    "sats": st.get("sats"),
                    "gps_age_ms": st.get("gps_age_ms"),
                },
            })
    return {"ok": True, "devices": items}


@app.get("/api/v1/ext/state")
def ext_state(
    device: str | None = None,
    token: str | None = None,
    x_api_token: str | None = Header(None),
    authorization: str | None = Header(None),
):
    scope = resolve_api_scope(_incoming_api_token(token, x_api_token, authorization), device)
    unit = ext_unit(scope, device)
    return get_device_state(unit)


@app.get("/api/v1/ext/latest.jpg")
def ext_latest_jpg(
    device: str | None = None,
    annotate: int = 0,
    token: str | None = None,
    x_api_token: str | None = Header(None),
    authorization: str | None = Header(None),
):
    scope = resolve_api_scope(_incoming_api_token(token, x_api_token, authorization), device)
    unit = ext_unit(scope, device)
    return latest_jpg(unit, annotate)


@app.get("/api/v1/ext/mjpeg")
async def ext_mjpeg(
    device: str | None = None,
    annotate: int = 0,
    token: str | None = None,
    x_api_token: str | None = Header(None),
    authorization: str | None = Header(None),
):
    scope = resolve_api_scope(_incoming_api_token(token, x_api_token, authorization), device)
    unit = ext_unit(scope, device)
    return await mjpeg(unit, annotate)


@app.get("/api/v1/ext/pcm")
async def ext_pcm(
    device: str | None = None,
    raw: int = 0,
    token: str | None = None,
    x_api_token: str | None = Header(None),
    authorization: str | None = Header(None),
):
    scope = resolve_api_scope(_incoming_api_token(token, x_api_token, authorization), device)
    unit = ext_unit(scope, device)
    return await pcm_stream(unit, raw)


@app.websocket("/ws/ext/live")
async def ws_ext_live(websocket: WebSocket, device: str = "bodycam-01"):
    await websocket.accept()
    token = websocket.query_params.get("token") or websocket.headers.get("x-api-token") or _bearer(
        websocket.headers.get("authorization")
    )
    try:
        scope = resolve_api_scope(token, device)
        unit = ext_unit(scope, device)
    except HTTPException:
        await websocket.close(code=4401)
        return
    await websocket.send_text(
        json.dumps({"t": "cfg", "rotate": CAMERA_ROTATE, "device": unit}, separators=(",", ":"))
    )
    last_jpeg_ts = -1.0
    last_overlay = ""

    async def sender():
        nonlocal last_jpeg_ts, last_overlay
        while True:
            t0 = time.monotonic()
            with lock:
                jpeg = latest.get(unit)
                jpeg_ts = latest_ts.get(unit, 0.0)
            if jpeg is not None and jpeg_ts > 0 and jpeg_ts != last_jpeg_ts:
                await websocket.send_bytes(b"\x01" + jpeg)
                last_jpeg_ts = jpeg_ts
                if face_engine:
                    ov = face_engine.live_overlay(unit)
                    blob = json.dumps({"t": "faces", **ov}, separators=(",", ":"))
                    if blob != last_overlay:
                        last_overlay = blob
                        await websocket.send_text(blob)
            elapsed = time.monotonic() - t0
            await asyncio.sleep(max(0.001, (1.0 / LIVE_FPS) - elapsed))

    try:
        await _ws_send_until_close(websocket, sender)
    except WebSocketDisconnect:
        return
    except Exception:
        return


@app.websocket("/ws/ext/audio")
async def ws_ext_audio(websocket: WebSocket, device: str = "bodycam-01", raw: int = 0):
    await websocket.accept()
    token = websocket.query_params.get("token") or websocket.headers.get("x-api-token") or _bearer(
        websocket.headers.get("authorization")
    )
    try:
        scope = resolve_api_scope(token, device)
        unit = ext_unit(scope, device)
    except HTTPException:
        await websocket.close(code=4401)
        return
    ring = audio_raw if raw else audio_live
    last_seq = 0
    primed = False

    async def sender():
        nonlocal last_seq, primed
        while True:
            with lock:
                buf = list(ring.get(unit, ()))
            if buf:
                if not primed:
                    last_seq = buf[-1][0] - 1
                    primed = True
                elif buf[-1][0] - last_seq > 5:
                    last_seq = buf[-1][0] - 1
                for seq, chunk in buf:
                    if seq > last_seq:
                        await websocket.send_bytes(chunk)
                        last_seq = seq
            await asyncio.sleep(0.012)

    try:
        await _ws_send_until_close(websocket, sender)
    except WebSocketDisconnect:
        return
    except Exception:
        return


@app.get("/")
def ui():
    # The audio worklet is inlined in this page, so a cached copy would keep
    # playing the old jitter buffer after a server update.
    return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})


if STATIC.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")
