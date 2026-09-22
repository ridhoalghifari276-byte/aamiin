"""Bridge bodycam PTT (GPIO hold-to-talk) into PQTALKIE radio.

Native IoT client — same protocol as the web App, not the HTTPS SPA on :3443.

  API_BASE = http://45.250.101.16:4000   (REST + Socket.IO, cleartext)
  POST /auth/kiosk  {token: <kiosk slug>}  → JWT, user, channelId
  Socket.IO ws://host:4000  auth {token: jwt}
  emit ptt:join / ptt:request / ptt:audio (WAV 16 kHz) / ptt:release

Do not point this bridge at https://…:3443/r/… — that is the browser SPA.
"""
from __future__ import annotations

import base64
import json
import os
import struct
import threading
import time
from collections import deque
from urllib.parse import urlparse

import httpx

PQTALKIE_URL = (os.getenv("PQTALKIE_URL") or "http://192.168.245.99:4000").rstrip("/")
_scheme = (urlparse(PQTALKIE_URL).scheme or "http").lower()
# Cleartext :4000 needs no TLS. HTTPS (e.g. nginx :3443) may use self-signed.
PQTALKIE_INSECURE = os.getenv(
    "PQTALKIE_INSECURE",
    "1" if _scheme == "https" else "0",
) not in ("0", "false", "False")
SAMPLE_RATE = 16000


def normalize_kiosk_token(raw: str) -> str:
    """Accept a raw kiosk slug or a paste of the web tautan …/r/<slug>."""
    raw = (raw or "").strip().strip("\"'")
    if not raw:
        return ""
    s = raw.replace("https://", "").replace("http://", "")
    parts = [p for p in s.split("/") if p]
    return parts[-1].split("?")[0].strip() if parts else ""


DEFAULT_TOKEN = normalize_kiosk_token(
    os.getenv("PQTALKIE_TOKEN") or os.getenv("PQTALKIE_KIOSK") or ""
)


def _env_tokens() -> dict[str, str]:
    raw = (os.getenv("PQTALKIE_TOKENS") or "").strip()
    if not raw:
        return {}
    try:
        if raw.startswith("{"):
            return {
                str(k).strip(): normalize_kiosk_token(str(v))
                for k, v in json.loads(raw).items()
                if normalize_kiosk_token(str(v))
            }
    except Exception:
        pass
    out: dict[str, str] = {}
    for part in raw.split(","):
        if ":" not in part:
            continue
        k, v = part.split(":", 1)
        k, v = k.strip(), v.strip()
        if k and v:
            out[k] = v
    return out


def _pcm_peak(pcm: bytes) -> int:
    if len(pcm) < 2:
        return 0
    samples = memoryview(pcm).cast("h")
    peak = 0
    for s in samples:
        a = -s if s < 0 else s
        if a > peak:
            peak = a
    return int(peak)


def _pcm_metrics(pcm: bytes) -> dict:
    if len(pcm) < 4:
        return {"peak": 0, "clip": 1.0, "zcr": 0.0, "rms": 0.0}
    samples = memoryview(pcm).cast("h")
    n = len(samples)
    peak = 0
    clip_n = 0
    zc = 0
    acc = 0.0
    prev = 0
    for i, s in enumerate(samples):
        a = -s if s < 0 else s
        if a > peak:
            peak = a
        if a > 28000:
            clip_n += 1
        acc += float(s) * float(s)
        if i and ((prev >= 0) != (s >= 0)):
            zc += 1
        prev = s
    return {
        "peak": int(peak),
        "clip": clip_n / float(n),
        "zcr": zc / float(n),
        "rms": (acc / float(n)) ** 0.5,
    }


def _score_pcm(m: dict) -> float:
    """Lower is better. Full-scale clipped noise scores worst."""
    score = m["clip"] * 20.0
    if m["peak"] >= 30000:
        score += 5.0
    # Speech-ish zero-crossing; pure square/noise is extreme.
    z = m["zcr"]
    if z < 0.01 or z > 0.45:
        score += 3.0
    # Prefer some energy but not pegged.
    if m["rms"] < 200:
        score += 2.0
    if m["rms"] > 20000:
        score += 2.0
    return score


def _resample_to_16k(pcm: bytes, rate: int) -> bytes:
    if rate <= 0 or rate == SAMPLE_RATE or len(pcm) < 4:
        return pcm
    src = memoryview(pcm).cast("h")
    n_src = len(src)
    n_dst = max(1, int(round(n_src * SAMPLE_RATE / float(rate))))
    out = bytearray(n_dst * 2)
    dst = memoryview(out).cast("h")
    if n_src == 1:
        for i in range(n_dst):
            dst[i] = src[0]
        return bytes(out)
    for i in range(n_dst):
        pos = i * (n_src - 1) / float(n_dst - 1)
        i0 = int(pos)
        i1 = i0 + 1 if i0 + 1 < n_src else i0
        frac = pos - i0
        dst[i] = int(src[i0] * (1.0 - frac) + src[i1] * frac)
    return bytes(out)


def _downmix_stereo(pcm: bytes) -> bytes:
    if len(pcm) < 4:
        return pcm
    if len(pcm) % 4:
        pcm = pcm[: len(pcm) - (len(pcm) % 4)]
    samples = memoryview(pcm).cast("h")
    n_frames = len(samples) // 2
    out = bytearray(n_frames * 2)
    dst = memoryview(out).cast("h")
    for i in range(n_frames):
        dst[i] = (int(samples[i * 2]) + int(samples[i * 2 + 1])) // 2
    return bytes(out)


def _u8_to_s16(raw: bytes) -> bytes:
    out = bytearray(len(raw) * 2)
    dst = memoryview(out).cast("h")
    for i, b in enumerate(raw):
        dst[i] = (int(b) - 128) << 8
    return bytes(out)


def _byteswap_s16(raw: bytes) -> bytes:
    if len(raw) < 2:
        return raw
    if len(raw) % 2:
        raw = raw[:-1]
    out = bytearray(len(raw))
    for i in range(0, len(raw), 2):
        out[i] = raw[i + 1]
        out[i + 1] = raw[i]
    return bytes(out)


def _attenuate(pcm: bytes, div: int) -> bytes:
    if div <= 1 or len(pcm) < 2:
        return pcm
    samples = memoryview(pcm).cast("h")
    out = bytearray(len(pcm))
    dst = memoryview(out).cast("h")
    for i, s in enumerate(samples):
        dst[i] = int(s) // div
    return bytes(out)


def _decode_candidates(raw: bytes) -> list[tuple[str, bytes, int]]:
    """Return list of (kind, pcm_mono_16k, assumed_src_rate)."""
    cands: list[tuple[str, bytes, int]] = []
    even = raw if (len(raw) % 2 == 0) else raw[:-1]
    if len(even) >= 4:
        cands.append(("s16le_16k", even, SAMPLE_RATE))
        cands.append(("s16be_16k", _byteswap_s16(even), SAMPLE_RATE))
        cands.append(("s16le_8k", _resample_to_16k(even, 8000), 8000))
        cands.append(("s16be_8k", _resample_to_16k(_byteswap_s16(even), 8000), 8000))
        stereo = _downmix_stereo(even)
        if len(stereo) >= 4:
            cands.append(("stereo_16k", stereo, SAMPLE_RATE))
            cands.append(("stereo_8k", _resample_to_16k(stereo, 8000), 8000))
    if len(raw) >= 8:
        u8 = _u8_to_s16(raw)
        cands.append(("u8_16k", u8, SAMPLE_RATE))
        cands.append(("u8_8k", _resample_to_16k(u8, 8000), 8000))
    return cands



_opus_decoders: dict[tuple[int, int], object] = {}
_opus_dec_lock = threading.Lock()


def _opus_decoder(rate: int, channels: int):
    """Cached libopus decoder (output rate/channels)."""
    key = (int(rate), int(channels))
    with _opus_dec_lock:
        dec = _opus_decoders.get(key)
        if dec is None:
            import opuslib
            dec = opuslib.Decoder(key[0], key[1])
            _opus_decoders[key] = dec
        return dec


def _parse_pqop(raw: bytes) -> dict | None:
    """PQTALKIE Opus frame: PQOP | ver | ch | rate_u32le | u16 | u16 | opus..."""
    if len(raw) < 14 or raw[:4] != b"PQOP":
        return None
    ch = int(raw[5]) or 1
    rate = struct.unpack_from("<I", raw, 6)[0] or 48000
    misc = struct.unpack_from("<H", raw, 10)[0]
    length = struct.unpack_from("<H", raw, 12)[0]
    return {
        "ver": int(raw[4]),
        "ch": ch,
        "rate": rate,
        "misc": misc,
        "length": length,
        "payload": raw[14:],
    }


def opus_to_pcm(raw: bytes) -> tuple[bytes, dict]:
    """Decode audio/opus (optional PQOP header) to mono s16le @ SAMPLE_RATE."""
    info: dict = {"kind": "opus", "bits": 16, "fmt": "opus"}
    if not raw:
        return b"", info
    rate = 48000
    ch = 1
    candidates: list[bytes] = []
    if raw.startswith(b"PQOP"):
        hdr = _parse_pqop(raw)
        if not hdr:
            info["err"] = "bad PQOP"
            return b"", info
        rate = int(hdr["rate"]) or 48000
        ch = int(hdr["ch"]) or 1
        payload = hdr["payload"]
        info.update(
            {
                "kind": "pqop/opus",
                "rate": rate,
                "ch": ch,
                "pqop_len": hdr["length"],
                "hex": raw[:16].hex(),
            }
        )
        plen = int(hdr["length"] or 0)
        if 10 <= plen <= len(payload):
            candidates.append(payload[:plen])
        candidates.append(payload)
    else:
        info["hex"] = raw[:16].hex()
        info["rate"] = rate
        info["ch"] = ch
        candidates.append(raw)

    last_err: object = None
    pcm = b""
    for cand in candidates:
        if not cand:
            continue
        try:
            dec = _opus_decoder(SAMPLE_RATE, 1)
            frame_size = SAMPLE_RATE * 120 // 1000
            pcm = dec.decode(bytes(cand), frame_size)
            last_err = None
            break
        except Exception as e1:
            last_err = e1
            try:
                dec = _opus_decoder(rate, 1 if ch < 2 else 2)
                frame_size = rate * 120 // 1000
                pcm = dec.decode(bytes(cand), frame_size)
                if ch >= 2:
                    pcm = _downmix_stereo(pcm)
                pcm = _resample_to_16k(pcm, rate)
                info["note"] = f"fallback48:{e1}"
                last_err = None
                break
            except Exception as e2:
                last_err = f"{e1} / {e2}"
                pcm = b""
    if not pcm:
        info["err"] = f"opus decode failed: {last_err}"
        return b"", info
    m = _pcm_metrics(pcm)
    info.update(
        {
            "rate_out": SAMPLE_RATE,
            "peak": m["peak"],
            "clip": round(m["clip"], 3),
            "zcr": round(m["zcr"], 3),
            "out": len(pcm),
        }
    )
    return pcm, info


def wav_to_pcm(raw: bytes, mime: str = "") -> tuple[bytes, dict]:
    """Extract s16le mono @ SAMPLE_RATE from WAV/PCM/Opus. Returns (pcm, info)."""
    info: dict = {"kind": "raw", "rate": SAMPLE_RATE, "ch": 1, "bits": 16, "fmt": 1}
    if not raw:
        return b"", info
    mime_l = (mime or "").lower()
    if "opus" in mime_l or raw.startswith(b"PQOP"):
        return opus_to_pcm(raw)

    if len(raw) >= 12 and raw[:4] == b"RIFF" and raw[8:12] == b"WAVE":
        info["kind"] = "wav"
        pos = 12
        rate = SAMPLE_RATE
        ch = 1
        bits = 16
        pcm = raw
        try:
            while pos + 8 <= len(raw):
                tag = raw[pos : pos + 4]
                sz = struct.unpack_from("<I", raw, pos + 4)[0]
                body = pos + 8
                end = min(len(raw), body + max(0, sz))
                if tag == b"fmt " and sz >= 16 and body + 16 <= len(raw):
                    fmt = struct.unpack_from("<H", raw, body)[0]
                    ch = struct.unpack_from("<H", raw, body + 2)[0] or 1
                    rate = struct.unpack_from("<I", raw, body + 4)[0] or SAMPLE_RATE
                    bits = struct.unpack_from("<H", raw, body + 14)[0] or 16
                    info.update({"fmt": fmt, "ch": ch, "rate": rate, "bits": bits})
                    if fmt not in (1, 65534):
                        return b"", info
                elif tag == b"data":
                    pcm = raw[body:end]
                    break
                pos = body + sz + (sz & 1)
        except Exception:
            i = raw.find(b"data")
            if i >= 0 and i + 8 <= len(raw):
                pcm = raw[i + 8 :]
        if bits != 16:
            return b"", info
        if ch >= 2 and len(pcm) >= 4:
            pcm = _downmix_stereo(pcm)
            info["ch"] = 1
        pcm = _resample_to_16k(pcm, rate)
        m = _pcm_metrics(pcm)
        info.update({"rate_out": SAMPLE_RATE, "peak": m["peak"], "clip": round(m["clip"], 3), "out": len(pcm)})
        return pcm, info

    # Headerless: pick the decode that looks least like full-scale noise.
    best_pcm = b""
    best_score = 1e9
    best_kind = "s16le_16k"
    best_rate = SAMPLE_RATE
    best_m: dict = {}
    for kind, pcm, rate in _decode_candidates(raw):
        m = _pcm_metrics(pcm)
        score = _score_pcm(m)
        if score < best_score:
            best_score = score
            best_pcm = pcm
            best_kind = kind
            best_rate = rate
            best_m = m
    # If still pegged, gently bring into speech range (last resort — source is hot/wrong).
    if best_m.get("clip", 1) > 0.35 or best_m.get("peak", 0) >= 30000:
        for div in (2, 4, 8):
            softened = _attenuate(best_pcm, div)
            m = _pcm_metrics(softened)
            score = _score_pcm(m) + div * 0.1
            if score < best_score:
                best_score = score
                best_pcm = softened
                best_kind = f"{best_kind}/att{div}"
                best_m = m
    info.update(
        {
            "kind": best_kind,
            "rate": best_rate,
            "ch": 1,
            "bits": 16,
            "rate_out": SAMPLE_RATE,
            "peak": best_m.get("peak", 0),
            "clip": round(best_m.get("clip", 0.0), 3),
            "zcr": round(best_m.get("zcr", 0.0), 3),
            "out": len(best_pcm),
            "hex": raw[:16].hex(),
        }
    )
    return best_pcm, info


def pcm_wav(pcm: bytes, rate: int = SAMPLE_RATE) -> bytes:
    n = len(pcm)
    hdr = b"RIFF" + struct.pack("<I", 36 + n) + b"WAVEfmt "
    hdr += struct.pack("<IHHIIHH", 16, 1, 1, rate, rate * 2, 2, 16)
    hdr += b"data" + struct.pack("<I", n)
    return hdr + pcm


class TalkieBridge:
    def __init__(self, device: str):
        self.device = device
        self.kiosk_token = ""
        self.jwt = ""
        self.channel_id: int | None = None
        self.label = ""
        self.user_name = ""
        self.tx = False
        self.sos = False
        self.sos_want = False
        self.alert_id = None
        self.lat = None
        self.lon = None
        self._loc_sent = 0.0
        self.ok = False
        self.last_err = ""
        self.rx_chunks = 0
        self.rx_bytes = 0
        self.rx_at = 0.0
        self.tx_chunks = 0
        self.sos_err = ""
        self._sio = None
        self._lock = threading.Lock()
        self._pending = bytearray()
        self._connect_thread: threading.Thread | None = None

    def set_token(self, token: str):
        """Apply kiosk slug. Changing token forces leave + re-auth (new channel)."""
        token = normalize_kiosk_token(token)
        with self._lock:
            same = token == self.kiosk_token
            alive = self._connect_thread is not None and self._connect_thread.is_alive()
            self.kiosk_token = token
            if same and (not token or alive):
                return
        if not token:
            self._disconnect()
            return
        # Drop old JWT/socket so _connect_loop re-auths (ch4 → ch11, etc.).
        if alive and not same:
            self._disconnect()
        self._ensure_connect_thread()

    def _ensure_connect_thread(self):
        with self._lock:
            t = self._connect_thread
            if t is not None and t.is_alive():
                return
            t = threading.Thread(target=self._connect_loop, name=f"ptt-{self.device}", daemon=True)
            self._connect_thread = t
        t.start()

    def set_tx(self, on: bool):
        on = bool(on)
        with self._lock:
            if on == self.tx:
                return
            self.tx = on
            self._pending.clear()
        if on:
            self._emit_request()
        else:
            self._emit_release()

    def set_sos(self, on: bool):
        on = bool(on)
        with self._lock:
            self.sos_want = on
            if on == self.sos and (bool(self.alert_id) == on):
                return
        threading.Thread(target=self._sos_apply, name=f"sos-{self.device}", daemon=True).start()

    def set_location(self, lat, lon):
        try:
            lat_f = float(lat)
            lon_f = float(lon)
        except (TypeError, ValueError):
            return
        if lat_f < -90 or lat_f > 90 or lon_f < -180 or lon_f > 180:
            return
        with self._lock:
            self.lat = lat_f
            self.lon = lon_f
            alert_id = self.alert_id
            sos = self.sos
            last = self._loc_sent
        now = time.monotonic()
        if sos and alert_id and (now - last) >= 4.0:
            threading.Thread(
                target=self._sos_location, args=(alert_id, lat_f, lon_f), daemon=True
            ).start()

    def _wait_ready(self, seconds: float = 8.0) -> bool:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            with self._lock:
                if self.ok and self.jwt:
                    return True
            time.sleep(0.2)
        with self._lock:
            return bool(self.ok and self.jwt)

    def feed_pcm(self, pcm: bytes):
        if not pcm:
            return
        with self._lock:
            if not self.tx or not self.ok:
                return
            self._pending.extend(pcm)
            # PQTALKIE radio uses ~200 ms @ 16 kHz.
            need = SAMPLE_RATE * 2 // 5
            chunk = bytes(self._pending[:need]) if len(self._pending) >= need else b""
            if chunk:
                del self._pending[:need]
        if chunk:
            self._emit_audio(chunk)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "device": self.device,
                "ready": self.ok,
                "tx": self.tx,
                "sos": self.sos,
                "alertId": self.alert_id,
                "channelId": self.channel_id,
                "label": self.label,
                "user": self.user_name,
                "has_token": bool(self.kiosk_token),
                "api": PQTALKIE_URL,
                "rx_chunks": self.rx_chunks,
                "rx_bytes": self.rx_bytes,
                "rx_age_ms": int((time.time() - self.rx_at) * 1000) if self.rx_at else None,
                "tx_chunks": self.tx_chunks,
                "sos_err": self.sos_err,
                "error": self.last_err,
            }

    def _post(self, path: str, body: dict, token: str | None = None) -> dict:
        headers = {"Content-Type": "application/json", "Accept": "application/json"}
        if token:
            headers["Authorization"] = f"Bearer {token}"
        url = PQTALKIE_URL + path
        # Native API on :4000 is usually fast; keep headroom for cold start.
        with httpx.Client(verify=not PQTALKIE_INSECURE, timeout=12.0) as c:
            r = c.post(url, headers=headers, json=body)
            r.raise_for_status()
            if not r.content:
                return {}
            return r.json()

    def _connect_loop(self):
        delay = 2.0
        while True:
            with self._lock:
                if not self.kiosk_token:
                    return
            try:
                self._connect_once()
            except Exception as e:
                with self._lock:
                    self.ok = False
                    self.last_err = f"{PQTALKIE_URL}: {e}"
                print(f"[ptt] {self.device} kiosk failed via {PQTALKIE_URL}: {e}", flush=True)
            while True:
                with self._lock:
                    if not self.kiosk_token:
                        return
                    if self.ok:
                        delay = 2.0
                    else:
                        break
                time.sleep(1.0)
            time.sleep(delay)
            delay = min(delay * 2.0, 30.0)

    def _connect_once(self):
        data = None
        last = None
        # Prefer raw API path (:4000). /api/auth/kiosk is the nginx SPA proxy on :3443.
        for path in ("/auth/kiosk", "/api/auth/kiosk"):
            try:
                data = self._post(path, {"token": self.kiosk_token})
                break
            except Exception as e:
                last = e
        if data is None:
            raise RuntimeError(f"kiosk auth failed at {PQTALKIE_URL}: {last}")
        jwt = (data.get("token") or "").strip()
        cid = data.get("channelId")
        if not jwt or cid is None:
            raise RuntimeError(f"kiosk response missing token/channelId from {PQTALKIE_URL}")
        user = data.get("user") or {}
        with self._lock:
            self.jwt = jwt
            self.channel_id = int(cid)
            self.label = str(data.get("label") or "")
            self.user_name = str(user.get("name") or "")
            self.last_err = ""
        self._socket_connect()

    def _socket_connect(self):
        try:
            import socketio
        except ImportError:
            with self._lock:
                self.ok = False
                self.last_err = "python-socketio not installed"
            print("[ptt] pip install python-socketio websocket-client", flush=True)
            return
        self._disconnect()
        sio = socketio.Client(
            ssl_verify=False if _scheme == "http" else (not PQTALKIE_INSECURE),
            reconnection=False,
        )
        jwt = self.jwt
        cid = self.channel_id

        @sio.event
        def connect():
            sio.emit("ptt:join", {"channelId": cid})
            with self._lock:
                self.ok = True
                self.last_err = ""
                want_sos = self.sos_want
                want_tx = self.tx
            print(f"[ptt] {self.device} joined channel {cid}", flush=True)
            if want_tx:
                self._emit_request()
            if want_sos:
                threading.Thread(target=self._sos_apply, name=f"sos-{self.device}", daemon=True).start()

        @sio.on("ptt:audio")
        def _rx_audio(data):
            with self._lock:
                if self.tx or not self.ok:
                    return
            # Web/HT emit: { channelId, chunk: base64 WAV|PCM, mime }
            if isinstance(data, (list, tuple)) and data:
                data = data[0]
            if not isinstance(data, dict):
                return
            chunk = data.get("chunk") or data.get("audio") or data.get("data") or ""
            if isinstance(chunk, (bytes, bytearray)):
                raw = bytes(chunk)
            elif isinstance(chunk, str) and chunk:
                try:
                    raw = base64.b64decode(chunk)
                except Exception:
                    return
            else:
                return
            mime = str(data.get("mime") or data.get("type") or "")
            pcm, meta = wav_to_pcm(raw, mime)
            if not pcm:
                return
            with self._lock:
                self.rx_chunks += 1
                self.rx_bytes += len(pcm)
                self.rx_at = time.time()
                n = self.rx_chunks
            if n <= 8 or n % 50 == 0:
                print(
                    f"[ptt] {self.device} RX radio audio #{n} "
                    f"in={len(raw)} out={len(pcm)} peak={meta.get('peak')} "
                    f"clip={meta.get('clip')} kind={meta.get('kind')} "
                    f"rate={meta.get('rate')} hex={meta.get('hex', '')} mime={mime!r}",
                    flush=True,
                )
            push_radio_pcm(self.device, pcm)

        @sio.event
        def disconnect():
            with self._lock:
                self.ok = False
            print(f"[ptt] {self.device} socket disconnected", flush=True)

        self._sio = sio
        sio.connect(
            PQTALKIE_URL,
            auth={"token": jwt},
            transports=["websocket"],
            socketio_path="socket.io",
            wait_timeout=15,
        )

    def _disconnect(self):
        sio = self._sio
        self._sio = None
        if not sio:
            return
        try:
            if self.channel_id is not None:
                sio.emit("ptt:leave", {"channelId": self.channel_id})
        except Exception:
            pass
        try:
            sio.disconnect()
        except Exception:
            pass
        with self._lock:
            self.ok = False

    def _emit_request(self):
        sio = self._sio
        cid = self.channel_id
        if not sio or cid is None:
            return
        try:
            sio.emit("ptt:request", {"channelId": cid})
            with self._lock:
                self.last_err = ""
            print(f"[ptt] {self.device} TX start", flush=True)
        except Exception as e:
            with self._lock:
                self.last_err = str(e)

    def _emit_release(self):
        sio = self._sio
        cid = self.channel_id
        if not sio or cid is None:
            return
        try:
            sio.emit("ptt:release", {"channelId": cid})
            print(f"[ptt] {self.device} TX stop", flush=True)
        except Exception as e:
            with self._lock:
                self.last_err = str(e)

    def _emit_audio(self, pcm: bytes):
        sio = self._sio
        cid = self.channel_id
        if not sio or cid is None:
            return
        wav = pcm_wav(pcm)
        chunk = base64.b64encode(wav).decode("ascii")
        try:
            sio.emit("ptt:audio", {"channelId": cid, "chunk": chunk, "mime": "audio/wav"})
            with self._lock:
                self.tx_chunks += 1
                n = self.tx_chunks
            if n <= 3 or n % 50 == 0:
                print(f"[ptt] {self.device} TX radio audio #{n} bytes={len(pcm)}", flush=True)
        except Exception as e:
            with self._lock:
                self.last_err = str(e)

    def _sos_apply(self):
        if not self._wait_ready():
            with self._lock:
                self.last_err = "radio not connected"
            print(f"[sos] {self.device} radio not connected", flush=True)
            return
        with self._lock:
            want = self.sos_want
            jwt = self.jwt
            cid = self.channel_id
            alert_id = self.alert_id
            lat, lon = self.lat, self.lon
        try:
            if want:
                if alert_id:
                    with self._lock:
                        self.sos = True
                    if lat is not None and lon is not None:
                        self._sos_location(alert_id, lat, lon)
                    return
                data = None
                last = None
                body = {"envelope": {}, "channelId": int(cid) if cid is not None else None}
                for path in ("/api/emergency/alerts", "/emergency/alerts"):
                    try:
                        data = self._post(path, body, jwt)
                        break
                    except Exception as e:
                        last = e
                if not data or not data.get("id"):
                    raise last or RuntimeError("SOS create failed")
                aid = data["id"]
                with self._lock:
                    self.sos = True
                    self.alert_id = aid
                    self.last_err = ""
                print(f"[sos] {self.device} ON alert={aid}", flush=True)
                if lat is not None and lon is not None:
                    self._sos_location(aid, lat, lon)
            else:
                if not alert_id:
                    with self._lock:
                        self.sos = False
                    return
                last = None
                for path in (
                    f"/api/emergency/alerts/{alert_id}/resolve",
                    f"/emergency/alerts/{alert_id}/resolve",
                ):
                    try:
                        self._post(path, {}, jwt)
                        last = None
                        break
                    except Exception as e:
                        last = e
                if last:
                    raise last
                with self._lock:
                    self.sos = False
                    self.alert_id = None
                    self.last_err = ""
                print(f"[sos] {self.device} OFF", flush=True)
        except Exception as e:
            # SOS API failure must not look like the radio link died.
            with self._lock:
                self.sos_err = str(e)
            print(f"[sos] {self.device} failed: {e}", flush=True)

    def _sos_location(self, alert_id, lat: float, lon: float):
        jwt = self.jwt
        env = {"lat": lat, "lng": lon, "accuracy": 15, "ts": int(time.time() * 1000)}
        last = None
        for path in (
            f"/api/emergency/alerts/{alert_id}/location",
            f"/emergency/alerts/{alert_id}/location",
        ):
            try:
                self._post(path, {"envelope": env}, jwt)
                with self._lock:
                    self._loc_sent = time.monotonic()
                return
            except Exception as e:
                last = e
        if last:
            print(f"[sos] {self.device} location failed: {last}", flush=True)


_bridges: dict[str, TalkieBridge] = {}
_mux = threading.Lock()
_env = _env_tokens()


def bridge_for(device: str) -> TalkieBridge:
    with _mux:
        b = _bridges.get(device)
        if b is None:
            b = TalkieBridge(device)
            _bridges[device] = b
            tok = _env.get(device) or DEFAULT_TOKEN
            if tok:
                b.set_token(tok)
        return b


def warm_default():
    """Radio joins in the background as soon as a bodycam comes online."""
    if not DEFAULT_TOKEN and not _env:
        print("[ptt] no kiosk token — PTT idle", flush=True)
        return
    print("[ptt] radio standby when a bodycam is online", flush=True)


def _existing(device: str) -> TalkieBridge | None:
    with _mux:
        return _bridges.get(device)


def ensure_standby(device: str):
    """Keep this unit's radio joined while the bodycam is alive. Non-blocking."""
    if not device:
        return
    tok = _env.get(device) or DEFAULT_TOKEN
    if not tok:
        return
    bridge_for(device)


def set_device_token(device: str, token: str):
    bridge_for(device).set_token(token)


def set_device_tx(device: str, on: bool):
    ensure_standby(device)
    b = _existing(device)
    if b is None:
        return
    b.set_tx(on)


def set_device_sos(device: str, on: bool):
    if on:
        ensure_standby(device)
    b = _existing(device)
    if b is None:
        return
    b.set_sos(on)


def set_device_location(device: str, lat, lon):
    b = _existing(device)
    if b:
        b.set_location(lat, lon)


def feed_device_pcm(device: str, pcm: bytes):
    b = _bridges.get(device)
    if b:
        b.feed_pcm(pcm)


_rx: dict[str, deque] = {}
_rx_lock = threading.Lock()


def push_radio_pcm(device: str, pcm: bytes):
    if not pcm:
        return
    with _rx_lock:
        q = _rx.get(device)
        if q is None:
            q = deque(maxlen=48)
            _rx[device] = q
        q.append(pcm)


def pop_radio_pcm(device: str) -> bytes:
    with _rx_lock:
        q = _rx.get(device)
        if not q:
            return b""
        return q.popleft()


def status(device: str | None = None) -> dict:
    with _mux:
        if device:
            b = _bridges.get(device)
            if not b:
                return {
                    "device": device,
                    "ready": False,
                    "tx": False,
                    "sos": False,
                    "has_token": bool(_env.get(device) or DEFAULT_TOKEN),
                    "api": PQTALKIE_URL,
                    "error": "",
                }
            return b.snapshot()
        return {d: b.snapshot() for d, b in _bridges.items()}
