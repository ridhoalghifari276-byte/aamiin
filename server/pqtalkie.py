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


def wav_to_pcm(raw: bytes) -> bytes:
    if len(raw) >= 44 and raw[:4] == b"RIFF":
        i = raw.find(b"data")
        if i >= 0 and i + 8 <= len(raw):
            return raw[i + 8 :]
    return raw


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
        token = normalize_kiosk_token(token)
        with self._lock:
            same = token == self.kiosk_token
            alive = self._connect_thread is not None and self._connect_thread.is_alive()
            self.kiosk_token = token
            if same and (not token or alive):
                return
        if token:
            self._ensure_connect_thread()
        else:
            self._disconnect()

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
            pcm = wav_to_pcm(raw)
            if not pcm:
                return
            with self._lock:
                self.rx_chunks += 1
                self.rx_bytes += len(pcm)
                self.rx_at = time.time()
                n = self.rx_chunks
            if n <= 3 or n % 50 == 0:
                print(
                    f"[ptt] {self.device} RX radio audio #{n} bytes={len(pcm)}",
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
