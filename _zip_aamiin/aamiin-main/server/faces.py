"""Face detect (YuNet) + optional SFace recognition, HOG fallback."""
from __future__ import annotations

import json
import os
import queue
import threading
import time
import uuid
from collections import defaultdict
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

CAPTURE_COOLDOWN_SEC = 2.0
MIN_FACE_PX = 40
CROP_PAD = 0.35
MAX_INDEX = 500
MAX_UNLABELED_PER_DEVICE = 80
SCORE_LIVE = 0.55
SCORE_CAPTURE = 0.85
DETECT_W = 640
DETECT_H = 480
INSIGHT_COSINE_TH = 0.28
SFACE_COSINE_TH = 0.363
AI_CAPTURE_SCORE = 0.50

YUNET_URL = (
    "https://github.com/opencv/opencv_zoo/raw/main/models/"
    "face_detection_yunet/face_detection_yunet_2023mar.onnx"
)
MODEL_NAME = "face_detection_yunet_2023mar.onnx"
SFACE_NAME = "face_recognition_sface_2021dec.onnx"


@dataclass
class FaceCapture:
    id: str
    device: str
    ts: float
    bbox: list
    score: float
    face: str
    full: str
    label: str | None = None
    embedding: list | None = None
    registered: bool = False


def _resolve_model(name: str, env_key: str) -> Path | None:
    candidates = [
        Path(os.getenv(env_key, "")),
        Path(__file__).resolve().parent / "models" / name,
        Path("/app/models") / name,
    ]
    for p in candidates:
        if p and str(p) != "." and p.is_file():
            return p
    return None


class FaceEngine:
    def __init__(self, data_dir: Path):
        self.data_dir = data_dir
        self.enabled = defaultdict(lambda: True)
        self.live_bboxes: dict[str, list] = defaultdict(list)
        self.live_scores: dict[str, list] = defaultdict(list)
        self.live_names: dict[str, list] = defaultdict(list)
        self.live_size: dict[str, tuple] = {}
        self.captures: list[dict] = []
        self._last_capture: dict[str, list] = defaultdict(list)
        self._q: queue.Queue = queue.Queue(maxsize=1)
        self._lock = threading.RLock()
        self._det_lock = threading.Lock()
        self._gallery: list[tuple[str, np.ndarray]] = []
        self._recognizer = None
        self._hog = None
        self._detector = None
        self.ai_url = (os.getenv("AI_URL") or os.getenv("AI_SERVICE_URL") or "").rstrip("/")
        self._http = None
        if self.ai_url:
            import httpx
            self._http = httpx.Client(timeout=4.0)
            print(f"[face] InsightFace via {self.ai_url}", flush=True)

        model = _resolve_model(MODEL_NAME, "FACE_MODEL")
        if model and not self.ai_url:
            self._detector = cv2.FaceDetectorYN.create(
                str(model), "", (DETECT_W, DETECT_H), SCORE_LIVE, 0.3, 5000,
            )
            print(f"[face] YuNet fallback: {model}", flush=True)
            self._hog = cv2.HOGDescriptor((96, 96), (16, 16), (8, 8), (8, 8), 9)
            sface = _resolve_model(SFACE_NAME, "FACE_RECOG_MODEL")
            if sface and hasattr(cv2, "FaceRecognizerSF"):
                try:
                    self._recognizer = cv2.FaceRecognizerSF.create(str(sface), "")
                except Exception:
                    self._recognizer = None
        elif not self.ai_url:
            print("[face] no AI_URL and no YuNet — detection disabled", flush=True)

        self._load_index()
        self._rebuild_gallery()
        self._worker = threading.Thread(target=self._loop, name="face-worker", daemon=True)
        self._worker.start()
        if self.ai_url:
            threading.Thread(target=self._refresh_ai_embeddings, name="face-refresh", daemon=True).start()

    def _refresh_ai_embeddings(self):
        time.sleep(10)
        labeled = []
        with self._lock:
            labeled = [c for c in self.captures if c.get("label")]
        changed = False
        for c in labeled:
            emb = c.get("embedding")
            if isinstance(emb, list) and len(emb) == 512:
                continue
            feat = self._embed_from_path(c.get("face"))
            if feat is not None:
                c["embedding"] = feat.tolist()
                changed = True
        if changed:
            with self._lock:
                self._save_index()
                self._save_registry()
                self._rebuild_gallery()
            print("[face] InsightFace embeddings refreshed for registered faces", flush=True)

    def _index_path(self) -> Path:
        return self.data_dir / "faces.json"

    def _registry_path(self) -> Path:
        return self.data_dir / "faces_registry.json"

    def _is_labeled(self, c: dict | None) -> bool:
        return bool(c and str(c.get("label") or "").strip())

    def _atomic_write(self, path: Path, payload) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(payload, separators=(",", ":")))
        tmp.replace(path)

    def _compact(self, c: dict, keep_embedding: bool) -> dict:
        item = {
            "id": c.get("id"),
            "device": c.get("device"),
            "ts": c.get("ts"),
            "bbox": c.get("bbox"),
            "score": c.get("score"),
            "face": c.get("face"),
            "full": c.get("full"),
            "label": c.get("label"),
            "registered": bool(str(c.get("label") or "").strip()),
        }
        if keep_embedding and c.get("embedding") is not None:
            item["embedding"] = c.get("embedding")
        return item

    def _trim_unlabeled(self) -> list[dict]:
        unlabeled_n: dict[str, int] = defaultdict(int)
        kept: list[dict] = []
        dropped: list[dict] = []
        for c in self.captures:
            if self._is_labeled(c):
                kept.append(c)
                continue
            dev = str(c.get("device") or "_")
            if unlabeled_n[dev] >= MAX_UNLABELED_PER_DEVICE:
                dropped.append(c)
                continue
            unlabeled_n[dev] += 1
            kept.append(c)
        self.captures = kept
        return dropped

    def _delete_files(self, items: list[dict]):
        for found in items:
            for key in ("face", "full"):
                rel = found.get(key)
                if not rel:
                    continue
                try:
                    (self.data_dir / rel).unlink(missing_ok=True)
                except Exception:
                    pass

    def _load_index(self):
        captures: list[dict] = []
        p = self._index_path()
        if p.exists():
            try:
                captures = json.loads(p.read_text())
            except Exception:
                captures = []
        registry: list[dict] = []
        rp = self._registry_path()
        if rp.exists():
            try:
                registry = json.loads(rp.read_text())
            except Exception:
                registry = []

        by_id: dict[str, dict] = {}
        for c in captures:
            cid = c.get("id")
            if cid:
                by_id[str(cid)] = c
        for c in registry:
            cid = c.get("id")
            if not cid:
                continue
            cid = str(cid)
            prev = by_id.get(cid)
            if prev is None or (self._is_labeled(c) and not self._is_labeled(prev)):
                by_id[cid] = c
            elif self._is_labeled(c):
                if c.get("embedding") and not prev.get("embedding"):
                    prev["embedding"] = c["embedding"]
                prev["label"] = c.get("label") or prev.get("label")
                prev["registered"] = True

        self.captures = list(by_id.values())
        self.captures.sort(key=lambda c: float(c.get("ts") or 0), reverse=True)
        for c in self.captures:
            if self._is_labeled(c):
                c["registered"] = True
            if (not self.ai_url) and self._is_labeled(c) and c.get("embedding") is None:
                emb = self._embed_from_path(c.get("face"))
                if emb is not None:
                    c["embedding"] = emb.tolist()

        dropped = self._trim_unlabeled()
        if dropped:
            self._delete_files(dropped)
        try:
            self._save_index()
            self._save_registry()
        except Exception as e:
            print(f"[face] index save on load: {e}", flush=True)

    def _save_index(self):
        payload = [
            self._compact(c, keep_embedding=self._is_labeled(c))
            for c in self.captures
        ]
        self._atomic_write(self._index_path(), payload)

    def _save_registry(self):
        labeled = [
            self._compact(c, keep_embedding=True)
            for c in self.captures
            if self._is_labeled(c)
        ]
        self._atomic_write(self._registry_path(), labeled)

    def set_enabled(self, device: str, on: bool):
        self.enabled[device] = bool(on)
        if not on:
            self.live_bboxes[device] = []
            self.live_scores[device] = []
            self.live_names[device] = []

    def submit(self, device: str, jpeg: bytes):
        if not self.enabled.get(device, True):
            return
        try:
            self._q.put_nowait((device, jpeg, time.time()))
        except queue.Full:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait((device, jpeg, time.time()))
            except queue.Full:
                pass

    def list_captures(self, device: str | None = None, limit: int = 50) -> list[dict]:
        with self._lock:
            items = [c for c in self.captures if not (c.get("label") or "").strip()]
            if device:
                items = [c for c in items if c.get("device") == device]
            out = []
            for c in items[:limit]:
                out.append({
                    "id": c.get("id"),
                    "device": c.get("device"),
                    "ts": c.get("ts"),
                    "bbox": c.get("bbox"),
                    "score": c.get("score"),
                    "face": c.get("face"),
                    "full": c.get("full"),
                    "label": c.get("label"),
                })
            return out

    def capture_devices(self) -> list[str]:
        with self._lock:
            return sorted({
                str(c.get("device"))
                for c in self.captures
                if c.get("device") and not self._is_labeled(c)
            })

    def live_overlay(self, device: str) -> dict:
        with self._lock:
            return {
                "enabled": self.enabled.get(device, True),
                "bboxes": list(self.live_bboxes.get(device, [])),
                "scores": list(self.live_scores.get(device, [])),
                "names": list(self.live_names.get(device, [])),
                "frame": list(self.live_size.get(device, (0, 0))),
            }

    def list_registered(self, device: str | None = None) -> list[dict]:
        """Registered identities are global — device filter is ignored."""
        with self._lock:
            items = [c for c in self.captures if self._is_labeled(c)]
            by_label: dict[str, list] = defaultdict(list)
            for c in items:
                by_label[str(c["label"]).strip()].append(c)
            out = []
            for label, group in sorted(by_label.items(), key=lambda kv: kv[0].lower()):
                first = group[0]
                out.append({
                    "label": label,
                    "count": len(group),
                    "face": first.get("face"),
                    "full": first.get("full"),
                    "ids": [g.get("id") for g in group],
                    "device": first.get("device"),
                    "devices": sorted({str(g.get("device")) for g in group if g.get("device")}),
                    "ts": first.get("ts"),
                })
            return out

    def status(self, device: str) -> dict:
        with self._lock:
            return {
                "enabled": self.enabled.get(device, True),
                "bboxes": list(self.live_bboxes.get(device, [])),
                "scores": list(self.live_scores.get(device, [])),
                "names": list(self.live_names.get(device, [])),
                "frame": list(self.live_size.get(device, (0, 0))),
                "captures": len([c for c in self.captures if c.get("device") == device]),
                "registered": len({c.get("label") for c in self.captures if c.get("label")}),
                "engine": "insightface-buffalo_l" if self.ai_url else (
                    "yunet+sface" if self._recognizer else "yunet"
                ),
            }

    def enroll_from_image(self, device: str, image_bytes: bytes, label: str) -> dict:
        """Register a named person from an uploaded portrait / photo."""
        label = (label or "").strip()
        if not label:
            raise ValueError("nama wajib diisi")
        device = (device or "bodycam-01").strip() or "bodycam-01"
        arr = np.frombuffer(image_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("file gambar tidak valid")
        h, w = img.shape[:2]
        ok, full_buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            raise ValueError("gagal encode gambar")
        jpeg = full_buf.tobytes()

        detected: list[dict] = []
        if self.ai_url:
            try:
                detected = self._detect_ai(jpeg)
            except Exception as e:
                print(f"[face] enroll ai: {e}", flush=True)
        if not detected and self._detector is not None:
            detected = self._detect(img)
            for det in detected:
                det["embedding"] = self._embed_from_row(img, det.get("row"), det["bbox"])

        if detected:
            det = detected[0]
            x, y, fw, fh = det["bbox"]
            pad_x = int(fw * CROP_PAD)
            pad_y = int(fh * CROP_PAD)
            x1 = max(0, x - pad_x)
            y1 = max(0, y - pad_y)
            x2 = min(w, x + fw + pad_x)
            y2 = min(h, y + fh + pad_y)
            crop = img[y1:y2, x1:x2]
            bbox = det["bbox"]
            emb = det.get("embedding")
            score = float(det.get("score") or 1.0)
        else:
            crop = img
            bbox = [0, 0, w, h]
            emb = None
            score = 1.0

        if crop is None or crop.size == 0:
            raise ValueError("tidak ada wajah di foto")

        if emb is None:
            emb = self._embed_crop(crop)

        cid = uuid.uuid4().hex[:10]
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        out_dir = self.data_dir / device / "faces"
        out_dir.mkdir(parents=True, exist_ok=True)
        face_path = out_dir / f"{stamp}_{cid}_face.jpg"
        full_path = out_dir / f"{stamp}_{cid}_full.jpg"
        ok, face_buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if not ok:
            raise ValueError("gagal simpan crop wajah")
        face_path.write_bytes(face_buf.tobytes())
        full_path.write_bytes(jpeg)

        meta = FaceCapture(
            id=cid,
            device=device,
            ts=time.time(),
            bbox=list(bbox),
            score=round(float(score), 3),
            face=str(face_path.relative_to(self.data_dir)).replace("\\", "/"),
            full=str(full_path.relative_to(self.data_dir)).replace("\\", "/"),
            label=label,
            embedding=emb.tolist() if emb is not None else None,
            registered=True,
        )
        item = asdict(meta)
        with self._lock:
            self.captures.insert(0, item)
            dropped = self._trim_unlabeled()
            self._save_index()
            self._save_registry()
            self._rebuild_gallery()
        if dropped:
            self._delete_files(dropped)
        print(f"[face] enrolled {device} {cid} label={label} emb={emb is not None}", flush=True)
        return item

    def set_label(self, face_id: str, label: str | None) -> dict | None:
        label = (label or "").strip() or None
        face_rel = None
        with self._lock:
            item = None
            for c in self.captures:
                if c.get("id") == face_id:
                    item = c
                    break
            if not item:
                return None
            item["label"] = label
            item["registered"] = bool(label)
            face_rel = item.get("face")
        if label:
            emb = self._embed_from_path(face_rel)
            with self._lock:
                if emb is not None:
                    item["embedding"] = emb.tolist()
                self._save_index()
                self._save_registry()
                self._rebuild_gallery()
        else:
            with self._lock:
                self._save_index()
                self._save_registry()
                self._rebuild_gallery()
        return item

    def delete_capture(self, face_id: str) -> bool:
        with self._lock:
            found = None
            for c in self.captures:
                if c.get("id") == face_id:
                    found = c
                    break
            if not found:
                return False
            self.captures = [c for c in self.captures if c.get("id") != face_id]
            self._save_index()
            self._save_registry()
            self._rebuild_gallery()
        for key in ("face", "full"):
            rel = found.get(key)
            if not rel:
                continue
            path = self.data_dir / rel
            try:
                path.unlink(missing_ok=True)
            except Exception:
                pass
        return True

    def delete_captures(self, ids: list[str]) -> int:
        want = {str(i) for i in ids if i}
        if not want:
            return 0
        dropped: list[dict] = []
        with self._lock:
            kept: list[dict] = []
            for c in self.captures:
                if str(c.get("id") or "") in want:
                    dropped.append(c)
                else:
                    kept.append(c)
            if not dropped:
                return 0
            self.captures = kept
            self._save_index()
            self._save_registry()
            self._rebuild_gallery()
        self._delete_files(dropped)
        return len(dropped)

    def delete_unlabeled_for_device(self, device: str) -> int:
        dropped: list[dict] = []
        with self._lock:
            kept: list[dict] = []
            for c in self.captures:
                same = (not device) or str(c.get("device") or "") == device
                if same and not self._is_labeled(c):
                    dropped.append(c)
                else:
                    kept.append(c)
            if not dropped:
                return 0
            self.captures = kept
            self._save_index()
            self._save_registry()
            self._rebuild_gallery()
        self._delete_files(dropped)
        return len(dropped)

    def unregister_label(self, label: str) -> int:
        n = 0
        with self._lock:
            for c in self.captures:
                if c.get("label") == label:
                    c["label"] = None
                    c["registered"] = False
                    n += 1
            if n:
                self._save_index()
                self._save_registry()
                self._rebuild_gallery()
        return n

    def _rebuild_gallery(self):
        gallery: dict[str, dict[int, list[np.ndarray]]] = defaultdict(lambda: defaultdict(list))
        for c in self.captures:
            name = (c.get("label") or "").strip()
            emb = c.get("embedding")
            if not name or not emb:
                continue
            vec = np.asarray(emb, dtype=np.float32).reshape(-1)
            if vec.size < 8:
                continue
            gallery[name][int(vec.size)].append(vec)
        self._gallery = []
        for name, by_dim in gallery.items():
            # Prefer InsightFace 512-d over leftover HOG/SFace vectors.
            dim = 512 if 512 in by_dim else max(by_dim.keys())
            vecs = by_dim[dim]
            mean = np.mean(np.stack(vecs, axis=0), axis=0)
            nrm = np.linalg.norm(mean)
            if nrm > 0:
                mean = mean / nrm
            self._gallery.append((name, mean))

    def _embed_from_path(self, rel: str | None) -> np.ndarray | None:
        if not rel:
            return None
        path = self.data_dir / rel
        if not path.is_file():
            return None
        if self.ai_url:
            try:
                dets = self._detect_ai(path.read_bytes())
                if dets and dets[0].get("embedding") is not None:
                    return dets[0]["embedding"]
            except Exception as e:
                print(f"[face] embed ai error: {e}", flush=True)
        img = cv2.imread(str(path))
        if img is None:
            return None
        return self._embed_crop(img)

    def _detect_ai(self, jpeg: bytes) -> list[dict]:
        if not self._http or not self.ai_url:
            return []
        resp = self._http.post(
            f"{self.ai_url}/face/detect",
            files={"file": ("frame.jpg", jpeg, "image/jpeg")},
            params={"min_score": "0.28"},
        )
        resp.raise_for_status()
        data = resp.json()
        if data.get("width") and data.get("height"):
            self._ai_wh = (int(data["width"]), int(data["height"]))
        out: list[dict] = []
        for face in data.get("faces") or []:
            bbox = face.get("bbox") or []
            if len(bbox) < 4:
                continue
            x1, y1, x2, y2 = [float(v) for v in bbox[:4]]
            bw, bh = int(max(1, x2 - x1)), int(max(1, y2 - y1))
            if bw < MIN_FACE_PX or bh < MIN_FACE_PX:
                continue
            emb = face.get("embedding")
            feat = None
            if emb:
                feat = np.asarray(emb, dtype=np.float32).reshape(-1)
                n = np.linalg.norm(feat)
                if n > 0:
                    feat = feat / n
            out.append({
                "bbox": [int(x1), int(y1), bw, bh],
                "score": float(face.get("confidence") or 0),
                "embedding": feat,
            })
        out.sort(key=lambda t: t["score"], reverse=True)
        return out

    def _embed_crop(self, crop: np.ndarray) -> np.ndarray | None:
        if crop is None or crop.size == 0:
            return None
        if self.ai_url:
            ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if ok:
                try:
                    dets = self._detect_ai(buf.tobytes())
                    if dets and dets[0].get("embedding") is not None:
                        return dets[0]["embedding"]
                except Exception:
                    pass
        if self._recognizer is not None:
            try:
                h, w = crop.shape[:2]
                dummy = np.array(
                    [0, 0, w, h] + [w * 0.3, h * 0.35, w * 0.7, h * 0.35, w * 0.5, h * 0.55,
                                    w * 0.35, h * 0.78, w * 0.65, h * 0.78, 1.0],
                    dtype=np.float32,
                )
                aligned = self._recognizer.alignCrop(crop, dummy)
                feat = self._recognizer.feature(aligned).astype(np.float32).reshape(-1)
                n = np.linalg.norm(feat)
                return feat / n if n else feat
            except Exception:
                pass
        if self._hog is None:
            return None
        gray = cv2.cvtColor(cv2.resize(crop, (96, 96), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        feat = self._hog.compute(gray)
        if feat is None:
            return None
        feat = feat.astype(np.float32).reshape(-1)
        n = np.linalg.norm(feat)
        return feat / n if n else feat

    def _embed_from_row(self, img: np.ndarray, row: np.ndarray, bbox: list) -> np.ndarray | None:
        if self._recognizer is not None:
            try:
                aligned = self._recognizer.alignCrop(img, row)
                feat = self._recognizer.feature(aligned).astype(np.float32).reshape(-1)
                n = np.linalg.norm(feat)
                return feat / n if n else feat
            except Exception:
                pass
        x, y, w, h = [int(v) for v in bbox]
        crop = img[max(0, y): y + h, max(0, x): x + w]
        return self._embed_crop(crop)

    def _match_name(self, feat: np.ndarray | None) -> str | None:
        if feat is None or not self._gallery:
            return None
        feat = feat.astype(np.float32).reshape(-1)
        n = np.linalg.norm(feat)
        if n:
            feat = feat / n
        th = INSIGHT_COSINE_TH if (feat.shape[0] >= 256) else (
            SFACE_COSINE_TH if self._recognizer is not None else 0.78
        )
        best_name, best = None, th
        for name, vec in self._gallery:
            if vec.shape[0] != feat.shape[0]:
                continue
            score = float(np.dot(feat, vec))
            if score > best:
                best, best_name = score, name
        return best_name

    def _iou(self, a, b) -> float:
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        x1, y1 = max(ax, bx), max(ay, by)
        x2, y2 = min(ax + aw, bx + bw), min(ay + ah, by + bh)
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        if inter <= 0:
            return 0.0
        union = aw * ah + bw * bh - inter
        return inter / union if union else 0.0

    def _should_capture(self, device: str, bbox) -> bool:
        now = time.time()
        recent = self._last_capture[device]
        recent[:] = [(t, b) for t, b in recent if now - t < CAPTURE_COOLDOWN_SEC * 3]
        for t, b in recent:
            if now - t < CAPTURE_COOLDOWN_SEC and self._iou(bbox, b) > 0.4:
                return False
        return True

    def _loop(self):
        while True:
            device, jpeg, _ts = self._q.get()
            try:
                self._process(device, jpeg)
            except Exception as e:
                print(f"[face] process error: {e}", flush=True)

    def _enhance(self, img: np.ndarray) -> np.ndarray:
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l2 = clahe.apply(l)
        return cv2.cvtColor(cv2.merge([l2, a, b]), cv2.COLOR_LAB2BGR)

    def _detect(self, img: np.ndarray) -> list[dict]:
        h, w = img.shape[:2]
        scale = min(DETECT_W / w, DETECT_H / h, 1.0)
        dw, dh = max(32, int(w * scale)), max(32, int(h * scale))
        dw -= dw % 2
        dh -= dh % 2
        if dw < 32 or dh < 32:
            return []

        if scale < 0.999:
            resized = cv2.resize(img, (dw, dh), interpolation=cv2.INTER_AREA)
            sx, sy = w / dw, h / dh
        else:
            resized = img
            sx = sy = 1.0
            dw, dh = w, h

        enhanced = self._enhance(resized)
        with self._det_lock:
            self._detector.setInputSize((dw, dh))
            _retval, faces = self._detector.detect(enhanced)

        out: list[dict] = []
        if faces is None:
            return out
        for row in faces:
            x, y, fw, fh = float(row[0]), float(row[1]), float(row[2]), float(row[3])
            score = float(row[-1])
            if score < SCORE_LIVE:
                continue
            bx = int(max(0, x * sx))
            by = int(max(0, y * sy))
            bw = int(min(w - bx, fw * sx))
            bh = int(min(h - by, fh * sy))
            if bw < MIN_FACE_PX or bh < MIN_FACE_PX:
                continue
            scaled = np.array(row, dtype=np.float32).copy()
            scaled[0], scaled[1], scaled[2], scaled[3] = bx, by, bw, bh
            for i in range(4, 14, 2):
                scaled[i] *= sx
                scaled[i + 1] *= sy
            out.append({"bbox": [bx, by, bw, bh], "score": score, "row": scaled})
        out.sort(key=lambda t: t["score"], reverse=True)
        return out

    def _process(self, device: str, jpeg: bytes):
        detected: list[dict] = []
        if self.ai_url:
            try:
                detected = self._detect_ai(jpeg)
            except Exception as e:
                print(f"[face] ai detect error: {e}", flush=True)
                detected = []
        img = None
        if not detected and self._detector is not None:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                return
            detected = self._detect(img)
            for det in detected:
                det["embedding"] = self._embed_from_row(img, det.get("row"), det["bbox"])

        if img is None:
            arr = np.frombuffer(jpeg, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            h = w = 0
        else:
            h, w = img.shape[:2]

        boxes, scores, names = [], [], []
        for det in detected:
            feat = det.get("embedding")
            names.append(self._match_name(feat))
            boxes.append(det["bbox"])
            scores.append(round(float(det["score"]), 3))

        with self._lock:
            self.live_bboxes[device] = boxes
            self.live_scores[device] = scores
            self.live_names[device] = names
            sz = getattr(self, "_ai_wh", None)
            if sz:
                self.live_size[device] = sz
            elif w and h:
                self.live_size[device] = (w, h)

        if not detected or img is None:
            return

        for det, name in zip(detected, names):
            if name:
                continue
            if det["score"] < (AI_CAPTURE_SCORE if self.ai_url else SCORE_CAPTURE):
                continue
            if not self._should_capture(device, det["bbox"]):
                continue
            self._save_capture(
                device, img, jpeg, det["bbox"], det["score"], w, h,
                embedding=det.get("embedding"),
            )

    def _save_capture(
        self, device: str, img, jpeg: bytes, bbox, score: float, frame_w: int, frame_h: int,
        embedding=None,
    ):
        x, y, fw, fh = bbox
        pad_x = int(fw * CROP_PAD)
        pad_y = int(fh * CROP_PAD)
        x1 = max(0, x - pad_x)
        y1 = max(0, y - pad_y)
        x2 = min(frame_w, x + fw + pad_x)
        y2 = min(frame_h, y + fh + pad_y)
        crop = img[y1:y2, x1:x2]
        if crop.size == 0:
            return

        cid = uuid.uuid4().hex[:10]
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        out_dir = self.data_dir / device / "faces"
        out_dir.mkdir(parents=True, exist_ok=True)
        face_path = out_dir / f"{stamp}_{cid}_face.jpg"
        full_path = out_dir / f"{stamp}_{cid}_full.jpg"

        ok, face_buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if not ok:
            return
        face_path.write_bytes(face_buf.tobytes())
        full_path.write_bytes(jpeg)

        emb = embedding if embedding is not None else self._embed_crop(crop)
        matched = self._match_name(emb)
        if matched:
            try:
                face_path.unlink(missing_ok=True)
                full_path.unlink(missing_ok=True)
            except Exception:
                pass
            print(f"[face] skip capture {device} known={matched}", flush=True)
            return
        meta = FaceCapture(
            id=cid,
            device=device,
            ts=time.time(),
            bbox=bbox,
            score=round(float(score), 3),
            face=str(face_path.relative_to(self.data_dir)),
            full=str(full_path.relative_to(self.data_dir)),
            label=None,
            embedding=emb.tolist() if emb is not None else None,
            registered=False,
        )
        with self._lock:
            self.captures.insert(0, asdict(meta))
            dropped = self._trim_unlabeled()
            self._last_capture[device].append((time.time(), bbox))
            self._save_index()
        if dropped:
            self._delete_files(dropped)
        print(f"[face] captured {device} {cid} score={score:.2f}", flush=True)

    def annotate_jpeg(self, device: str, jpeg: bytes) -> bytes:
        boxes = self.live_bboxes.get(device) or []
        scores = self.live_scores.get(device) or []
        names = self.live_names.get(device) or []
        if not boxes:
            return jpeg
        arr = np.frombuffer(jpeg, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            return jpeg
        for i, (x, y, w, h) in enumerate(boxes):
            sc = scores[i] if i < len(scores) else 0
            name = names[i] if i < len(names) else None
            color = (0, 220, 80) if name else ((0, 220, 80) if sc >= SCORE_CAPTURE else (0, 180, 220))
            if name:
                color = (46, 230, 214)
            cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
            label = name if name else f"UNKNOWN {int(sc * 100)}%"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            ty = max(th + 6, y - 8)
            cv2.rectangle(img, (x, ty - th - 6), (x + tw + 8, ty + 4), (0, 0, 0), -1)
            cv2.putText(
                img,
                label,
                (x + 4, ty),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        return buf.tobytes() if ok else jpeg
