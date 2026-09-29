"""InsightFace buffalo_l — local server-side face detection + 512-d embeddings."""

from __future__ import annotations

import os
import threading
from typing import Any

import cv2
import numpy as np

INSIGHTFACE_ROOT = os.environ.get("INSIGHTFACE_ROOT", "/app/models/insightface")
MODEL_NAME = os.environ.get("INSIGHTFACE_MODEL", "buffalo_l")
MAX_FACE_SIDE = int(os.environ.get("FACE_MAX_SIDE", "1280"))

_face_app = None
_engine_ready = False
_engine_error: str | None = None
_lock = threading.Lock()


def _safe_int(value: object) -> int | None:
    if value is None:
        return None
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _parse_gender(face: object) -> str | None:
    raw = getattr(face, "sex", None)
    if raw is None:
        raw = getattr(face, "gender", None)
    if raw is None:
        return None
    if isinstance(raw, str):
        label = raw.strip().upper()
        if label in ("M", "MALE", "MAN"):
            return "M"
        if label in ("F", "FEMALE", "WOMAN"):
            return "F"
        num = _safe_int(label)
        if num is not None:
            return "M" if num == 1 else "F"
        return None
    num = _safe_int(raw)
    if num is None:
        return None
    return "M" if num == 1 else "F"


def _get_app():
    global _face_app, _engine_ready, _engine_error
    if _face_app is not None:
        return _face_app
    with _lock:
        if _face_app is not None:
            return _face_app
        try:
            from insightface.app import FaceAnalysis

            os.makedirs(INSIGHTFACE_ROOT, exist_ok=True)
            app = FaceAnalysis(
                name=MODEL_NAME,
                root=INSIGHTFACE_ROOT,
                providers=["CPUExecutionProvider"],
            )
            app.prepare(ctx_id=-1, det_size=(640, 640))
            _face_app = app
            _engine_ready = True
            _engine_error = None
            return _face_app
        except Exception as exc:
            _engine_error = str(exc)
            raise


def warmup() -> None:
    """Load model at container start so first M530 scan does not hit cold-start 503."""
    app = _get_app()
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    with _lock:
        app.get(dummy)


def engine_status() -> dict[str, Any]:
    return {
        "ready": _engine_ready,
        "model": MODEL_NAME,
        "root": INSIGHTFACE_ROOT,
        "error": _engine_error,
        "max_side": MAX_FACE_SIDE,
        "offline": True,
    }


def _resize_for_face(frame: np.ndarray, max_side: int) -> tuple[np.ndarray, float]:
    h, w = frame.shape[:2]
    longest = max(h, w)
    if longest <= max_side:
        return frame, 1.0
    scale = max_side / float(longest)
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return resized, 1.0 / scale


def analyze_faces_bgr(frame: np.ndarray, *, min_score: float = 0.45) -> list[dict[str, Any]]:
    if frame is None or frame.size == 0:
        return []
    work, scale = _resize_for_face(frame, MAX_FACE_SIDE)
    app = _get_app()
    with _lock:
        faces = app.get(work)
    results: list[dict[str, Any]] = []
    for face in faces:
        score = float(face.det_score)
        if score < min_score:
            continue
        x1, y1, x2, y2 = face.bbox.tolist()
        if scale != 1.0:
            x1, y1, x2, y2 = x1 * scale, y1 * scale, x2 * scale, y2 * scale
        embedding = face.embedding
        if embedding is None:
            continue
        norm = float(np.linalg.norm(embedding))
        if norm < 1e-6:
            continue
        embedding = (embedding / norm).tolist()
        gender = _parse_gender(face)
        age_val = _safe_int(getattr(face, "age", None))
        results.append(
            {
                "bbox": [float(x1), float(y1), float(x2), float(y2)],
                "confidence": score,
                "embedding": embedding,
                "age": age_val,
                "gender": gender,
            }
        )
    results.sort(key=lambda item: item["confidence"], reverse=True)
    return results[:10]


def crop_face_jpeg(frame: np.ndarray, bbox: list[float], quality: int = 85) -> bytes | None:
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = [int(v) for v in bbox]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    if x2 <= x1 or y2 <= y1:
        return None
    crop = frame[y1:y2, x1:x2]
    ok, buf = cv2.imencode(".jpg", crop, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        return None
    return buf.tobytes()
