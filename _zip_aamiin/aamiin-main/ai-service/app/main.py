"""
M530-TACS AI Service — YOLOv8n object detection + InsightFace recognition.
All inference runs locally in Docker (no cloud API).
"""

from __future__ import annotations

import asyncio
import io
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import cv2
import numpy as np
from fastapi import FastAPI, File, HTTPException, UploadFile, Query
from PIL import Image, ImageOps

from app.face_engine import analyze_faces_bgr, engine_status, warmup

MODELS_DIR = Path(__file__).resolve().parent / "models"
ONNX_PATH = MODELS_DIR / "yolov8n.onnx"

# COCO class names (YOLOv8)
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

# Tactical-relevant labels
PRIORITY_LABELS = {
    "person", "car", "truck", "bus", "motorcycle", "bicycle",
    "backpack", "suitcase", "knife", "cell phone",
}

HOG = cv2.HOGDescriptor()
HOG.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

_ort_session = None


@asynccontextmanager
async def lifespan(_app: FastAPI):
    """Preload InsightFace so M530 face scan never hits cold-start 503."""
    loop = asyncio.get_running_loop()
    try:
        await loop.run_in_executor(None, warmup)
    except Exception:
        pass
    yield


app = FastAPI(title="M530-TACS AI Service", version="1.3.0", lifespan=lifespan)


def _get_yolo_session():
    global _ort_session
    if _ort_session is not None:
        return _ort_session
    if not ONNX_PATH.exists():
        return None
    try:
        import onnxruntime as ort

        _ort_session = ort.InferenceSession(
            str(ONNX_PATH),
            providers=["CPUExecutionProvider"],
        )
        return _ort_session
    except Exception:
        return None


def _load_image(data: bytes) -> np.ndarray:
    image = ImageOps.exif_transpose(Image.open(io.BytesIO(data)).convert("RGB"))
    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)


def _letterbox(img: np.ndarray, size: int = 640) -> tuple[np.ndarray, float, tuple[int, int]]:
    h, w = img.shape[:2]
    scale = min(size / h, size / w)
    nh, nw = int(h * scale), int(w * scale)
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    top = (size - nh) // 2
    left = (size - nw) // 2
    canvas[top : top + nh, left : left + nw] = resized
    return canvas, scale, (left, top)


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thresh: float = 0.45) -> list[int]:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[1:][iou <= iou_thresh]
    return keep


def detect_yolo(frame: np.ndarray, conf_thresh: float = 0.35) -> list[dict[str, Any]]:
    session = _get_yolo_session()
    if session is None:
        return []

    h0, w0 = frame.shape[:2]
    img, scale, (pad_x, pad_y) = _letterbox(frame, 640)
    blob = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    blob = np.expand_dims(blob, 0)

    input_name = session.get_inputs()[0].name
    out = session.run(None, {input_name: blob})[0]
    if out.ndim == 3:
        out = out[0]
    if out.shape[0] in (84, 85):
        out = out.T

    boxes_raw = out[:, :4]
    scores_all = out[:, 4:]
    class_ids = np.argmax(scores_all, axis=1)
    confidences = scores_all[np.arange(len(class_ids)), class_ids]

    mask = confidences >= conf_thresh
    boxes_raw = boxes_raw[mask]
    confidences = confidences[mask]
    class_ids = class_ids[mask]

    if len(boxes_raw) == 0:
        return []

    # xywh -> xyxy in original image coords
    cx, cy, bw, bh = boxes_raw.T
    x1 = (cx - bw / 2 - pad_x) / scale
    y1 = (cy - bh / 2 - pad_y) / scale
    x2 = (cx + bw / 2 - pad_x) / scale
    y2 = (cy + bh / 2 - pad_y) / scale
    boxes = np.stack([x1, y1, x2, y2], axis=1)
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, w0)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, h0)

    keep = _nms(boxes, confidences)
    detections: list[dict[str, Any]] = []
    for i in keep[:20]:
        cid = int(class_ids[i])
        label = COCO_NAMES[cid] if cid < len(COCO_NAMES) else f"class_{cid}"
        x1i, y1i, x2i, y2i = boxes[i]
        detections.append(
            {
                "label": label.title(),
                "confidence": float(confidences[i]),
                "bbox": [float(x1i), float(y1i), float(x2i - x1i), float(y2i - y1i)],
                "priority": label in PRIORITY_LABELS,
            }
        )
    detections.sort(key=lambda d: (d["priority"], d["confidence"]), reverse=True)
    return detections


def detect_hog(frame: np.ndarray) -> list[dict[str, Any]]:
    h, w = frame.shape[:2]
    scale = 640 / max(w, 1)
    work = cv2.resize(frame, (int(w * scale), int(h * scale))) if scale < 1 else frame
    inv = 1 / scale if scale < 1 else 1.0
    boxes, weights = HOG.detectMultiScale(work, winStride=(8, 8), padding=(8, 8), scale=1.05)
    out: list[dict[str, Any]] = []
    for (x, y, bw, bh), conf in zip(boxes, weights):
        c = float(conf[0] if hasattr(conf, "__len__") else conf)
        if c < 0.4:
            continue
        out.append(
            {
                "label": "Person",
                "confidence": float(min(0.95, c)),
                "bbox": [float(x * inv), float(y * inv), float(bw * inv), float(bh * inv)],
                "priority": True,
            }
        )
    return out


@app.get("/health")
async def health() -> dict:
    engine = "yolov8n-onnx" if ONNX_PATH.exists() else "opencv-hog"
    face = engine_status()
    return {
        "status": "ok" if face.get("ready") else "degraded",
        "engine": engine,
        "onnx_ready": ONNX_PATH.exists(),
        "face_engine": "insightface-buffalo_l",
        "face_ready": face.get("ready", False),
        "face_error": face.get("error"),
        "face_offline": True,
        "version": "1.3.0",
    }


@app.post("/detect")
async def detect(file: UploadFile = File(...)) -> dict:
    data = await file.read()
    frame = _load_image(data)
    detections = detect_yolo(frame)
    engine = "yolov8n-onnx"
    if not detections:
        detections = detect_hog(frame)
        engine = "opencv-hog-fallback"
    return {
        "detections": detections,
        "count": len(detections),
        "engine": engine,
    }


@app.post("/face/detect")
async def face_detect(file: UploadFile = File(...), min_score: float = Query(0.28)) -> dict:
    """InsightFace: detect faces + 512-d normalized embeddings."""
    data = await file.read()
    if len(data) < 500:
        raise HTTPException(status_code=400, detail="Image too small")
    frame = _load_image(data)
    try:
        faces = analyze_faces_bgr(frame, min_score=min_score)
    except Exception as exc:
        raise HTTPException(
            status_code=503,
            detail=f"InsightFace local engine error: {exc}",
        ) from exc
    h, w = frame.shape[:2]
    return {
        "faces": faces,
        "count": len(faces),
        "width": int(w),
        "height": int(h),
        "engine": "insightface-buffalo_l",
        "offline": True,
    }
