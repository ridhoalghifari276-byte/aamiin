FROM python:3.12-slim

WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 curl libgomp1 build-essential g++ \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY app ./app

ENV INSIGHTFACE_ROOT=/app/models/insightface
ENV INSIGHTFACE_MODEL=buffalo_l

# YOLOv8n ONNX (optional — falls back to HOG if missing)
RUN mkdir -p app/models \
    && (curl -fsSL -o app/models/yolov8n.onnx \
      https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.onnx \
      || echo "YOLO model download skipped")

# Pre-download InsightFace buffalo_l weights (det + recognition)
RUN python -c "\
from insightface.app import FaceAnalysis; \
import os; \
root=os.environ['INSIGHTFACE_ROOT']; \
os.makedirs(root, exist_ok=True); \
app=FaceAnalysis(name=os.environ['INSIGHTFACE_MODEL'], root=root, providers=['CPUExecutionProvider']); \
app.prepare(ctx_id=-1, det_size=(640,640)); \
print('InsightFace ready')" \
    || echo "InsightFace model download skipped — will retry at runtime"

EXPOSE 8001

CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8001", "--workers", "1"]
