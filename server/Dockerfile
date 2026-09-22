FROM python:3.12-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg libglib2.0-0 libgomp1 curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY models ./models
COPY app.py faces.py pqtalkie.py ./
COPY static ./static
ENV FACE_MODEL=/app/models/face_detection_yunet_2023mar.onnx
ENV FACE_RECOG_MODEL=/app/models/face_recognition_sface_2021dec.onnx
RUN curl -fsSL -o /app/models/face_recognition_sface_2021dec.onnx \
    "https://github.com/opencv/opencv_zoo/raw/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx" \
    || echo "SFace download skipped"
CMD ["uvicorn","app:app","--host","0.0.0.0","--port","8080","--timeout-keep-alive","15","--timeout-graceful-shutdown","10","--ws-ping-interval","0"]
