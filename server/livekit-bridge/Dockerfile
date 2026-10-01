FROM golang:1.24-bookworm AS build
ENV GOTOOLCHAIN=auto
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config libopus-dev gcc libc6-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY go.mod main.go ./
RUN go get gopkg.in/hraban/opus.v2@latest \
    && go mod tidy \
    && CGO_ENABLED=1 go build -o /out/bridge .

FROM debian:bookworm-slim
RUN apt-get update && apt-get install -y --no-install-recommends \
    ffmpeg libopus0 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=build /out/bridge /bridge
ENTRYPOINT ["/bridge"]
