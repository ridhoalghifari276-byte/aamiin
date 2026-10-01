FROM golang:1.22-bookworm AS build
WORKDIR /src
COPY gateway/go.mod gateway/main.go ./
RUN CGO_ENABLED=0 go build -o /out/gateway .

FROM debian:bookworm-slim
RUN apt-get update \
 && apt-get install -y --no-install-recommends ffmpeg ca-certificates \
 && rm -rf /var/lib/apt/lists/*
COPY --from=build /out/gateway /gateway
COPY web /web
ENV RT_WEB=/web
ENV RT_LISTEN=:50310
ENV RT_HTTP=:9090
ENV RT_RTSP=rtsp://mediamtx:8554
ENV RT_MEDIAMTX=http://mediamtx:8889
EXPOSE 9090 50310/udp
ENTRYPOINT ["/gateway"]
