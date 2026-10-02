#!/usr/bin/env python3
"""Stress test for bodycam FastAPI gateway.

Spawns N virtual ESP32 bodycams (senders) that push JPEG frames at the
configured live/record rates, and M browser-side viewers that consume the
MJPEG stream. Prints per-device stats and aggregates.

Usage:
    python stress_10bodycam.py --server http://127.0.0.1:7890 --token X01040688 \
        --num-cams 10 --num-viewers 10 --duration 60 --mode live
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import statistics
import sys
import time
from dataclasses import dataclass, field

import aiohttp


SAMPLE_720P = os.path.join(os.path.dirname(__file__), "sample_720p.jpg")
SAMPLE_1080P = os.path.join(os.path.dirname(__file__), "sample_1080p.jpg")


@dataclass
class DeviceStats:
    device: str
    mode: str = "live"
    target_fps: int = 30
    bytes_sent: int = 0
    frames_sent: int = 0
    frames_recv: int = 0
    bytes_recv: int = 0
    dropped_send: int = 0
    latencies_ms: list[float] = field(default_factory=list)
    last_recv_ts: float = 0.0
    start_ts: float = 0.0
    end_ts: float = 0.0


async def sender(
    session: aiohttp.ClientSession,
    server: str,
    token: str,
    stats: DeviceStats,
    jpeg_bytes: bytes,
    running: asyncio.Event,
):
    """Pushes JPEG frames over WS /ws/device at the configured target FPS.

    Mirrors the firmware: 1-byte kind (0x01) + JPEG payload. Server treats
    these as new live frames; face detection and record path are also
    exercised when 'mode' is set to 'video' for at least one device.
    """
    url = f"{server}/ws/device?device={stats.device}&token={token}"
    stats.start_ts = time.monotonic()
    period = 1.0 / stats.target_fps
    try:
        async with session.ws_connect(url, autoclose=False, autoping=True) as ws:
            next_t = time.monotonic()
            while running.is_set():
                now = time.monotonic()
                if now < next_t:
                    await asyncio.sleep(next_t - now)
                t_send = time.monotonic()
                await ws.send_bytes(b"\x01" + jpeg_bytes)
                stats.bytes_sent += len(jpeg_bytes)
                stats.frames_sent += 1
                stats.latencies_ms.append((time.monotonic() - t_send) * 1000.0)
                next_t += period
                # keep only last 1000 latency samples
                if len(stats.latencies_ms) > 1000:
                    stats.latencies_ms = stats.latencies_ms[-1000:]
    except Exception as e:
        print(f"[sender {stats.device}] {e}", file=sys.stderr)
    stats.end_ts = time.monotonic()


async def viewer(
    session: aiohttp.ClientSession,
    server: str,
    stats: DeviceStats,
    running: asyncio.Event,
):
    """Pulls MJPEG stream for one device. Counts frames, measures inter-arrival."""
    url = f"{server}/api/v1/mjpeg?device={stats.device}&annotate=0"
    stats.start_ts = time.monotonic()
    try:
        async with session.get(url, timeout=None) as resp:
            if resp.status != 200:
                print(f"[viewer {stats.device}] HTTP {resp.status}", file=sys.stderr)
                return
            buf = b""
            boundary = b"--frame"
            last = time.monotonic()
            async for chunk in resp.content.iter_chunked(8192):
                if not running.is_set():
                    return
                buf += chunk
                while True:
                    idx = buf.find(boundary)
                    if idx < 0:
                        if len(buf) > 1_000_000:
                            buf = b""
                        break
                    next_idx = buf.find(boundary, idx + len(boundary))
                    if next_idx < 0:
                        buf = buf[idx:]
                        break
                    chunk_bytes = buf[idx + len(boundary):next_idx]
                    buf = buf[next_idx:]
                    if b"\r\n\r\n" in chunk_bytes:
                        head, _, body = chunk_bytes.partition(b"\r\n\r\n")
                        body = body.rstrip(b"\r\n")
                        stats.frames_recv += 1
                        stats.bytes_recv += len(body)
                        stats.last_recv_ts = time.monotonic()
                        last = stats.last_recv_ts
    except Exception as e:
        print(f"[viewer {stats.device}] {e}", file=sys.stderr)
    stats.end_ts = time.monotonic()


async def poll_health(session: aiohttp.ClientSession, server: str, duration: float) -> list[dict]:
    """Sample /health every second during the run for CPU/RAM-like signals."""
    snapshots = []
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        try:
            async with session.get(f"{server}/health", timeout=aiohttp.ClientTimeout(total=4)) as r:
                data = await r.json()
                snapshots.append(data)
        except Exception:
            pass
        await asyncio.sleep(1.0)
    return snapshots


def pct(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    return statistics.quantiles(values, n=100)[int(p) - 1] if len(values) >= 100 else min(values)


def print_table(stats: list[DeviceStats], snapshots: list[dict]):
    print("=" * 100)
    print(f"{'device':14} {'role':8} {'sent_fps':9} {'recv_fps':9} {'drop_pct':9} {'p50_ms':8} {'p95_ms':8} {'bytes':>10}")
    print("-" * 100)
    for s in stats:
        dur = max(0.001, s.end_ts - s.start_ts)
        sent_fps = s.frames_sent / dur
        recv_fps = s.frames_recv / dur if s.frames_recv else 0.0
        drop_pct = (s.dropped_send / max(1, s.frames_sent)) * 100.0
        lat = s.latencies_ms
        print(
            f"{s.device:14} {s.mode:8} {sent_fps:>9.2f} {recv_fps:>9.2f} "
            f"{drop_pct:>9.2f} {pct(lat, 50):>8.1f} {pct(lat, 95):>8.1f} {s.bytes_sent:>10}"
        )
    print("-" * 100)
    if snapshots:
        latest = snapshots[-1]
        devs = latest.get("devices") or []
        cpu_hint = latest.get("counters", {}) if latest else {}
        print(f"SERVER HEALTH: devices_online={len(devs)} {devs}")
        if "video_fps_actual" in latest:
            print(f"  per-device video_fps_actual={latest.get('video_fps_actual')}")
        if "audio_rate_hz" in latest:
            print(f"  per-device audio_rate_hz={latest.get('audio_rate_hz')}")
    print("=" * 100)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="http://127.0.0.1:7890")
    ap.add_argument("--token", default=os.getenv("BODYCAM_TOKEN", "X01040688"))
    ap.add_argument("--num-cams", type=int, default=10)
    ap.add_argument("--num-viewers", type=int, default=10)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--mode", choices=["live", "record1", "recordall"], default="live")
    args = ap.parse_args()

    with open(SAMPLE_720P, "rb") as f:
        jpeg_720p = f.read()
    with open(SAMPLE_1080P, "rb") as f:
        jpeg_1080p = f.read()

    stats: list[DeviceStats] = []
    for i in range(args.num_cams):
        if args.mode == "recordall":
            mode = "video"
            jpeg = jpeg_1080p
            fps = 15
        elif args.mode == "record1" and i == 0:
            mode = "video"
            jpeg = jpeg_1080p
            fps = 15
        else:
            mode = "live"
            jpeg = jpeg_720p
            fps = 30
        stats.append(
            DeviceStats(
                device=f"stress-{i:02d}",
                mode=mode,
                target_fps=fps,
            )
        )

    running = asyncio.Event()
    running.set()

    async with aiohttp.ClientSession() as session:
        senders = [
            asyncio.create_task(sender(session, args.server, args.token, s, jpeg_720p if s.mode == "live" else jpeg_1080p, running))
            for s in stats
        ]
        viewers = [
            asyncio.create_task(viewer(session, args.server, s, running))
            for s in stats[: args.num_viewers]
        ]
        health = asyncio.create_task(poll_health(session, args.server, args.duration))

        print(f"Running stress: {args.num_cams} cams × {args.num_viewers} viewers for {args.duration}s · mode={args.mode}")
        await asyncio.sleep(args.duration)
        running.clear()
        for t in senders + viewers:
            t.cancel()
        for t in senders + viewers:
            try:
                await t
            except (asyncio.CancelledError, Exception):
                pass
        snapshots = await health

    print_table(stats, snapshots)
    out = {
        "duration_sec": args.duration,
        "mode": args.mode,
        "num_cams": args.num_cams,
        "stats": [
            {
                "device": s.device,
                "mode": s.mode,
                "frames_sent": s.frames_sent,
                "frames_recv": s.frames_recv,
                "bytes_sent": s.bytes_sent,
                "bytes_recv": s.bytes_recv,
                "latency_p95_ms": pct(s.latencies_ms, 95),
                "target_fps": s.target_fps,
            }
            for s in stats
        ],
        "server_snapshots": snapshots[-3:],
    }
    out_path = os.path.join(os.path.dirname(__file__), "stress_results.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"Saved {out_path}")


if __name__ == "__main__":
    asyncio.run(main())