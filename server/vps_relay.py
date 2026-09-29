"""TCP relay: laptop LAN :7890 -> public VPS. ESP can reach the laptop
when the meeting AP blocks the ESP from the internet."""
import socket
import threading
import sys

UP = (sys.argv[1] if len(sys.argv) > 1 else "45.250.101.17",
      int(sys.argv[2]) if len(sys.argv) > 2 else 7890)
LISTEN = int(sys.argv[3]) if len(sys.argv) > 3 else 7890


def pipe(src, dst):
    try:
        while True:
            data = src.recv(65536)
            if not data:
                break
            dst.sendall(data)
    except OSError:
        pass
    try:
        src.close()
        dst.close()
    except OSError:
        pass


def handle(client, addr):
    try:
        up = socket.create_connection(UP, 8)
    except OSError as exc:
        print(f"[relay] {addr} -> {UP} fail {exc}", flush=True)
        client.close()
        return
    print(f"[relay] {addr} -> {UP}", flush=True)
    threading.Thread(target=pipe, args=(client, up), daemon=True).start()
    pipe(up, client)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", LISTEN))
    sock.listen(32)
    print(f"[relay] 0.0.0.0:{LISTEN} -> {UP[0]}:{UP[1]}", flush=True)
    while True:
        client, addr = sock.accept()
        threading.Thread(target=handle, args=(client, addr), daemon=True).start()


if __name__ == "__main__":
    main()
