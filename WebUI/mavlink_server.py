#!/usr/bin/env python3
"""Standalone local MAVLink monitor. Does not import the flight simulation stack."""
from __future__ import annotations

import argparse
import importlib.util
import json
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from mavlink_telemetry import TelemetryService

ROOT = Path(__file__).resolve().parent
ASSETS = {"/": "mavlink.html", "/mavlink.html": "mavlink.html",
          "/mavlink.js": "mavlink.js", "/mavlink.css": "mavlink.css",
          "/mavlink_plot.mjs": "mavlink_plot.mjs"}


class MonitorServer(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, address):
        self.telemetry = TelemetryService()
        super().__init__(address, MonitorHandler)

    def server_close(self):
        try:
            self.telemetry.disconnect()
        finally:
            super().server_close()


class MonitorHandler(BaseHTTPRequestHandler):
    server_version = "RLFlightMavlink/1.0"

    def send_payload(self, payload, status=200):
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        if self.path.startswith("/api/samples") and len(args) > 1 and str(args[1]) == "200":
            return
        super().log_message(fmt, *args)

    def do_GET(self):
        parsed = urlparse(self.path)
        service = self.server.telemetry
        try:
            if parsed.path == "/api/samples":
                query = parse_qs(parsed.query)
                cursor = max(0, int(query.get("cursor", ["0"])[0]))
                if query.get("epoch", [""])[0] != service.store.epoch:
                    cursor = 0
                payload = service.store.snapshot(cursor)
                payload["connection"] = service.status()
                self.send_payload(payload)
            elif parsed.path == "/api/ports":
                try:
                    from serial.tools import list_ports
                    ports = [{"device": p.device, "description": p.description} for p in list_ports.comports()]
                except ImportError:
                    ports = []
                self.send_payload({"ports": ports, "dependencies": bool(importlib.util.find_spec("pymavlink"))})
            elif parsed.path in ASSETS:
                path = ROOT / ASSETS[parsed.path]
                data = path.read_bytes()
                mime = {".html": "text/html", ".js": "text/javascript", ".mjs": "text/javascript", ".css": "text/css"}[path.suffix]
                self.send_response(200)
                self.send_header("Content-Type", f"{mime}; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_error(404)
        except (ValueError, TypeError) as exc:
            self.send_payload({"error": str(exc)}, 400)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        # The local serial API accepts only same-origin browser requests.
        origin, host = self.headers.get("Origin"), self.headers.get("Host")
        if origin and urlparse(origin).netloc != host:
            self.send_payload({"error": "仅允许同源请求"}, 403)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8192:
                raise ValueError("请求正文长度无效")
            if self.headers.get("Content-Type", "").split(";", 1)[0] != "application/json":
                raise ValueError("请求需要 application/json")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("请求正文必须为对象")
            actions = {"/api/connect": self.server.telemetry.connect,
                       "/api/disconnect": lambda _: self.server.telemetry.disconnect(),
                       "/api/request-stream": self.server.telemetry.request_stream}
            action = actions.get(urlparse(self.path).path)
            if action is None:
                self.send_error(404)
                return
            self.send_payload(action(body))
        except (ValueError, TypeError, OverflowError) as exc:
            self.send_payload({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--demo", action="store_true", help="启动后连接演示数据")
    args = parser.parse_args()
    server = MonitorServer((args.host, args.port))
    if args.demo:
        server.telemetry.connect({"transport": "demo"})
    print(f"MAVLink monitor: http://{args.host}:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
