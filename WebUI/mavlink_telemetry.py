"""Read-only MAVLink telemetry collection, independent of simulation and CEKF."""
from __future__ import annotations

import math
import queue
import re
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass
from typing import Any

IMU_NAMES = tuple(
    f"{aircraft}_{sensor}_{frame}_{axis}"
    for sensor in ("ACC", "GYR") for frame in ("R", "C")
    for aircraft in ("A", "B") for axis in "XYZ"
)
# Matches the firmware's former IMU_AB_C array layout as well as named scalars.
IMU_PATTERN = re.compile(r"^([AB])_(ACC|GYR)_([RC])_([XYZ])$")
VECTOR_ALIASES = {
    "CE_A_ACC": "A_ACC_R", "CE_B_ACC": "B_ACC_R",
    "CE_A_ACCC": "A_ACC_C", "CE_B_ACCC": "B_ACC_C",
    "CE_A_GYR": "A_GYR_R", "CE_B_GYR": "B_GYR_R",
    "CE_A_GYRC": "A_GYR_C", "CE_B_GYRC": "B_GYR_C",
    "A_ACC_RAW": "A_ACC_R", "B_ACC_RAW": "B_ACC_R",
    "A_GYR_RAW": "A_GYR_R", "B_GYR_RAW": "B_GYR_R",
}


# Names fit MAVLink NAMED_VALUE_FLOAT's ten-byte name field (nine plus NUL).
COMP_FIELDS = {
    "C_ROLL": ("组合体 Roll", "deg", "CEKF / 姿态"),
    "C_PITCH": ("组合体 Pitch", "deg", "CEKF / 姿态"),
    "C_YAW": ("组合体 Yaw", "deg", "CEKF / 姿态"),
    **{f"C_{sensor}_{axis}": (f"组合体 · {label} {axis}", unit, f"CEKF / {label}")
       for sensor, label, unit in (("ACC", "虚拟比力", "m/s²"), ("GYR", "虚拟角速度", "rad/s"),
                                   ("ALP", "角加速度", "rad/s²")) for axis in "XYZ"},
    "CE_RUN": ("模块运行心跳 · 0 停止/过期，1 运行", "", "CEKF / 状态"),
    "CE_EN": ("CEKF 启用", "", "CEKF / 状态"),
    "CE_INIT": ("主机姿态初始化完成", "", "CEKF / 状态"),
    "CE_STATE": ("状态 · 0 初始化，1 双机，2 主机单独，3 故障", "", "CEKF / 状态"),
    "CE_LINK": ("Airlink · 0 断开，1 连接", "", "CEKF / 状态"),
    "CE_W_A": ("主机 IMU 权重", "", "CEKF / 权重"),
    "CE_W_B": ("从机 IMU 权重", "", "CEKF / 权重"),
    "CE_VALID": ("融合输出有效", "", "CEKF / 状态"),
    "CE_RESET": ("CEKF 重置计数", "", "CEKF / 状态"),
    "CE_FLAGS": ("融合健康标志位", "", "CEKF / 状态"),
    "CE_AGE_A": ("主机输入年龄", "ms", "CEKF / 数据年龄"),
    "CE_AGE_B": ("从机输入年龄 · 断链为 0", "ms", "CEKF / 数据年龄"),
    "CE_FRAMES": ("双机融合样本计数", "", "CEKF / 状态"),
    "CE_YAWREF": ("下发 yaw 参考", "deg", "CEKF / 姿态"),
    "CE_YSIG": ("Yaw 参考标准差", "rad", "CEKF / 不确定度"),
}
DEMO_NAMES = IMU_NAMES + tuple(COMP_FIELDS)


def text_name(value: Any) -> str:
    if isinstance(value, bytes):
        value = value.decode("utf-8", "replace")
    return str(value).split("\0", 1)[0][:80]


def normalize_name(name: str) -> str:
    # Earlier firmware used A_ACCR_X / A_GYRC_X.
    return re.sub(r"^([AB])_(ACC|GYR)([RC])_([XYZ])$", r"\1_\2_\3_\4", name)


def describe(name: str) -> dict[str, str]:
    if name in COMP_FIELDS:
        label, unit, group = COMP_FIELDS[name]
        return {"label": label, "unit": unit, "group": group}
    match = IMU_PATTERN.fullmatch(name)
    if not match:
        return {"label": name, "unit": "", "group": "其他字段"}
    aircraft, sensor, frame, axis = match.groups()
    body = "主机 A" if aircraft == "A" else "从机 B"
    quantity, unit = ("加速度", "m/s²") if sensor == "ACC" else ("角速度", "rad/s")
    coordinate = "原始" if frame == "R" else "C 系"
    return {"label": f"{body} · {coordinate}{quantity} {axis}", "unit": unit,
            "group": f"{body} / {coordinate}{quantity}"}


def decode_fields(message: Any) -> list[tuple[str, float]]:
    """Decode semantic names instead of collapsing a MAVLink ID to its last value."""
    kind = message.get_type()
    if kind == "NAMED_VALUE_FLOAT" or kind == "NAMED_VALUE_INT":
        return [(normalize_name(text_name(message.name)), message.value)]
    if kind == "DEBUG_VECT":
        name = text_name(message.name)
        prefix = VECTOR_ALIASES.get(name, name)
        return [(f"{prefix}_{axis.upper()}", getattr(message, axis)) for axis in "xyz"]
    if kind == "DEBUG_FLOAT_ARRAY":
        name = text_name(message.name)
        if name == "IMU_AB_C":
            return list(zip(IMU_NAMES, message.data[:24]))
        return [(f"{name or 'ARRAY'}[{message.array_id}].data[{i}]", v)
                for i, v in enumerate(message.data)]
    fields = []
    for name, value in message.to_dict().items():
        if name == "mavpackettype":
            continue
        if isinstance(value, (float, int)):
            fields.append((name, value))
        elif isinstance(value, (list, tuple)):
            fields.extend((f"{name}[{i}]", v) for i, v in enumerate(value[:64])
                          if isinstance(v, (float, int)))
    return fields


class TelemetryStore:
    def __init__(self, *, max_samples: int = 120_000, max_fields: int = 2048,
                 history_seconds: float = 300):
        self.lock = threading.RLock()
        self.max_samples = max_samples
        self.max_fields = max_fields
        self.history_seconds = history_seconds
        self.reset()

    def reset(self) -> None:
        with self.lock:
            self.epoch = uuid.uuid4().hex
            self.seq = 0
            self.events: deque = deque(maxlen=self.max_samples)
            self.fields: dict[str, dict] = {}
            self.messages = 0
            self.bad_packets = 0
            self.dropped_fields = 0
            self.recent: deque = deque(maxlen=10000)
            self.last_packet: float | None = None
            self.targets: set[tuple[int, int]] = set()

    def ingest(self, message: Any, now: float | None = None) -> None:
        now = time.time() if now is None else now
        with self.lock:
            if message.get_type() == "BAD_DATA":
                self.bad_packets += 1
                return
            system, component = message.get_srcSystem(), message.get_srcComponent()
            # Do not present the monitor's own echoed GCS heartbeat as an aircraft.
            if system == 254 and component == 191:
                return
            self.messages += 1
            self.last_packet = now
            self.recent.append(now)
            if message.get_type() == "HEARTBEAT" and getattr(message, "autopilot", 8) != 8:
                self.targets.add((system, component))
            kind = message.get_type()
            for name, value in decode_fields(message):
                if not name:
                    continue
                key = f"{system}/{component}/{kind}/{name}"
                if key not in self.fields:
                    if len(self.fields) >= self.max_fields:
                        self.dropped_fields += 1
                        continue
                    self.fields[key] = {"id": key, "name": name, "system": system,
                                        "component": component, "message": kind,
                                        "count": 0, "first_time": now, **describe(name)}
                field = self.fields[key]
                number = float(value)
                number = number if math.isfinite(number) else None
                previous = field.get("time")
                if previous is not None and now > previous:
                    interval = now - previous
                    field["period"] = .8 * field.get("period", interval) + .2 * interval
                field.update(value=number, time=now, count=field["count"] + 1)
                self.seq += 1
                # [sequence, receive UNIX seconds, field id, scalar or null]
                self.events.append([self.seq, now, key, number])
            while self.events and self.events[0][1] < now - self.history_seconds:
                self.events.popleft()

    def snapshot(self, cursor: int = 0, *, limit: int = 12000) -> dict:
        now = time.time()
        with self.lock:
            while self.recent and self.recent[0] < now - 1:
                self.recent.popleft()
            oldest = self.events[0][0] if self.events else self.seq + 1
            events = []
            for item in self.events:
                if item[0] > cursor:
                    events.append(item)
                    if len(events) >= limit:
                        break
            return {"epoch": self.epoch, "server_time": now,
                    "cursor": events[-1][0] if events else self.seq,
                    "has_more": bool(events and events[-1][0] < self.seq),
                    "truncated": cursor > 0 and cursor < oldest - 1,
                    "samples": events, "fields": list(self.fields.values()),
                    "stats": {"messages": self.messages, "bad_packets": self.bad_packets,
                              "messages_per_second": len(self.recent), "last_packet": self.last_packet,
                              "stored_samples": len(self.events), "dropped_fields": self.dropped_fields,
                              "targets": sorted(self.targets)}}


@dataclass(frozen=True)
class ConnectionConfig:
    transport: str
    endpoint: str = ""
    baud: int = 115200
    heartbeat: bool = True

    @classmethod
    def parse(cls, body: dict) -> "ConnectionConfig":
        transport = body.get("transport", "udp")
        if transport not in ("serial", "udp", "udpout", "tcp", "demo"):
            raise ValueError("连接类型必须为 serial、udp、udpout、tcp 或 demo")
        endpoint = str(body.get("endpoint", "")).strip()
        try:
            baud = int(body.get("baud", 115200))
        except (TypeError, ValueError):
            raise ValueError("波特率必须为整数") from None
        if baud not in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1500000, 2000000):
            raise ValueError("不支持的波特率")
        if transport == "serial":
            if not (endpoint.startswith("/dev/") or re.fullmatch(r"COM\d+", endpoint, re.I)):
                raise ValueError("串口请填写 /dev/ttyACM0、/dev/ttyUSB0 或 COM3")
        elif transport != "demo":
            match = re.fullmatch(r"([a-zA-Z0-9_.-]+):(\d{1,5})", endpoint)
            if not match or not 1 <= int(match[2]) <= 65535:
                raise ValueError("网络地址请填写 主机:端口，例如 0.0.0.0:14550")
        return cls(transport, endpoint, baud, bool(body.get("heartbeat", True)))


class TelemetryService:
    def __init__(self):
        self.store = TelemetryStore()
        self._lifecycle = threading.Lock()
        self._state_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._commands: queue.Queue = queue.Queue(maxsize=8)
        self._connection = None
        self._status = {"state": "disconnected", "error": "", "config": None,
                        "stream_request": ""}

    def status(self) -> dict:
        with self._state_lock:
            return dict(self._status)

    def _set(self, **kwargs) -> None:
        with self._state_lock:
            self._status.update(kwargs)

    def connect(self, body: dict) -> dict:
        config = ConnectionConfig.parse(body)
        with self._lifecycle:
            self._disconnect()
            self.store.reset()
            self._commands = queue.Queue(maxsize=8)
            self._stop = threading.Event()
            self._set(state="connecting", error="", config=config.__dict__, stream_request="")
            self._thread = threading.Thread(target=self._run, args=(config,), daemon=True)
            self._thread.start()
        return self.status()

    def _disconnect(self) -> None:
        self._stop.set()
        connection = self._connection
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise ValueError("上次连接仍在关闭，请稍后重试")
        self._thread = None
        self._connection = None
        self._set(state="disconnected")

    def disconnect(self) -> dict:
        with self._lifecycle:
            self._disconnect()
        return self.status()

    def request_stream(self, body: dict) -> dict:
        hz = float(body.get("hz", 10))
        if not math.isfinite(hz) or not 1 <= hz <= 50:
            raise ValueError("请求频率范围为 1–50 Hz")
        system, component = int(body.get("system", 1)), int(body.get("component", 1))
        if not 1 <= system <= 255 or not 1 <= component <= 255:
            raise ValueError("system/component 范围为 1–255")
        if self.status()["state"] != "connected" or self.status()["config"]["transport"] == "demo":
            raise ValueError("请先连接真实 MAVLink 数据源")
        try:
            self._commands.put_nowait((system, component, hz))
        except queue.Full:
            raise ValueError("请求过多，请稍后重试") from None
        self._set(stream_request=f"已排队：向 {system}/{component} 请求 NAMED_VALUE_FLOAT {hz:g} Hz")
        return self.status()

    def _run(self, config: ConnectionConfig) -> None:
        try:
            if config.transport == "demo":
                self._set(state="connected")
                self._demo()
                return
            import os
            os.environ.setdefault("MAVLINK20", "1")
            from pymavlink import mavutil
            prefixes = {"udp": "udpin:", "udpout": "udpout:", "tcp": "tcp:", "serial": ""}
            endpoint = prefixes[config.transport] + config.endpoint
            if config.transport == "tcp":
                import socket

                class BoundedMavTCP(mavutil.mavtcp):
                    def do_connect(self):
                        self.port = socket.create_connection(self.destination_addr, timeout=3)
                        self.port.setblocking(False)
                        self.port.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                connection = BoundedMavTCP(config.endpoint, source_system=254,
                                           source_component=191, autoreconnect=False, retries=1)
            else:
                connection = mavutil.mavlink_connection(endpoint, baud=config.baud,
                             source_system=254, source_component=191, dialect="common",
                             autoreconnect=False, retries=1)
            self._connection = connection
            self._set(state="connected")
            last_heartbeat = 0.0
            try:
                while not self._stop.is_set():
                    if config.heartbeat and time.monotonic() - last_heartbeat >= 1:
                        connection.mav.heartbeat_send(6, 8, 0, 0, 4)  # GCS, invalid autopilot, active
                        last_heartbeat = time.monotonic()
                    while not self._commands.empty():
                        system, component, hz = self._commands.get_nowait()
                        connection.mav.command_long_send(system, component, 511, 0,
                                                       251, 1_000_000 / hz, 0, 0, 0, 0, 0)
                        self._set(stream_request=f"已发送：{system}/{component} NAMED_VALUE_FLOAT {hz:g} Hz，等待 ACK")
                    message = connection.recv_match(blocking=True, timeout=.1)
                    if message is not None:
                        self.store.ingest(message)
                        if message.get_type() == "COMMAND_ACK" and message.command == 511:
                            self._set(stream_request=f"飞控返回 SET_MESSAGE_INTERVAL ACK，result={message.result}")
            finally:
                connection.close()
                self._connection = None
        except ImportError:
            self._set(state="error", error="缺少依赖：请运行 pip install -r WebUI/requirements-mavlink.txt")
        except Exception as exc:
            if not self._stop.is_set():
                self._set(state="error", error=f"{type(exc).__name__}: {exc}")
        finally:
            if self._stop.is_set():
                self._set(state="disconnected")

    def _demo(self) -> None:
        # Encode and parse real MAVLink 2 bytes, including fractional packet chunks.
        from pymavlink.dialects.v20 import common as mavlink
        encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
        parser = mavlink.MAVLink(None)
        start = time.monotonic()
        while not self._stop.is_set():
            t = time.monotonic() - start
            values = {}
            for name in IMU_NAMES:
                aircraft, sensor, frame, axis = name.split("_")
                phase = "XYZ".index(axis) * 1.2
                base = math.sin(t * 1.5 + phase) * (.2 if sensor == "GYR" else .9)
                x = base + (.025 * math.sin(t * 7) if aircraft == "B" else 0)
                if sensor == "ACC" and axis == "Z":
                    x -= 9.81
                if frame == "C" and axis in "XY":
                    # Coherent XY rotation using the same mounting sign as firmware.
                    raw_x = math.sin(t * 1.5) * (.2 if sensor == "GYR" else .9)
                    raw_y = math.sin(t * 1.5 + 1.2) * (.2 if sensor == "GYR" else .9)
                    noise = .025 * math.sin(t * 7) if aircraft == "B" else 0
                    angle = math.radians(-30 if aircraft == "A" else 30)
                    rotated = ((raw_x + noise) * math.cos(angle) - (raw_y + noise) * math.sin(angle),
                               (raw_x + noise) * math.sin(angle) + (raw_y + noise) * math.cos(angle))
                    x = rotated[axis == "Y"]
                values[name] = x
            connected = int(t / 8) % 2 == 0
            weight_a, weight_b = (.5, .5) if connected else (1., 0.)
            for sensor in ("ACC", "GYR"):
                for axis in "XYZ":
                    values[f"C_{sensor}_{axis}"] = (weight_a * values[f"A_{sensor}_C_{axis}"]
                                                        + weight_b * values[f"B_{sensor}_C_{axis}"])
            values.update(C_ROLL=5 * math.sin(t), C_PITCH=3 * math.cos(t), C_YAW=15 * math.sin(t / 3),
                          C_ALP_X=.1 * math.cos(t), C_ALP_Y=.1 * math.sin(t), C_ALP_Z=0.,
                          CE_RUN=1., CE_EN=1., CE_INIT=1., CE_STATE=1. if connected else 2.,
                          CE_LINK=float(connected), CE_W_A=weight_a, CE_W_B=weight_b, CE_VALID=1.,
                          CE_RESET=1. + int(t / 16), CE_FLAGS=44. if connected else 37.,
                          CE_AGE_A=0., CE_AGE_B=2. if connected else 0., CE_FRAMES=int(t * 500),
                          CE_YAWREF=15 * math.sin(t / 3), CE_YSIG=.1)
            for name, x in values.items():
                msg = encoder.named_value_float_encode(int(t * 1000), name.encode(), x)
                packet = msg.pack(encoder)
                encoder.seq = (encoder.seq + 1) % 256
                for chunk in (packet[:5], packet[5:]):
                    for decoded in parser.parse_buffer(chunk) or []:
                        self.store.ingest(decoded)
            self._stop.wait(.05)
