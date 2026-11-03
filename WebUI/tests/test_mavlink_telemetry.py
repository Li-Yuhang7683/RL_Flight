"""Protocol and real UDP/HTTP integration tests for the standalone monitor."""
import json
import math
import socket
import sys
import threading
import time
import unittest
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink.dialects.v20 import common as mavlink
from mavlink_telemetry import (ConnectionConfig, COMP_FIELDS, DEMO_NAMES, IMU_NAMES, TelemetryService,
                               TelemetryStore, decode_fields)
from mavlink_server import MonitorServer


def named(name="A_ACC_R_X", value=1., system=1, component=1, v1=False):
    encoder = mavlink.MAVLink(None, srcSystem=system, srcComponent=component)
    packet = encoder.named_value_float_encode(123, name.encode(), value).pack(encoder, force_mavlink1=v1)
    parser = mavlink.MAVLink(None)
    assert parser.parse_buffer(packet[:7]) is None
    return parser.parse_buffer(packet[7:])[0], packet


def wait_for(predicate, timeout=3):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("condition timed out")


class ProtocolTests(unittest.TestCase):
    def test_both_wire_versions_and_fragmentation(self):
        for v1 in (True, False):
            message, _ = named(v1=v1)
            self.assertEqual(decode_fields(message), [("A_ACC_R_X", 1.)])

    def test_named_channels_are_distinct_and_sources_do_not_collide(self):
        store = TelemetryStore()
        for system in (1, 2):
            for channel in IMU_NAMES:
                store.ingest(named(channel, system=system)[0])
        payload = store.snapshot()
        self.assertEqual(len(payload["fields"]), 48)
        self.assertEqual(len(payload["samples"]), 48)
        self.assertEqual(len(set(field["id"] for field in payload["fields"])), 48)
        self.assertTrue(all(field["unit"] in ("m/s²", "rad/s") for field in payload["fields"]))

    def test_cekf_named_fields_roundtrip_and_units(self):
        store = TelemetryStore()
        for i, name in enumerate(COMP_FIELDS):
            self.assertLessEqual(len(name), 9)
            store.ingest(named(name, value=i)[0])
        fields = {field["name"]: field for field in store.snapshot()["fields"]}
        self.assertEqual(set(fields), set(COMP_FIELDS))
        for i, (name, (_, unit, group)) in enumerate(COMP_FIELDS.items()):
            self.assertEqual(fields[name]["value"], i)
            self.assertEqual(fields[name]["unit"], unit)
            self.assertEqual(fields[name]["group"], group)
        self.assertEqual(fields["C_YAW"]["unit"], "deg")
        self.assertEqual(fields["CE_AGE_B"]["unit"], "ms")

    def test_nulls_remain_invalid_and_json_is_strict(self):
        store = TelemetryStore()
        for value in (math.nan, math.inf, -math.inf):
            store.ingest(named(value=value)[0])
        payload = store.snapshot()
        self.assertTrue(all(sample[3] is None for sample in payload["samples"]))
        json.dumps(payload, allow_nan=False)

    def test_legacy_array_order(self):
        encoder = mavlink.MAVLink(None)
        message = encoder.debug_float_array_encode(1, b"IMU_AB_C", 1, list(range(58)))
        fields = decode_fields(message)
        self.assertEqual(len(fields), 24)
        self.assertEqual(fields[0], ("A_ACC_R_X", 0))
        self.assertEqual(fields[9], ("B_ACC_C_X", 9))
        self.assertEqual(fields[23], ("B_GYR_C_Z", 23))

    def test_legacy_names_and_debug_vectors(self):
        self.assertEqual(decode_fields(named("A_ACCR_X")[0])[0][0], "A_ACC_R_X")
        message = mavlink.MAVLink(None).debug_vect_encode(b"CE_B_GYRC", 1, 1, 2, 3)
        self.assertEqual(decode_fields(message), [("B_GYR_C_X", 1), ("B_GYR_C_Y", 2), ("B_GYR_C_Z", 3)])

    def test_generic_numeric_messages(self):
        message = mavlink.MAVLink(None).attitude_encode(10, .1, .2, .3, .4, .5, .6)
        fields = dict(decode_fields(message))
        self.assertEqual(fields["roll"], .1)
        self.assertEqual(fields["time_boot_ms"], 10)

    def test_bounded_history_and_cursor_paging(self):
        store = TelemetryStore(max_samples=4)
        for i in range(8):
            store.ingest(named(value=i)[0])
        page = store.snapshot(cursor=1, limit=2)
        self.assertTrue(page["truncated"])
        self.assertTrue(page["has_more"])
        self.assertEqual([sample[0] for sample in page["samples"]], [5, 6])
        page = store.snapshot(cursor=page["cursor"])
        self.assertEqual([sample[0] for sample in page["samples"]], [7, 8])
        self.assertFalse(page["has_more"])
        self.assertFalse(store.snapshot(cursor=8)["samples"])

    def test_bad_packet_does_not_become_signal(self):
        store = TelemetryStore()
        store.ingest(mavlink.MAVLink_bad_data(b"x", "bad CRC"))
        payload = store.snapshot()
        self.assertEqual(payload["stats"]["bad_packets"], 1)
        self.assertFalse(payload["fields"])

    def test_field_limit(self):
        store = TelemetryStore(max_fields=1)
        store.ingest(named("A_ACC_R_X")[0])
        store.ingest(named("B_ACC_R_X")[0])
        payload = store.snapshot()
        self.assertEqual(len(payload["fields"]), 1)
        self.assertEqual(payload["stats"]["dropped_fields"], 1)

    def test_config_rejects_file_connections_and_invalid_ports(self):
        for body in ({"transport": "serial", "endpoint": "/etc/passwd"},
                     {"transport": "tcp", "endpoint": "localhost:70000"},
                     {"transport": "udp", "endpoint": "localhost"},
                     {"transport": "file", "endpoint": "a.tlog"}):
            with self.assertRaises(ValueError):
                ConnectionConfig.parse(body)


class TransportTests(unittest.TestCase):
    def test_udp_real_packets_and_corruption_recovery(self):
        service = TelemetryService()
        reservation = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
        reservation.close()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            service.connect({"transport": "udp", "endpoint": f"127.0.0.1:{port}", "heartbeat": False})
            wait_for(lambda: service.status()["state"] == "connected")
            packet1 = named(value=3.5, v1=True)[1]
            packet2 = named("B_GYR_C_Z", value=-.25, system=2)[1]
            bad = bytearray(packet2); bad[-1] ^= 0xff
            sender.sendto(bytes(bad) + packet1 + packet2, ("127.0.0.1", port))
            wait_for(lambda: len(service.store.snapshot()["fields"]) == 2)
            payload = service.store.snapshot()
            self.assertEqual([sample[3] for sample in payload["samples"]], [3.5, -.25])
            self.assertGreaterEqual(payload["stats"]["bad_packets"], 1)
        finally:
            sender.close()
            service.disconnect()
        self.assertEqual(service.status()["state"], "disconnected")

    def test_demo_reconnect_resets_epoch_and_stops_cleanly(self):
        service = TelemetryService()
        try:
            service.connect({"transport": "demo"})
            wait_for(lambda: len(service.store.snapshot()["fields"]) == len(DEMO_NAMES))
            epoch = service.store.epoch
            service.connect({"transport": "demo"})
            self.assertNotEqual(epoch, service.store.epoch)
            wait_for(lambda: len(service.store.snapshot()["fields"]) == len(DEMO_NAMES))
        finally:
            service.disconnect()
        self.assertFalse(service._thread)

    def test_tcp_real_fragmented_stream(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0)); listener.listen()
        port = listener.getsockname()[1]
        packet = named(value=8.)[1]
        def send():
            connection, _ = listener.accept()
            with connection:
                connection.sendall(packet[:3]); time.sleep(.02); connection.sendall(packet[3:])
                time.sleep(.2)
        worker = threading.Thread(target=send, daemon=True); worker.start()
        service = TelemetryService()
        try:
            service.connect({"transport": "tcp", "endpoint": f"127.0.0.1:{port}", "heartbeat": False})
            wait_for(lambda: service.store.snapshot()["stats"]["messages"] >= 1)
            self.assertEqual(service.store.snapshot()["samples"][0][3], 8.)
        finally:
            service.disconnect(); listener.close(); worker.join(timeout=1)

    def test_serial_via_pseudoterminal(self):
        # Exercises pyserial and fragmented framing without a physical FC.
        import os
        import pty
        master, slave = pty.openpty()
        service = TelemetryService()
        try:
            service.connect({"transport": "serial", "endpoint": os.ttyname(slave), "heartbeat": False})
            wait_for(lambda: service.status()["state"] == "connected")
            packet = named(value=7.25)[1]
            os.write(master, packet[:6]); time.sleep(.02); os.write(master, packet[6:])
            wait_for(lambda: service.store.snapshot()["stats"]["messages"] >= 1)
            self.assertEqual(service.store.snapshot()["samples"][0][3], 7.25)
        finally:
            service.disconnect(); os.close(master); os.close(slave)

    def test_stream_request_requires_live_source(self):
        with self.assertRaises(ValueError):
            TelemetryService().request_stream({"hz": 10})


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.server = MonitorServer(("127.0.0.1", 0))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.base = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join()

    def post(self, path, body, origin=None):
        headers = {"Content-Type": "application/json"}
        if origin: headers["Origin"] = origin
        return urlopen(Request(self.base + path, json.dumps(body).encode(), headers=headers), timeout=3)

    def test_static_and_no_filesystem_exposure(self):
        for path in ("/", "/mavlink.js", "/mavlink.css", "/mavlink_plot.mjs"):
            with urlopen(self.base + path) as response:
                self.assertEqual(response.status, 200)
        with self.assertRaises(HTTPError) as error:
            urlopen(self.base + "/mavlink_telemetry.py")
        self.assertEqual(error.exception.code, 404)

    def test_demo_api_and_epoch_reset(self):
        with self.post("/api/connect", {"transport": "demo"}) as response:
            self.assertEqual(response.status, 200)
        wait_for(lambda: len(self.server.telemetry.store.snapshot()["fields"]) == len(DEMO_NAMES))
        # An old browser cursor must not discard the first samples of a new session.
        with urlopen(self.base + "/api/samples?cursor=999999&epoch=old") as response:
            payload = json.load(response)
        self.assertEqual(len(payload["fields"]), len(DEMO_NAMES))
        self.assertGreaterEqual(len(payload["samples"]), len(DEMO_NAMES))
        with self.post("/api/disconnect", {}) as response:
            self.assertEqual(json.load(response)["state"], "disconnected")

    def test_bad_config_and_cross_origin(self):
        with self.assertRaises(HTTPError) as error:
            self.post("/api/connect", {"transport": "udp", "endpoint": "invalid"})
        self.assertEqual(error.exception.code, 400)
        with self.assertRaises(HTTPError) as error:
            self.post("/api/connect", {"transport": "demo"}, origin="https://other.example")
        self.assertEqual(error.exception.code, 403)


if __name__ == "__main__":
    unittest.main()
