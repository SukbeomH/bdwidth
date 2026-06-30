import csv
import importlib.util
import json
import sys
import tempfile
import types
from pathlib import Path
import unittest


def load_bdwidth_module():
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "klipper" / "bdwidth.py"
    package = types.ModuleType("klipper")
    package.__path__ = [str(module_path.parent)]
    sys.modules.setdefault("klipper", package)
    sys.modules.setdefault("klipper.bus", types.ModuleType("klipper.bus"))
    fake_serial = types.ModuleType("serial")
    fake_serial.Serial = object
    fake_tools = types.ModuleType("serial.tools")
    fake_list_ports = types.ModuleType("serial.tools.list_ports")
    fake_list_ports.comports = lambda: []
    fake_tools.list_ports = fake_list_ports
    fake_serial.tools = fake_tools
    sys.modules.setdefault("serial", fake_serial)
    sys.modules.setdefault("serial.tools", fake_tools)
    sys.modules.setdefault("serial.tools.list_ports", fake_list_ports)
    fake_sensor = types.ModuleType("klipper.filament_switch_sensor")
    fake_sensor.RunoutHelper = object
    sys.modules.setdefault("klipper.filament_switch_sensor", fake_sensor)
    spec = importlib.util.spec_from_file_location("klipper.bdwidth", module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["klipper.bdwidth"] = module
    spec.loader.exec_module(module)
    return module


def ccd_packet(values):
    packet = bytearray()
    for value in values:
        packet.append(value & 0xff)
        packet.append((value >> 8) & 0xff)
    packet.extend(b"\xff\xff")
    return bytes(packet)


class BDWidthCCDHelperTest(unittest.TestCase):
    def test_decode_ccd_frames_accepts_valid_frame_and_clips_adc_glitches(self):
        bdwidth = load_bdwidth_module()
        values = [2500] * 2547
        values[100] = 5000
        values[1200] = 420

        frames = bdwidth.decode_ccd_frames(ccd_packet(values))

        self.assertEqual(len(frames), 1)
        self.assertEqual(len(frames[0]), 2547)
        self.assertEqual(frames[0][100], 4096)
        self.assertEqual(frames[0][1200], 420)

    def test_decode_ccd_frames_rejects_short_frames(self):
        bdwidth = load_bdwidth_module()

        frames = bdwidth.decode_ccd_frames(ccd_packet([1234] * 100))

        self.assertEqual(frames, [])

    def test_write_ccd_snapshot_files_persists_png_csv_raw_and_metadata(self):
        bdwidth = load_bdwidth_module()
        frame = [2500] * 2547
        frame[1200] = 420
        metadata = {
            "reason": "implausible_width",
            "width": 336.851,
            "raw_width": 64162,
            "motion": -9072,
        }

        with tempfile.TemporaryDirectory() as tmpdir:
            result = bdwidth.write_ccd_snapshot_files(
                tmpdir, "20260701_021500", [frame], b"raw-bytes", metadata)

            for key in ("png", "csv", "raw", "json"):
                self.assertTrue(Path(result[key]).exists(), key)

            with open(result["csv"], newline="") as f:
                rows = list(csv.reader(f))
            self.assertEqual(rows[0], ["index", "amplitude"])
            self.assertEqual(rows[1201], ["1200", "420"])

            with open(result["json"]) as f:
                saved_metadata = json.load(f)
            self.assertEqual(saved_metadata["reason"], "implausible_width")
            self.assertEqual(saved_metadata["samples"], 2547)
            self.assertEqual(saved_metadata["min"], 420)
            self.assertEqual(saved_metadata["max"], 2500)
            self.assertTrue(saved_metadata["png_rendered"])

    def test_write_ccd_snapshot_files_can_defer_png_rendering(self):
        bdwidth = load_bdwidth_module()
        frame = [2500] * 2547
        frame[1200] = 420

        with tempfile.TemporaryDirectory() as tmpdir:
            result = bdwidth.write_ccd_snapshot_files(
                tmpdir, "20260701_021501", [frame], b"raw-bytes",
                {"reason": "outlier_width"}, render_png=False)

            self.assertFalse(Path(result["png"]).exists())
            self.assertTrue(Path(result["csv"]).exists())
            self.assertTrue(Path(result["raw"]).exists())
            self.assertTrue(Path(result["json"]).exists())
            with open(result["json"]) as f:
                saved_metadata = json.load(f)
            self.assertFalse(saved_metadata["png_rendered"])

    def test_render_deferred_ccd_png_creates_png_once_from_json_and_csv(self):
        bdwidth = load_bdwidth_module()
        frame = [2500] * 2547
        frame[1200] = 420

        with tempfile.TemporaryDirectory() as tmpdir:
            result = bdwidth.write_ccd_snapshot_files(
                tmpdir, "20260701_021502", [frame], b"raw-bytes",
                {"reason": "outlier_width"}, render_png=False)

            self.assertTrue(bdwidth.render_deferred_ccd_png(result["json"]))
            self.assertTrue(Path(result["png"]).exists())
            with open(result["json"]) as f:
                saved_metadata = json.load(f)
            self.assertTrue(saved_metadata["png_rendered"])
            self.assertFalse(bdwidth.render_deferred_ccd_png(result["json"]))

    def test_render_deferred_ccd_pngs_respects_limit(self):
        bdwidth = load_bdwidth_module()
        frame = [2500] * 2547

        with tempfile.TemporaryDirectory() as tmpdir:
            first = bdwidth.write_ccd_snapshot_files(
                tmpdir, "20260701_021503", [frame], b"raw-1",
                {"reason": "outlier_width"}, render_png=False)
            second = bdwidth.write_ccd_snapshot_files(
                tmpdir, "20260701_021504", [frame], b"raw-2",
                {"reason": "outlier_width"}, render_png=False)

            rendered = bdwidth.render_deferred_ccd_pngs(
                [first["json"], second["json"]], limit=1)

            self.assertEqual(rendered, [first["json"]])
            self.assertTrue(Path(first["png"]).exists())
            self.assertFalse(Path(second["png"]).exists())

    def test_capture_ccd_snapshot_from_serial_enters_and_exits_stream_mode(self):
        bdwidth = load_bdwidth_module()
        values = [2500] * 2547
        values[1000] = 430
        fake_serial = FakeSerial([ccd_packet(values)])

        result = bdwidth.capture_ccd_snapshot_from_serial(
            fake_serial, frame_count=1, timeout=1.0)

        self.assertEqual(fake_serial.writes[0], b"D01;")
        self.assertEqual(fake_serial.writes[-1], b"G00;")
        self.assertEqual(len(result["frames"]), 1)
        self.assertEqual(result["frames"][0][1000], 430)
        self.assertGreater(len(result["raw"]), 5000)

    def test_capture_ccd_snapshot_from_serial_exits_stream_mode_without_frames(self):
        bdwidth = load_bdwidth_module()
        fake_serial = FakeSerial([b"noise"])

        result = bdwidth.capture_ccd_snapshot_from_serial(
            fake_serial, frame_count=1, timeout=0.01)

        self.assertEqual(fake_serial.writes[0], b"D01;")
        self.assertEqual(fake_serial.writes[-1], b"G00;")
        self.assertEqual(result["frames"], [])
        self.assertEqual(result["raw"], b"noise")

    def test_should_start_ccd_snapshot_requires_enabled_idle_and_rate_limit(self):
        bdwidth = load_bdwidth_module()

        self.assertTrue(bdwidth.should_start_ccd_snapshot(
            enabled=True, in_progress=False, last_capture_time=100.0,
            now=701.0, min_interval=600.0))
        self.assertFalse(bdwidth.should_start_ccd_snapshot(
            enabled=False, in_progress=False, last_capture_time=100.0,
            now=701.0, min_interval=600.0))
        self.assertFalse(bdwidth.should_start_ccd_snapshot(
            enabled=True, in_progress=True, last_capture_time=100.0,
            now=701.0, min_interval=600.0))
        self.assertFalse(bdwidth.should_start_ccd_snapshot(
            enabled=True, in_progress=False, last_capture_time=100.0,
            now=699.0, min_interval=600.0))

    def test_is_ccd_snapshot_outlier_uses_configured_width_thresholds(self):
        bdwidth = load_bdwidth_module()

        self.assertTrue(bdwidth.is_ccd_snapshot_outlier(1.499, 1.5, 2.0))
        self.assertFalse(bdwidth.is_ccd_snapshot_outlier(1.5, 1.5, 2.0))
        self.assertFalse(bdwidth.is_ccd_snapshot_outlier(1.75, 1.5, 2.0))
        self.assertFalse(bdwidth.is_ccd_snapshot_outlier(2.0, 1.5, 2.0))
        self.assertTrue(bdwidth.is_ccd_snapshot_outlier(2.001, 1.5, 2.0))

    def test_read_bdwidth_suppresses_invalid_frame_console_output_by_default(self):
        bdwidth = load_bdwidth_module()
        sensor = make_bdwidth_sensor(
            bdwidth, FakeSerial([b"\x00\x00\x00\x00\x00"]), is_debug=False)

        self.assertFalse(sensor.Read_bdwidth())
        self.assertEqual(sensor.gcode.messages, [])

    def test_read_bdwidth_reports_invalid_frame_when_debug_enabled(self):
        bdwidth = load_bdwidth_module()
        sensor = make_bdwidth_sensor(
            bdwidth, FakeSerial([b"\x04\x0a\x05\x0a\x02"]), is_debug=True)

        self.assertFalse(sensor.Read_bdwidth())
        self.assertEqual(
            sensor.gcode.messages,
            ["4", "10", "5", "10", "2", "fila_width_0: read data error"])


class FakeSerial:
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.writes = []
        self.is_open = True
        self.timeout = None

    def write(self, data):
        self.writes.append(data)

    def read(self, size):
        if not self.chunks:
            return b""
        return self.chunks.pop(0)

    def reset_input_buffer(self):
        pass


class FakeGCode:
    def __init__(self):
        self.messages = []

    def respond_info(self, message):
        self.messages.append(message)


def make_bdwidth_sensor(bdwidth, serial, is_debug=False):
    sensor = object.__new__(bdwidth.BDWidthMotionSensor)
    sensor.port = "usb"
    sensor.usb = serial
    sensor.usb_lock = None
    sensor.gcode = FakeGCode()
    sensor.bd_name = "fila_width_0"
    sensor.is_debug = is_debug
    return sensor


if __name__ == "__main__":
    unittest.main()
