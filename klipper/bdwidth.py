import logging
import csv
import json
import math
import struct
import statistics
import threading
import zlib
import serial
import serial.tools.list_ports
import os
import time


from . import bus
from . import filament_switch_sensor


# --- USB Auto-detection helpers ---

# Version strings returned by 'G00;' / 'V;':
#   bdwidth    -> pandapi3dV1  (capital V)
#   bdpressure -> pandapi3dv1  (lowercase v)
# We match on 'pandapi3dV' (capital V, case-sensitive) to identify bdwidth.
BDWIDTH_BAUD = 500000
BDWIDTH_VERSION_MARKER = 'pandapi3dV'   # capital V  = bdwidth

# CH340/CH341 USB-serial chips  vendor ID 1A86 (QinHeng Electronics)
CH340_VID = 0x1A86
CH340_KEYWORDS = ('ch340', 'ch341', '1a86', 'qinheng')
CCD_FRAME_TERMINATOR = b'\xff\xff'
CCD_MIN_SAMPLES = 2540
CCD_MAX_SAMPLES = 2600
CCD_MAX_AMPLITUDE = 4096


def decode_ccd_frames(raw_bytes, min_samples=CCD_MIN_SAMPLES,
                      max_samples=CCD_MAX_SAMPLES,
                      max_amplitude=CCD_MAX_AMPLITUDE):
    buffer = bytearray(raw_bytes)
    frames = []
    while True:
        frame_end = buffer.find(CCD_FRAME_TERMINATOR)
        if frame_end < 0:
            break
        packet = bytes(buffer[:frame_end])
        del buffer[:frame_end + len(CCD_FRAME_TERMINATOR)]
        if len(packet) % 2:
            packet = packet[:-1]
        values = []
        for i in range(0, len(packet), 2):
            value = ((packet[i + 1] << 8) + packet[i]) & 0xffff
            if value > max_amplitude:
                value = max_amplitude
            values.append(value)
        if len(values) > min_samples and len(values) < max_samples:
            frames.append(values)
    return frames


def capture_ccd_snapshot_from_serial(ser, frame_count=3, timeout=2.0,
                                     read_size=4096):
    raw = bytearray()
    frames = []
    try:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        ser.write(b"D01;")
        deadline = time.time() + timeout
        while time.time() < deadline and len(frames) < frame_count:
            chunk = ser.read(read_size)
            if chunk:
                raw.extend(chunk)
                frames = decode_ccd_frames(bytes(raw))
            else:
                time.sleep(0.01)
    finally:
        try:
            ser.write(b"G00;")
        except Exception:
            pass
    if len(frames) > frame_count:
        frames = frames[-frame_count:]
    return {"frames": frames, "raw": bytes(raw)}


def should_start_ccd_snapshot(enabled, in_progress, last_capture_time, now,
                              min_interval):
    if not enabled or in_progress:
        return False
    return now - last_capture_time >= min_interval


def is_ccd_snapshot_outlier(width, min_width, max_width):
    return width < min_width or width > max_width


def write_ccd_snapshot_files(outdir, stamp, frames, raw_bytes, metadata=None,
                             render_png=True):
    if not frames:
        raise ValueError("no CCD frames to save")
    outdir = os.path.abspath(os.path.expanduser(outdir))
    os.makedirs(outdir, exist_ok=True)
    base = os.path.join(outdir, "ccd_%s" % stamp)
    png_path = base + ".png"
    csv_path = base + ".csv"
    raw_path = base + ".raw"
    json_path = base + ".json"
    frame = frames[-1]

    with open(raw_path, "wb") as f:
        f.write(raw_bytes)
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "amplitude"])
        writer.writerows(enumerate(frame))

    snapshot_metadata = dict(metadata or {})
    snapshot_metadata.update({
        "frames": len(frames),
        "samples": len(frame),
        "min": min(frame),
        "max": max(frame),
        "mean": statistics.mean(frame),
        "png": png_path,
        "csv": csv_path,
        "raw": raw_path,
        "png_rendered": bool(render_png),
    })

    if render_png:
        write_ccd_plot_png(png_path, frame)

    snapshot_metadata["json"] = json_path
    with open(json_path, "w") as f:
        json.dump(snapshot_metadata, f, indent=2, sort_keys=True)
    return snapshot_metadata


def render_deferred_ccd_png(json_path):
    json_path = os.path.abspath(os.path.expanduser(json_path))
    with open(json_path) as f:
        metadata = json.load(f)
    if metadata.get("png_rendered"):
        return False
    csv_path = metadata["csv"]
    png_path = metadata["png"]
    frame = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            frame.append(int(row["amplitude"]))
    if not frame:
        return False
    write_ccd_plot_png(png_path, frame)
    metadata["png_rendered"] = True
    metadata["png_rendered_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(json_path, "w") as f:
        json.dump(metadata, f, indent=2, sort_keys=True)
    return True


def render_deferred_ccd_pngs(json_paths, limit=1):
    rendered = []
    for json_path in list(json_paths):
        if len(rendered) >= limit:
            break
        if render_deferred_ccd_png(json_path):
            rendered.append(json_path)
    return rendered


def write_ccd_plot_png(png_path, frame):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 6), dpi=120)
        ax.plot(range(len(frame)), frame, "b-", linewidth=0.8)
        ax.set_title("BDWidth CCD pixel amplitude vs index")
        ax.set_xlabel("index")
        ax.set_ylabel("amplitude")
        ax.set_ylim(0, CCD_MAX_AMPLITUDE)
        ax.grid(True, linewidth=0.3, alpha=0.5)
        fig.tight_layout()
        fig.savefig(png_path)
        plt.close(fig)
        return
    except ImportError:
        pass
    write_simple_ccd_png(png_path, frame)


def write_simple_ccd_png(png_path, frame, width=1400, height=600):
    margin_left = 48
    margin_right = 18
    margin_top = 18
    margin_bottom = 42
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom
    pixels = bytearray([255] * width * height * 3)

    def set_pixel(x, y, color):
        if x < 0 or x >= width or y < 0 or y >= height:
            return
        offset = (y * width + x) * 3
        pixels[offset:offset + 3] = bytes(color)

    def draw_line(x0, y0, x1, y1, color):
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            set_pixel(x0, y0, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    axis_color = (80, 80, 80)
    grid_color = (230, 230, 230)
    line_color = (0, 0, 255)
    for n in range(0, 5):
        y = margin_top + int(plot_h * n / 4.0)
        draw_line(margin_left, y, width - margin_right, y, grid_color)
    for n in range(0, 6):
        x = margin_left + int(plot_w * n / 5.0)
        draw_line(x, margin_top, x, height - margin_bottom, grid_color)
    draw_line(margin_left, margin_top, margin_left, height - margin_bottom,
              axis_color)
    draw_line(margin_left, height - margin_bottom, width - margin_right,
              height - margin_bottom, axis_color)

    if len(frame) > 1:
        last_x = margin_left
        last_y = height - margin_bottom - int(
            max(0, min(CCD_MAX_AMPLITUDE, frame[0]))
            / float(CCD_MAX_AMPLITUDE) * plot_h)
        for i, value in enumerate(frame[1:], start=1):
            x = margin_left + int(i / float(len(frame) - 1) * plot_w)
            y = height - margin_bottom - int(
                max(0, min(CCD_MAX_AMPLITUDE, value))
                / float(CCD_MAX_AMPLITUDE) * plot_h)
            draw_line(last_x, last_y, x, y, line_color)
            last_x, last_y = x, y

    def png_chunk(chunk_type, data):
        return (struct.pack(">I", len(data)) + chunk_type + data
                + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xffffffff))

    raw_rows = bytearray()
    stride = width * 3
    for y in range(height):
        raw_rows.append(0)
        raw_rows.extend(pixels[y * stride:(y + 1) * stride])
    png = bytearray(b"\x89PNG\r\n\x1a\n")
    png.extend(png_chunk(b"IHDR", struct.pack(">IIBBBBB",
                                              width, height, 8, 2, 0, 0, 0)))
    png.extend(png_chunk(b"IDAT", zlib.compress(bytes(raw_rows), 9)))
    png.extend(png_chunk(b"IEND", b""))
    with open(png_path, "wb") as f:
        f.write(png)


def _list_all_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def _list_ch340_ports():
    found = set()

    # --- Methods 1 & 2: pyserial comports() ---
    for p in serial.tools.list_ports.comports():
        vid = getattr(p, 'vid', None)
        desc = (p.description or '').lower()
        hwid = (p.hwid or '').lower()
        if vid == CH340_VID:
            found.add(p.device)
            continue
        if any(k in desc or k in hwid for k in CH340_KEYWORDS):
            found.add(p.device)

    # --- Method 3: /dev/serial/by-id/ symlink scan ---
    by_id_dir = '/dev/serial/by-id'
    if os.path.isdir(by_id_dir):
        for name in os.listdir(by_id_dir):
            if any(k in name.lower() for k in CH340_KEYWORDS):
                symlink = os.path.join(by_id_dir, name)
                try:
                    real = os.path.realpath(symlink)
                    found.add(real)
                    logging.info(
                        "bdwidth auto-detect: found CH340 via by-id: %s -> %s" % (name, real)
                    )
                except Exception:
                    pass

    return list(found)


def _probe_port_for_bdwidth(port):
    ser = None
    try:
        ser = serial.Serial(port, BDWIDTH_BAUD, timeout=0.6)
        time.sleep(0.2)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        ser.write(b'G00;\n')
        line = ser.readline()
        text = line.decode('utf-8', errors='ignore').strip()
        logging.info(
            "bdwidth auto-detect: port=%s baud=%d cmd='G00;' response=%r"
            % (port, BDWIDTH_BAUD, text)
        )
        ser.close()
        # Capital V = bdwidth; lowercase v = bdpressure
        if BDWIDTH_VERSION_MARKER in text:
            return port, text

    except Exception as e:
        logging.warning("bdwidth auto-detect: error probing %s at %d baud: %s"
                        % (port, BDWIDTH_BAUD, e))
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
    return port, None


def auto_detect_bdwidth_port():
    # --- Pass 1: CH340-labelled ports ---
    ch340_ports = _list_ch340_ports()
    logging.info("bdwidth auto-detect: CH340 candidates = %s" % ch340_ports)
    for port in ch340_ports:
        matched_port, resp = _probe_port_for_bdwidth(port)
        if resp is not None:
            logging.info(
                "bdwidth auto-detect: found BDWidth on %s (version: %r)" % (matched_port, resp)
            )
            return matched_port

    # --- Pass 2: all serial ports (fallback) ---
    all_ports = _list_all_serial_ports()
    # skip ports we already tried
    remaining = [p for p in all_ports if p not in ch340_ports]
    logging.info(
        "bdwidth auto-detect: CH340 pass found nothing; "
        "trying remaining ports: %s" % remaining
    )
    for port in remaining:
        matched_port, resp = _probe_port_for_bdwidth(port)
        if resp is not None:
            logging.info(
                "bdwidth auto-detect: found BDWidth on %s (version: %r)" % (matched_port, resp)
            )
            return matched_port

    logging.warning(
        "bdwidth auto-detect: BDWidth not found. "
        "Probed CH340=%s + fallback=%s. "
        "Check klippy.log for per-port responses." % (ch340_ports, remaining)
    )
    return None


BDWIDTH_CHIP_ADDR = 3
BDWIDTH_I2C_SPEED = 100000
BDWIDTH_REGS = {
     '_version' : 0x6,
     '_measure_data' : 0x16

}

class BDWidthMotionSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.port = config.get("port")

        # if config.get("resistance1", None) is None:
        if "i2c" in self.port:  
            self.i2c = bus.MCU_I2C_from_config(config, BDWIDTH_CHIP_ADDR, BDWIDTH_I2C_SPEED)
            self.usb_lock = None
        elif "usb" in self.port:
            baudrate = 500000
            configured_serial = config.get("serial", None)
            if configured_serial is None or configured_serial.strip().lower() == "auto":
                # Auto-detect: scan CH340 ports and identify BDWidth by baud rate (500000)
                usb_port = auto_detect_bdwidth_port()
                if usb_port is None:
                    raise config.error(
                        "BDWidthMotionSensor: could not auto-detect USB port. "
                        "No CH340 device responded at 500000 baud. "
                        "Set 'serial' explicitly in the config if auto-detection fails."
                    )
                logging.info("BDWidthMotionSensor: using auto-detected port %s" % usb_port)
            else:
                usb_port = configured_serial
            self.usb_port = usb_port
            self.usb = serial.Serial(self.usb_port, baudrate, timeout=1)
            self.usb_lock = threading.Lock()
        self.gcode = self.printer.lookup_object('gcode')
        self.extruder_name = config.get('extruder')
        self.check_on_print_start = config.getboolean(
            "check_on_print_start", False)
        try: 
            self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        except Exception as e:
            self.runout_helper = filament_switch_sensor.RunoutHelper(config,self)
        self.get_status = self.runout_helper.get_status
        self.extruder = None
        self.estimated_print_time = None
        # Initialise internal state
        self.filament_runout_pos = 0.0
        self.filament_present = True
        
        self.nominal_filament_dia = config.getfloat(
            'default_nominal_filament_diameter', above=1.0)
        self.sensor_to_nozzle_length = config.getfloat('sensor_to_nozzle_length', above=0.)
   
        self.runout_delay_length = config.getfloat('runout_delay_length', 7., above=0.)
        self.tolerance_count = config.getfloat('tolerance_count', 2, above=1)

        self.flowrate_adjust_length = config.getfloat('flowrate_adjust_length', 5., above=1.)

        self.is_active =config.get('enable')    
        self.min_diameter=config.getfloat('min_diameter', 1.0)
        self.linear_motion=config.getfloat('motion_linear_coefficient', 42.8)
        self.max_diameter=config.getfloat('max_diameter', 1.9)
        self.min_plausible_diameter = config.getfloat(
            'min_plausible_diameter', 1.5, above=0.)
        self.max_plausible_diameter = config.getfloat(
            'max_plausible_diameter', 2.0,
            above=self.min_plausible_diameter)
        self.ccd_snapshot_on_outlier = config.getboolean(
            'ccd_snapshot_on_outlier', False)
        self.ccd_snapshot_min_diameter = config.getfloat(
            'ccd_snapshot_min_diameter', 1.5)
        self.ccd_snapshot_max_diameter = config.getfloat(
            'ccd_snapshot_max_diameter', 2.0,
            above=self.ccd_snapshot_min_diameter)
        self.ccd_snapshot_dir = config.get(
            'ccd_snapshot_dir', self.get_log_path()+"bdwidth_ccd_auto")
        self.ccd_snapshot_min_interval = config.getfloat(
            'ccd_snapshot_min_interval', 600., minval=0.)
        self.ccd_snapshot_frames = int(config.getfloat(
            'ccd_snapshot_frames', 3., above=0.))
        self.ccd_snapshot_timeout = config.getfloat(
            'ccd_snapshot_timeout', 2., above=0.)
        self.ccd_snapshot_defer_png_during_print = config.getboolean(
            'ccd_snapshot_defer_png_during_print', True)
        self.ccd_snapshot_png_limit_per_print = config.getint(
            'ccd_snapshot_png_limit_per_print', 1, minval=0)
        self.ccd_snapshot_in_progress = False
        self.ccd_snapshot_last_capture_time = 0.
        self.ccd_snapshot_thread = None
        self.ccd_snapshot_is_printing = False
        self.ccd_snapshot_pending_pngs = []
        self.ccd_snapshot_pending_lock = threading.Lock()
        self.ccd_snapshot_render_in_progress = False
        self.sample_time=config.getfloat('sample_time', 1.0) # in second
        self.is_log =config.getboolean('logging', False)
        self.is_debug =config.getboolean('debug_info', False)
        self.raw_width = 0
        self.lastFilamentWidthReading = 0
        self.lastMotionReading = 0
        self.actual_total_move = 0
        self.filament_array = []
        self.width_out_count = 0
        self.runout_count = 0
        # Register commands
        self.bd_name = config.get_name().split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_BDWIDTH", "NAME", self.bd_name,
                                   self.cmd_SET_BDWIDTH,
                                   desc=self.cmd_SET_BDWIDTH_help)
        
        if self.is_log == True:
        
           # logging.basicConfig(handlers=[logging.FileHandler(filename=self.get_log_path()+"bdwidth.log", 
            #                                     encoding='utf-8', mode='a+')],
           #         format="%(asctime)s  %(message)s", 
           #         datefmt="%F %A %T", 
           #         level=print)
            self.logerb=self.get_logger(self.get_log_path()+"bdwidth_"+self.bd_name+".log.csv")
                    

        # Register commands and event handlers
        self.printer.register_event_handler('klippy:ready',
                                            self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._shutdown)
        
        self.printer.register_event_handler('idle_timeout:printing',
                                            self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready',
                                            self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle',
                                            self._handle_not_printing)

        self.filament_array = []                                     
        self.extrude_factor_update_timer = self.reactor.register_timer(
            self.extrude_factor_update_event)
            
    #def handle_connect(self):
        #self.reactor.update_timer(self.sample_timer, self.reactor.NOW)
        self.extruder_pos_old = 0
        self.angel_to_len_old = 0
        #self.gcode.register_command('QUERY_FILAMENT_WIDTH', self.cmd_M407)
       # self.gcode.register_command('RESET_FILAMENT_WIDTH_SENSOR',
       #                                 self.cmd_ClearFilamentArray)
       # self.gcode.register_command('DISABLE_FILAMENT_WIDTH_SENSOR',
       #                                 self.cmd_M406)
       # self.gcode.register_command('ENABLE_FILAMENT_WIDTH_SENSOR',
       #                                 self.cmd_M405)
      #  self.gcode.register_command('ENABLE_FILAMENT_WIDTH_INFO',
      #                              self.cmd_info_enable)
      #  self.gcode.register_command('DISABLE_FILAMENT_WIDTH_INFO',
      #                              self.cmd_info_disable)

                                    

    
    cmd_SET_BDWIDTH_help = "cmd for bdwidth sensor,SET_BDWIDTH NAME=xxx COMMAND=ENABLE/DISABLE/QUERY"
    def cmd_SET_BDWIDTH(self, gcmd):
        # Read requested value
        cmd = gcmd.get('COMMAND')
        self.gcode.respond_info("Send %s to bdsensor:%s"%(cmd,self.bd_name))
        if 'ENABLE' in cmd:
            self.cmd_enable(gcmd)
        elif 'DISABLE' in cmd:  
            self.cmd_disable(gcmd)
        elif 'QUERY' in cmd:  
            self.cmd_query(gcmd)  
    def get_logger(self,name):
        logger = logging.getLogger(self.bd_name)
        log_path = os.path.abspath(os.path.expanduser(name))
        formatter = logging.Formatter('%(asctime)s,%(message)s',"%m/%d %H:%M:%S")
        logger.setLevel(logging.DEBUG)
        logger.propagate = False
        for handler in logger.handlers:
            if (isinstance(handler, logging.FileHandler)
                and os.path.abspath(handler.baseFilename) == log_path):
                return logger
        fh = logging.FileHandler(log_path, mode='a+', encoding='utf-8')
        fh.setFormatter(formatter)
        logger.addHandler(fh)
        return logger


    def get_log_path(self):
        #result=subprocess.run(["ps", "-ef"], check=True, text=True, capture_output=True)
        os.system("ps -ef > /tmp/logd")
        with open("/tmp/logd", "r") as f:
            result = f.read()
            result=str(result).split(" ")
            for c_path in result:
                if '/klippy.log' in c_path:
               # folders['logs'] =  os.path.dirname(c_path) + '/'     
                    return os.path.dirname(c_path) + '/'
        return '/tmp/'

    def log_file(self,mes_str):
        if self.is_log == True:
            self.logerb.info(mes_str)

    def _queue_ccd_snapshot(self, reason, filament_width, raw_width, motion):
        if "usb" != self.port:
            return
        now = time.time()
        if not should_start_ccd_snapshot(
            self.ccd_snapshot_on_outlier,
            self.ccd_snapshot_in_progress,
            self.ccd_snapshot_last_capture_time,
            now,
            self.ccd_snapshot_min_interval):
            return
        self.ccd_snapshot_in_progress = True
        self.ccd_snapshot_last_capture_time = now
        metadata = {
            "reason": reason,
            "bd_name": self.bd_name,
            "width": filament_width,
            "raw_width": raw_width,
            "motion": motion,
            "trigger_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        try:
            self.ccd_snapshot_thread = threading.Thread(
                target=self._capture_ccd_snapshot_worker,
                args=(metadata,),
                name="bdwidth-ccd-snapshot",
                daemon=True)
            self.ccd_snapshot_thread.start()
            logging.info("%s: queued CCD snapshot for %s width=%.3f raw=%d"
                         % (self.bd_name, reason, filament_width, raw_width))
        except Exception:
            self.ccd_snapshot_in_progress = False
            logging.exception("%s: failed to queue CCD snapshot" % self.bd_name)

    def _capture_ccd_snapshot_worker(self, metadata):
        try:
            if self.usb_lock is None:
                return
            with self.usb_lock:
                old_timeout = getattr(self.usb, "timeout", None)
                try:
                    self.usb.timeout = 0.05
                    result = capture_ccd_snapshot_from_serial(
                        self.usb,
                        frame_count=self.ccd_snapshot_frames,
                        timeout=self.ccd_snapshot_timeout)
                finally:
                    try:
                        self.usb.timeout = old_timeout
                    except Exception:
                        pass
            stamp = time.strftime("%Y%m%d_%H%M%S")
            if not result["frames"]:
                logging.warning(
                    "%s: CCD snapshot captured no valid frames raw_bytes=%d"
                    % (self.bd_name, len(result["raw"])))
                return
            render_png = True
            if (self.ccd_snapshot_defer_png_during_print
                and self.ccd_snapshot_is_printing):
                render_png = False
            saved = write_ccd_snapshot_files(
                self.ccd_snapshot_dir, stamp, result["frames"], result["raw"],
                metadata, render_png=render_png)
            if render_png:
                logging.info("%s: saved CCD snapshot png=%s csv=%s json=%s"
                             % (self.bd_name, saved["png"], saved["csv"],
                                saved["json"]))
            else:
                with self.ccd_snapshot_pending_lock:
                    self.ccd_snapshot_pending_pngs.append(saved["json"])
                logging.info(
                    "%s: saved CCD snapshot data for deferred png csv=%s json=%s"
                    % (self.bd_name, saved["csv"], saved["json"]))
        except Exception:
            logging.exception("%s: CCD snapshot failed" % self.bd_name)
        finally:
            self.ccd_snapshot_in_progress = False

    def _queue_deferred_ccd_png_render(self):
        if self.ccd_snapshot_render_in_progress:
            return
        with self.ccd_snapshot_pending_lock:
            if not self.ccd_snapshot_pending_pngs:
                return
            pending = self.ccd_snapshot_pending_pngs[
                :self.ccd_snapshot_png_limit_per_print]
            self.ccd_snapshot_pending_pngs = []
        if not pending:
            return
        self.ccd_snapshot_render_in_progress = True
        thread = threading.Thread(
            target=self._render_deferred_ccd_png_worker,
            args=(pending,),
            name="bdwidth-ccd-png-render",
            daemon=True)
        thread.start()

    def _render_deferred_ccd_png_worker(self, json_paths):
        try:
            rendered = render_deferred_ccd_pngs(
                json_paths, limit=self.ccd_snapshot_png_limit_per_print)
            if rendered:
                logging.info("%s: rendered deferred CCD png from %s"
                             % (self.bd_name, rendered[0]))
        except Exception:
            logging.exception("%s: deferred CCD png render failed"
                              % self.bd_name)
        finally:
            self.ccd_snapshot_render_in_progress = False

    
    def update_filament_array(self, last_epos):
        # Fill array
        if len(self.filament_array) > 0:
            # Get last reading position in array & calculate next
            # reading position
            next_reading_position = (self.filament_array[-1][0]
                                     + self.flowrate_adjust_length)
            if next_reading_position <= (last_epos + self.sensor_to_nozzle_length):
                self.filament_array.append([last_epos + self.sensor_to_nozzle_length,
                                            self.lastFilamentWidthReading])
                if self.is_debug == True:
                    self.gcode.respond_info("%s , Width:%.3f" % (self.bd_name,self.lastFilamentWidthReading))                             
        else:
            # add first item to array
            self.filament_array.append([self.sensor_to_nozzle_length + last_epos,
                                        self.lastFilamentWidthReading])
            #if self.is_debug == True:
             #   self.gcode.respond_info("add first item to array.lastFilamentWidthReading:%.3f" % (self.lastFilamentWidthReading))                             

    def Read_bdwidth(self):
        self.bdw_data = ''
         
        buffer = bytearray()
        if "usb" == self.port:
            locked = False
            if self.usb_lock is not None:
                locked = self.usb_lock.acquire(False)
                if not locked:
                    return False
            try:
                if self.usb.is_open:
                    try:
                        self.usb.reset_input_buffer()
                    except Exception:
                        pass
                    self.usb.write('\n'.encode())
                    self.usb.timeout = 0.01
                    data = self.usb.read(5)
                    if data:
                        for byte in data:
                            buffer.append(byte)
            finally:
                if locked:
                    self.usb_lock.release()
        elif "i2c" == self.port: 
            buffer = self.read_register('_measure_data', 5)
        if len(buffer) == 5 and buffer[4] == 0x0a:
            raw_width = ((buffer[1] << 8) + buffer[0])&0xffff
            lastMotionReading = ((buffer[3] << 8) + buffer[2])&0xffff
            if lastMotionReading>32767 :
                lastMotionReading = lastMotionReading - 65536
            lastMotionReading = -lastMotionReading # change the default dir
            filament_width = raw_width*0.00525
            if is_ccd_snapshot_outlier(
                filament_width, self.ccd_snapshot_min_diameter,
                self.ccd_snapshot_max_diameter):
                reason = "implausible_width"
                if (filament_width >= self.min_plausible_diameter
                    and filament_width <= self.max_plausible_diameter):
                    reason = "outlier_width"
                self._queue_ccd_snapshot(reason, filament_width, raw_width,
                                         lastMotionReading)
            if (filament_width < self.min_plausible_diameter
                or filament_width > self.max_plausible_diameter):
                self.gcode.respond_info(
                    "%s: ignored implausible width frame: %.3fmm raw:%d motion:%d"
                    % (self.bd_name, filament_width, raw_width,
                       lastMotionReading))
                return False
            
            self.raw_width = raw_width
            self.lastMotionReading = lastMotionReading
            self.lastFilamentWidthReading = filament_width
            self.actual_total_move = self.actual_total_move + self.lastMotionReading
            if self.lastMotionReading !=0:
                if self.is_debug == True:
                    self.gcode.respond_info(str(round(self.lastFilamentWidthReading,3))+'mm,'+str(round(self.actual_total_move/self.linear_motion,1))+'mm,'+str(self.actual_total_move))
                self.log_file(str(round(self.lastFilamentWidthReading,3))+','+str(round(self.actual_total_move/self.linear_motion,1))+'mm,'+str(self.actual_total_move))
        else:
            for i in buffer:
                self.gcode.respond_info("%d"%i)
            self.gcode.respond_info("%s: read data error"%self.bd_name)
            return False
        #if self.is_debug == True:
        #    self.gcode.respond_info("bdwidth, port:%s, width:%.3f mm (%d),motion:%d" % (self.port,self.lastFilamentWidthReading,
         #                                        self.raw_width,self.lastMotionReading))          
        return True


    def width_process(self,eventtime,last_epos):
    # width process Update extrude factor      
        # Check runout
        try:
            self.runout_helper.note_filament_present(eventtime, True)
        except Exception as e:
            self.runout_helper.note_filament_present(True)
            pass
        
        # Does filament exists
       # if self.is_debug == True:
        #    self.gcode.respond_info(" width:%.4fmm, pending_position:%f,last_epos:%f" % (self.lastFilamentWidthReading,self.filament_array[0][0],last_epos))
        if self.lastFilamentWidthReading >= self.min_diameter and self.lastFilamentWidthReading <= self.max_diameter:
            self.filament_present = True
            self.width_out_count = 0
            try:
                self.runout_helper.note_filament_present(eventtime, True)
            except Exception as e:
                self.runout_helper.note_filament_present(True)
                pass
            if len(self.filament_array) > 0:
                # Get first position in filament array
                pending_position = self.filament_array[0][0]
                if pending_position <= last_epos:
                    # Get first item in filament_array queue
                    item = self.filament_array.pop(0)
                    filament_width = item[1]
                    if ((filament_width <= self.max_diameter)
                        and (filament_width >= self.min_diameter)):
                        percentage = round(self.nominal_filament_dia**2
                                           / filament_width**2 * 100,2)
                        self.gcode.run_script("M221 S" + str(percentage))
                        if self.is_debug == True:
                            self.gcode.respond_info("M221 S:%.3f ; %s, width:%.3f" %  (percentage,self.bd_name,filament_width))
                    else:
                        self.gcode.run_script("M221 S100")
        else:
            self.width_out_count=self.width_out_count+1
            if self.width_out_count < self.tolerance_count:
                return
            if self.filament_present == True:
                self.gcode.respond_info("%s:filament width is out of range: %0.3fmm [%0.3f,%0.3f]!!! pause"%(self.bd_name,self.lastFilamentWidthReading,
                                                                       self.min_diameter,self.max_diameter))
                self.filament_present = False                                                       
            #self.runout_helper.note_filament_present(eventtime, False)
            try:
                self.runout_helper.note_filament_present(eventtime, False)
            except Exception as e:
                self.runout_helper.note_filament_present(False)
                pass
            
            self.gcode.run_script("M221 S100")
            self.filament_array = []

    def motion_process(self,eventtime):
         # motion process
        if self.lastMotionReading!=0:
         #   self.gcode.respond_info("port:%s, width:%.3f mm (%d),motion:%d" % (self.port,self.lastFilamentWidthReading,
         #                                    self.raw_width,self.lastMotionReading))
            self._update_filament_runout_pos(eventtime)
            self.runout_count = 0
        else:
            
            extruder_pos = self._get_extruder_pos(eventtime)
            #self.gcode.respond_info("epos:%0.1f filament_runout_pos:%0.1f,actual_total_move:%d" % (extruder_pos, 
            #                            self.filament_runout_pos,self.actual_total_move))
            # Check for filament runout
            if extruder_pos > (self.filament_runout_pos-5):
                self.runout_count = self.runout_count+1
                if self.runout_count < self.tolerance_count:
                    return
                self.gcode.respond_info("Rounout: because extruder_postion:%0.1f > filament_runout_pos:%0.1f, (actual_total_move:%d)" % (extruder_pos, 
                                            self.filament_runout_pos,self.actual_total_move))
                self.gcode.respond_info("If the trigger is incorrect, you can increase the runout_delay_length or check the flow rate in the gcode file")
               # self.runout_helper.note_filament_present(eventtime, False)
                try:
                    self.runout_helper.note_filament_present(eventtime, False)
                except Exception as e:
                    self.runout_helper.note_filament_present(False)
                    pass
                self._update_filament_runout_pos(eventtime) 
            
          #  self._update_filament_runout_pos(eventtime)  

    
    def extrude_factor_update_event(self, eventtime):
        if 'disable' in self.is_active:     
            return eventtime + self.sample_time
            
        if self.Read_bdwidth() == True:
            last_epos = self.toolhead.get_position()[3]
            # Update filament array for lastFilamentWidthReading
            self.update_filament_array(last_epos)
            if 'width' in self.is_active or 'all' in self.is_active:
                self.width_process(eventtime,last_epos)
            if 'motion' in self.is_active or 'all' in self.is_active:    
                self.motion_process(eventtime) 
           
        else:
            return eventtime + 10

        return eventtime + self.sample_time



    def read_register(self, reg_name, read_len):
        # read a single register
        regs = [BDWIDTH_REGS[reg_name]]
        params = self.i2c.i2c_read(regs, read_len)
        return bytearray(params['response'])

    def write_register(self, reg_name, data):
        if type(data) is not list:
            data = [data]
        reg = BDWIDTH_REGS[reg_name]
        data.insert(0, reg)
        self.i2c.i2c_write(data)
        


    def compare_float(self, a, b, precision):
        if abs(a - b) <= precision:
            return True
        return False

    def _update_filament_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.filament_runout_pos = (
                self._get_extruder_pos(eventtime) +
                self.runout_delay_length)
    def _handle_ready(self):
        
        self.toolhead = self.printer.lookup_object('toolhead')
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = (
                self.printer.lookup_object('mcu').estimated_print_time)
        self._update_filament_runout_pos()
        
        #self.reactor.update_timer(self.extrude_factor_update_timer,  # width sensor
        #                          self.reactor.NOW)        

    def _shutdown(self):
        self.reactor.update_timer(self.extrude_factor_update_timer,  
                                  self.reactor.NEVER)      
    def _handle_printing(self, print_time):
        self.ccd_snapshot_is_printing = True

    def _handle_not_printing(self, print_time):
        self.ccd_snapshot_is_printing = False
        self._queue_deferred_ccd_png_render()

        return

    def _get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)

    
            
    def cmd_query(self, gcmd):
        # response = ""
        # if "usb" == self.port:
        #     self.usb.write('G00;'.encode())
        #     response += self.usb.readline().decode('ascii').strip()
        # elif "i2c" == self.port: 
        #     response += self.read_register('_version', 15).decode('utf-8')
        eventtime = self.reactor.monotonic()
        self.extrude_factor_update_event(eventtime)   
        
      #  if self.lastFilamentWidthReading > 0:
      #      response += (" Filament dia (measured mm): "
      #                   + str(self.lastFilamentWidthReading)
      #                   +" Motion:" + str(self.lastMotionReading))
       # else:
       #     response += " Filament NOT present,"
        gcmd.respond_info("active:"+ self.is_active+", motion:"+str(self.lastMotionReading)+",width:"+str(self.lastFilamentWidthReading) )

    def cmd_ClearFilamentArray(self, gcmd):
        self.filament_array = []
        gcmd.respond_info("Filament width measurements cleared!")
        # Set extrude multiplier to 100%
        self.gcode.run_script_from_command("M221 S100")

    def cmd_enable(self, gcmd):
       # cmd_bd = gcmd.get('enable', None)
      #  if cmd_bd is not None:
      #      self.is_active = cmd_bd
        cmd = gcmd.get('COMMAND')
        gcmd.respond_info("cmd:"+cmd)
        self.is_active = 'all'
        if 'MOTION' in cmd:
            self.is_active = 'motion'
        elif 'WIDTH' in cmd:
            self.is_active = 'width'    
        response = "bdwidth sensor status:" + self.is_active
        self.reactor.update_timer(self.extrude_factor_update_timer,  # width sensor
                                  self.reactor.NOW)   
        gcmd.respond_info(response)


    def cmd_disable(self, gcmd):
        #response = "Filament width sensor Turned Off"
        self.is_active = 'disable'
        # Stop extrude factor update timer
        self.reactor.update_timer(self.extrude_factor_update_timer,
                                  self.reactor.NEVER)
        # Clear filament array
        self.filament_array = []
        # Set extrude multiplier to 100%
        self.gcode.run_script_from_command("M221 S100")
        gcmd.respond_info("Filament width sensor:%s Turned Off"%self.bd_name)
        
    def sensor_get_status(self, eventtime):
        return {
            "runout_distance": float(self.runout_helper.runout_distance),
            "runout_elapsed": float(self.runout_helper.runout_elapsed),
            "check_on_print_start": bool(self.check_on_print_start),
        }      
        
    def get_status(self, eventtime):
        return {'Diameter': self.self.lastFilamentWidthReading,
                'Raw':self.raw_width,
                'Motion':self.lastMotionReading,
                'active':self.is_active}
                
    def cmd_info_enable(self, gcmd):
        self.is_debug = True
        gcmd.respond_info("Filament width debug inforamtion Turned On")

    def cmd_info_disable(self, gcmd):
        self.is_debug = False
        gcmd.respond_info("Filament width debug inforamtion Turned Off")
    def cmd_bdwidth_screen_off(self, gcmd):
        buffer = bytearray()
        if "usb" == self.port:
            if self.usb.is_open:
                self.usb.write('\n'.encode())
                self.usb.timeout = 0.01
                data = self.usb.read(5)
                if data:
                    for byte in data:
                        buffer.append(byte)
        elif "i2c" == self.port: 
            buffer = self.read_register('_measure_data', 5)

    def cmd_bdwidth_screen_on(self, gcmd):
        response = ""
        if "usb" == self.port:
            self.usb.write('G00;'.encode())
            response += self.usb.readline().decode('ascii').strip()
        elif "i2c" == self.port: 
            response += self.read_register('_version', 15).decode('utf-8')

def load_config_prefix(config):
    return BDWidthMotionSensor(config)
