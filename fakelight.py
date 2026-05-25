import math
import sys
import time
import threading
import subprocess
import sqlite3
import re
import serial
from pyftdi.ftdi import Ftdi


# ---------------------------------------------------------------------------
# Radar sensor (RD03D)
# ---------------------------------------------------------------------------

class Target:
    def __init__(self, x, y, speed, pixel_distance):
        self.x = x                  # mm
        self.y = y                  # mm
        self.speed = speed          # cm/s
        self.pixel_distance = pixel_distance  # mm
        self.distance = math.sqrt(x**2 + y**2)
        self.angle = math.degrees(math.atan2(x, y))

    def __str__(self):
        return ('Target(x={}mm, y={}mm, speed={}cm/s, pixel_dist={}mm, '
                'distance={:.1f}mm, angle={:.1f}°)').format(
                self.x, self.y, self.speed, self.pixel_distance,
                self.distance, self.angle)


class RD03D:
    SINGLE_TARGET_CMD = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01])
    MULTI_TARGET_CMD  = bytes([0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01])

    def __init__(self, uart_port='/dev/ttyS0', baudrate=256000, multi_mode=True):
        self.uart = serial.Serial(uart_port, baudrate, timeout=0.1)
        self.targets = []
        self.buffer = b''
        time.sleep(0.2)
        self.set_multi_mode(multi_mode)

    def set_multi_mode(self, multi_mode=True):
        cmd = self.MULTI_TARGET_CMD if multi_mode else self.SINGLE_TARGET_CMD
        self.uart.write(cmd)
        self.uart.flush()
        time.sleep(0.2)
        self.uart.reset_input_buffer()
        self.buffer = b''
        self.multi_mode = multi_mode

    @staticmethod
    def parse_signed16(high, low):
        raw = (high << 8) + low
        sign = 1 if (raw & 0x8000) else -1
        value = raw & 0x7FFF
        return sign * value

    def _decode_frame(self, data):
        targets = []
        if len(data) < 30 or data[0] != 0xAA or data[1] != 0xFF or data[-2] != 0x55 or data[-1] != 0xCC:
            return targets
        for i in range(3):
            base = 4 + i * 8
            x = self.parse_signed16(data[base+1], data[base])
            y = self.parse_signed16(data[base+3], data[base+2])
            speed = self.parse_signed16(data[base+5], data[base+4])
            pixel_dist = data[base+6] + (data[base+7] << 8)
            targets.append(Target(x, y, speed, pixel_dist))
        return targets

    def _find_complete_frame(self, data):
        start_idx = -1
        for i in range(len(data) - 1):
            if data[i] == 0xAA and data[i+1] == 0xFF:
                start_idx = i
                break
        if start_idx == -1:
            return None, data
        for i in range(start_idx + 2, len(data) - 1):
            if data[i] == 0x55 and data[i+1] == 0xCC:
                frame = data[start_idx:i+2]
                remaining = data[i+2:]
                return frame, remaining
        return None, data[start_idx:]

    def update(self):
        if self.uart.in_waiting > 0:
            self.buffer += self.uart.read(self.uart.in_waiting)
        if len(self.buffer) > 300:
            self.buffer = self.buffer[-150:]
        latest_frame = None
        temp_buffer = self.buffer
        while True:
            frame, temp_buffer = self._find_complete_frame(temp_buffer)
            if frame:
                latest_frame = frame
            else:
                break
        if latest_frame:
            frame_end_pos = self.buffer.rfind(latest_frame) + len(latest_frame)
            self.buffer = self.buffer[frame_end_pos:]
            decoded = self._decode_frame(latest_frame)
            if decoded:
                self.targets = decoded
                return True
        return False

    def get_target(self, target_number=1):
        if 1 <= target_number <= len(self.targets):
            return self.targets[target_number - 1]
        return None

    def close(self):
        if self.uart.is_open:
            self.uart.close()

# --- Install-time calibration ---
PAN_ZERO       = 128   # DMX value pointing "forward"
TILT_ZERO      = 64    # DMX value that's level for this mount
PAN_RANGE_DEG  = 540   # physical pan range of fixture (full 540°)
TILT_RANGE_DEG = 220   # physical tilt range of fixture (measure to confirm)

RADAR_HEIGHT_M = 4.0   # height of radar above ground (measure at install)
HUMAN_HEIGHT_M = 1.7   # aim for center of mass
LIGHT_OFFSET_M = 2.0   # light is this far below the radar

# --- Timing ---
SAD_LINGER_S      = 5.0   # how long to stay sad after humans leave
ATTRACT_STEP_S    = 0.05  # attract mode update rate
PRESENCE_TIMEOUT_S = 0.5  # seconds without a detection before declaring no human
TRACK_SPEED       = 20    # XY speed in tracking mode (0=max fast, 255=slowest)
EMA_ALPHA         = 0.4   # radar EMA smoothing (1.0=raw/no smoothing, lower=smoother)
RADAR_MIN_M       = 0.5   # ignore targets closer than this (structure reflections)
RADAR_MAX_M       = 8.0   # ignore targets farther than this

# --- Channel map (0-indexed) ---
# 0  pan               0-255
# 1  pan fine          0-255
# 2  tilt              0-255
# 3  tilt fine         0-255
# 4  xy speed          0-255 (0=fast, 255=slow)
# 5  dimmer            0-255 (0=dark, 255=bright)
# 6  strobe            0-3=open, 4-99=sync, 100-149=pulse, 150-199=random, 200-249=consecutive
# 7  color wheel       see COLOR_* constants below
# 8  gobo solid        0=open, 3-66=patterns, 67+=jitter/shake
# 9  gobo colorful     0-127=static, 128+=flowing effects
# 10 prism             0-127=cut in/out, 128-255=rotation
# 11 prism rotation    0-255
# 12 focus             0-255
# 13 reset             200-205=reset all

# --- Color wheel positions (ch8, DMX channel 8) ---
COLOR_WHITE  = 0    # 0-6
COLOR_RED    = 9    # 7-11
COLOR_GREEN  = 14   # 12-16
COLOR_BLUE   = 19   # 17-21
COLOR_YELLOW = 24   # 22-26
COLOR_SPIN   = 200  # continuous rainbow spin (high values)


def clamp(v):
    return max(0, min(255, v))


def make_frame(pan=128, pan_fine=0, tilt=64, tilt_fine=0, speed=0,
               dimmer=255, strobe=0, color=COLOR_WHITE,
               gobo=0, colorful=0, prism=0, prism_rot=0, focus=128):
    frame = [0] * 14
    frame[0]  = clamp(pan)
    frame[1]  = clamp(pan_fine)
    frame[2]  = clamp(tilt)
    frame[3]  = clamp(tilt_fine)
    frame[4]  = clamp(speed)
    frame[5]  = clamp(dimmer)
    frame[6]  = clamp(strobe)
    frame[7]  = clamp(color)
    frame[8]  = clamp(gobo)
    frame[9]  = clamp(colorful)
    frame[10] = clamp(prism)
    frame[11] = clamp(prism_rot)
    frame[12] = clamp(focus)
    frame[13] = 0   # reset — never set this casually
    return frame


# ---------------------------------------------------------------------------
# Shared application state  (live data + tunable constants)
# ---------------------------------------------------------------------------

class AppState:
    """Thread-shared live data and tunable constants.

    CPython GIL covers scalar attribute reads/writes; use .lock only for
    the bt_macs dict.
    """
    def __init__(self):
        self.lock           = threading.Lock()
        self.sm_state       = ATTRACT
        self.happy_count    = 0      # ATTRACT→TRACKING transitions
        self.sad_count      = 0      # TRACKING→SAD transitions
        self.start_time     = time.time()
        self.radar_az_deg    = 0.0
        self.radar_range_m   = 0.0
        self.radar_az_sm     = 0.0   # EMA-smoothed (used for aiming)
        self.radar_range_sm  = 0.0
        self.radar_speed     = 0.0   # cm/s, selected target
        self.radar_com_mm    = 0.0   # lateral COM of all valid targets
        self.radar_raw_targets = []  # [(az°, dist_m, spd, px_dist, is_selected), ...]
        self.radar_min_m     = RADAR_MIN_M
        self.radar_max_m     = RADAR_MAX_M
        self.pan_dmx        = PAN_ZERO
        self.tilt_dmx       = TILT_ZERO
        self.elev_deg       = 0.0
        self.slant_m        = 0.0
        self.bt_macs: dict  = {}     # MAC str → last_seen epoch float
        self.bt_rssi: dict  = {}     # MAC str → latest RSSI dBm (int)
        self.bt_peak_10m    = 0      # highest active-10m count seen
        # Tunable constants — start from module-level defaults
        self.radar_height_m = RADAR_HEIGHT_M
        self.light_offset_m = LIGHT_OFFSET_M
        self.human_height_m = HUMAN_HEIGHT_M
        self.pan_zero       = PAN_ZERO
        self.tilt_zero      = TILT_ZERO
        self.pan_range_deg  = float(PAN_RANGE_DEG)
        self.tilt_range_deg = float(TILT_RANGE_DEG)
        self.sad_linger_s   = SAD_LINGER_S
        self.track_speed    = TRACK_SPEED    # XY speed channel during tracking
        self.ema_alpha      = EMA_ALPHA      # radar smoothing factor


def _elev_from_state(range_m: float, s: AppState) -> float:
    # Positive vert means light is above person → negate so the fixture tilts DOWN.
    vert = (s.radar_height_m - s.light_offset_m) - s.human_height_m
    return -math.degrees(math.atan2(vert, range_m))


def _aim_dmx_from_state(az_deg: float, elev_deg: float, s: AppState):
    pan  = clamp(s.pan_zero  + int((az_deg   / (s.pan_range_deg  / 2)) * 127))
    tilt = clamp(s.tilt_zero + int((elev_deg  / (s.tilt_range_deg / 2)) * 127))
    return pan, tilt


# ---------------------------------------------------------------------------
# Data logger  (SQLite + WAL, batched writes)
# ---------------------------------------------------------------------------

class DataLogger:
    """Append-only SQLite log with in-RAM batching.

    WAL mode + synchronous=NORMAL + large page cache keeps SD write pressure
    minimal — one fsync per flush call (default: every 60 s from the TUI tick).
    """

    _DDL = """
    CREATE TABLE IF NOT EXISTS state_log (
        ts       REAL, state TEXT,
        az_deg   REAL, range_m  REAL, speed   REAL, com_mm   REAL,
        pan_dmx  INT,  tilt_dmx INT,  elev_deg REAL, slant_m  REAL,
        bt_active INT
    );
    CREATE TABLE IF NOT EXISTS bt_events (
        ts REAL, mac TEXT, rssi INT
    );
    """

    def __init__(self, db_path: str):
        self._db = sqlite3.connect(db_path, check_same_thread=False)
        self._db.execute("PRAGMA journal_mode=WAL")
        self._db.execute("PRAGMA synchronous=NORMAL")
        self._db.execute("PRAGMA cache_size=-32000")   # 32 MB page cache
        self._db.execute("PRAGMA temp_store=MEMORY")
        self._db.executescript(self._DDL)
        self._db.commit()
        self._lock      = threading.Lock()
        self._state_buf = []
        self._bt_buf    = []
        self._bt_last: dict = {}   # mac → last-logged ts (throttle)

    def log_state(self, s: AppState) -> None:
        now = time.time()
        active = sum(1 for t in s.bt_macs.values() if now - t < 600)
        row = (now, s.sm_state,
               s.radar_az_deg, s.radar_range_m, s.radar_speed, s.radar_com_mm,
               s.pan_dmx, s.tilt_dmx, s.elev_deg, s.slant_m, active)
        with self._lock:
            self._state_buf.append(row)

    def log_bt(self, mac: str, rssi) -> None:
        now = time.time()
        with self._lock:
            if now - self._bt_last.get(mac, 0) < 30:   # at most once per 30 s per MAC
                return
            self._bt_last[mac] = now
            self._bt_buf.append((now, mac, rssi))

    def flush(self) -> None:
        with self._lock:
            state_rows, self._state_buf = self._state_buf[:], []
            bt_rows,    self._bt_buf    = self._bt_buf[:],    []
        if not state_rows and not bt_rows:
            return
        try:
            with self._db:
                if state_rows:
                    self._db.executemany(
                        "INSERT INTO state_log VALUES (?,?,?,?,?,?,?,?,?,?,?)", state_rows)
                if bt_rows:
                    self._db.executemany(
                        "INSERT INTO bt_events VALUES (?,?,?)", bt_rows)
        except Exception:
            pass

    def close(self) -> None:
        self.flush()
        try:
            self._db.execute("PRAGMA wal_checkpoint(TRUNCATE)")
        except Exception:
            pass
        self._db.close()


class DMXSender:
    DMX_BAUD = 250000

    def __init__(self):
        devices = Ftdi.list_devices()
        if not devices:
            raise RuntimeError("No FTDI device found - is the Enttec plugged in?")
        desc = devices[0][0]
        url = f'ftdi://0x{desc.vid:04x}:0x{desc.pid:04x}:{desc.sn}/1'
        self.ftdi = Ftdi()
        self.ftdi.open_from_url(url)
        self.ftdi.set_baudrate(self.DMX_BAUD)
        self.ftdi.set_line_property(8, 2, 'N')

    def send(self, frame):
        self.ftdi.set_break(True)
        time.sleep(0.000176)   # 176μs break (DMX spec minimum: 88μs)
        self.ftdi.set_break(False)
        time.sleep(0.000012)   # 12μs mark-after-break (spec minimum: 8μs)
        packet = bytes([0x00] + list(frame) + [0x00] * (512 - len(frame)))
        self.ftdi.write_data(packet)

    def close(self):
        self.ftdi.close()


def home(dmx):
    print("Homing fixture...")
    frame = make_frame(dimmer=0)
    frame[13] = 200  # reset
    dmx.send(frame)
    time.sleep(8)
    frame[13] = 0
    frame[0] = PAN_ZERO
    frame[2] = TILT_ZERO
    frame[5] = 255
    dmx.send(frame)
    print("Homed.")


def compute_elevation(range_m):
    vertical = (RADAR_HEIGHT_M - LIGHT_OFFSET_M) - HUMAN_HEIGHT_M
    return -math.degrees(math.atan2(vertical, range_m))


def aim_angles(pan_deg, tilt_deg):
    """Convert real-world angles to DMX values.
    pan_deg:  degrees from forward (0=center, +right, -left), range ±PAN_RANGE_DEG/2
    tilt_deg: degrees from level   (0=horizontal, +up, -down), range ±TILT_RANGE_DEG/2
    """
    pan  = PAN_ZERO  + int((pan_deg  / (PAN_RANGE_DEG  / 2)) * 127)
    tilt = TILT_ZERO + int((tilt_deg / (TILT_RANGE_DEG / 2)) * 127)
    return clamp(pan), clamp(tilt)


def aim_light(azimuth_deg, range_m):
    elevation = compute_elevation(range_m)
    return aim_angles(azimuth_deg, elevation)


def aim_info(azimuth_deg, range_m):
    """Return (elevation_deg, slant_dist_m) for the center-of-mass aim point."""
    elevation = compute_elevation(range_m)
    light_height = RADAR_HEIGHT_M - LIGHT_OFFSET_M
    vert = light_height - HUMAN_HEIGHT_M
    slant = math.sqrt(range_m**2 + vert**2)
    return elevation, slant


def get_latest_radar(radar):
    """Poll radar, return (angle_deg, distance_m) for primary target or None.

    Requires pixel_distance > 0 to reject empty/ghost target slots that the
    RD03D pads into unused positions when fewer than 3 targets are present.
    """
    if radar.update():
        try:
            target = radar.get_target(1)
            if target.distance > 0 and target.pixel_distance > 0:
                return target.angle, target.distance / 1000.0  # mm -> m
        except Exception:
            pass
    return None


# ---------------------------------------------------------------------------
# Attract mode - slow pan sweep, spinning color wheel
# ---------------------------------------------------------------------------

def attract_loop(dmx, stop_event):
    pan     = PAN_ZERO
    pan_dir = 1
    pan_min = clamp(PAN_ZERO - 80)
    pan_max = clamp(PAN_ZERO + 80)

    while not stop_event.is_set():
        pan += pan_dir
        if pan >= pan_max:
            pan_dir = -1
        elif pan <= pan_min:
            pan_dir = 1

        frame = make_frame(pan=pan, tilt=TILT_ZERO, speed=200,
                           dimmer=255, color=COLOR_SPIN)
        dmx.send(frame)
        time.sleep(ATTRACT_STEP_S)


def attract_loop_s(dmx, stop_event, app_state: AppState):
    """attract_loop variant that reads calibration constants from AppState."""
    pan     = app_state.pan_zero
    pan_dir = 1

    while not stop_event.is_set():
        center  = app_state.pan_zero
        pan_min = clamp(center - 80)
        pan_max = clamp(center + 80)
        pan    += pan_dir
        if pan >= pan_max:
            pan_dir = -1
        elif pan <= pan_min:
            pan_dir = 1
        frame = make_frame(pan=pan, tilt=app_state.tilt_zero,
                           speed=200, dimmer=255, color=COLOR_SPIN)
        dmx.send(frame)
        time.sleep(ATTRACT_STEP_S)


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

ATTRACT  = 'attract'
TRACKING = 'tracking'
SAD      = 'sad'


def run(dmx, radar_port):
    state          = ATTRACT
    last_seen_time = 0
    attract_stop   = threading.Event()
    attract_thread = None

    def start_attract():
        nonlocal attract_thread
        attract_stop.clear()
        attract_thread = threading.Thread(target=attract_loop,
                                          args=(dmx, attract_stop), daemon=True)
        attract_thread.start()
        print("State: ATTRACT")

    def stop_attract():
        attract_stop.set()
        if attract_thread:
            attract_thread.join(timeout=1)

    start_attract()

    try:
        while True:
            result = get_latest_radar(radar_port)
            if result is not None:
                azimuth, range_m = result
                last_seen_time = time.time()

            human_present = (time.time() - last_seen_time) < PRESENCE_TIMEOUT_S

            if human_present:
                if state != TRACKING:
                    stop_attract()
                    state = TRACKING
                    print("State: TRACKING")
                if result is not None:
                    pan, tilt = aim_light(azimuth, range_m)
                    elev, slant = aim_info(azimuth, range_m)
                    print(f"  az={azimuth:+.1f}° elev={elev:+.1f}° | radar={range_m:.2f}m slant={slant:.2f}m -> pan={pan} tilt={tilt}")
                frame = make_frame(pan=pan, tilt=tilt, speed=0,
                                   dimmer=255, color=COLOR_WHITE)
                dmx.send(frame)

            else:
                if state == TRACKING:
                    state = SAD
                    print("State: SAD")

                if state == SAD:
                    elapsed = time.time() - last_seen_time
                    fade = clamp(int(255 * (1 - elapsed / SAD_LINGER_S)))
                    frame = make_frame(pan=PAN_ZERO, tilt=TILT_ZERO, speed=220,
                                       dimmer=fade, color=COLOR_BLUE)
                    dmx.send(frame)
                    time.sleep(0.05)

                    if elapsed >= SAD_LINGER_S:
                        start_attract()
                        state = ATTRACT

    except KeyboardInterrupt:
        pass
    finally:
        stop_attract()


def run_with_state(dmx, radar_port, app_state: AppState, stop_event: threading.Event,
                   logger=None):
    """State machine that writes live data and reads tunable constants from AppState."""
    state          = ATTRACT
    last_seen_time = 0.0
    last_log_time  = 0.0
    attract_stop   = threading.Event()
    attract_thread = None
    pan            = app_state.pan_zero
    tilt           = app_state.tilt_zero
    az_sm          = None   # EMA-smoothed azimuth
    range_sm       = None   # EMA-smoothed range

    def start_attract():
        nonlocal attract_thread
        attract_stop.clear()
        attract_thread = threading.Thread(
            target=attract_loop_s, args=(dmx, attract_stop, app_state), daemon=True)
        attract_thread.start()
        app_state.sm_state = ATTRACT

    def stop_attract():
        attract_stop.set()
        if attract_thread:
            attract_thread.join(timeout=1)

    start_attract()

    while not stop_event.is_set():
        if radar_port.update():
            mn_mm = app_state.radar_min_m * 1000
            mx_mm = app_state.radar_max_m * 1000
            valid = [t for t in radar_port.targets
                     if t.pixel_distance > 0 and mn_mm <= t.distance <= mx_mm]

            best = None
            if valid:
                if az_sm is not None:
                    # Sticky: pick whichever valid target is closest to our last
                    # known smoothed position (converts az diff to approx metres).
                    def _cost(t):
                        da = math.radians(t.angle - az_sm) * range_sm
                        dr = t.distance / 1000.0 - range_sm
                        return da * da + dr * dr
                    best = min(valid, key=_cost)
                else:
                    best = max(valid, key=lambda t: t.pixel_distance)

                azimuth = best.angle
                range_m = best.distance / 1000.0
                last_seen_time = time.time()

                alpha = app_state.ema_alpha
                if az_sm is None:
                    az_sm, range_sm = azimuth, range_m
                else:
                    az_sm    = alpha * azimuth + (1 - alpha) * az_sm
                    range_sm = alpha * range_m + (1 - alpha) * range_sm

                app_state.radar_az_deg  = azimuth
                app_state.radar_range_m = range_m
                app_state.radar_az_sm   = az_sm
                app_state.radar_range_sm = range_sm
                app_state.radar_speed   = best.speed
                app_state.radar_com_mm  = sum(t.x for t in valid) / len(valid)

            # Always refresh raw target display so TUI shows what radar sees
            sel_az = best.angle    if best else None
            sel_d  = best.distance if best else None
            app_state.radar_raw_targets = [
                (t.angle, t.distance / 1000.0, t.speed, t.pixel_distance,
                 t.angle == sel_az and t.distance == sel_d)
                for t in radar_port.targets
            ]

        human_present = (time.time() - last_seen_time) < PRESENCE_TIMEOUT_S

        if human_present:
            if state != TRACKING:
                stop_attract()
                state                  = TRACKING
                app_state.sm_state     = TRACKING
                app_state.happy_count += 1
                az_sm = range_sm = None   # reset EMA so stale history doesn't drag aim
            if az_sm is not None:
                elev  = _elev_from_state(range_sm, app_state)
                lh    = app_state.radar_height_m - app_state.light_offset_m
                vert  = lh - app_state.human_height_m
                slant = math.sqrt(range_sm**2 + vert**2)
                pan, tilt = _aim_dmx_from_state(az_sm, elev, app_state)
                app_state.pan_dmx  = pan
                app_state.tilt_dmx = tilt
                app_state.elev_deg = elev
                app_state.slant_m  = slant
            frame = make_frame(pan=pan, tilt=tilt,
                               speed=app_state.track_speed, dimmer=255, color=COLOR_WHITE)
            dmx.send(frame)

        else:
            if state == TRACKING:
                state                 = SAD
                app_state.sm_state    = SAD
                app_state.sad_count  += 1

            if state == SAD:
                elapsed = time.time() - last_seen_time
                fade    = clamp(int(255 * (1 - elapsed / app_state.sad_linger_s)))
                frame   = make_frame(pan=app_state.pan_zero, tilt=app_state.tilt_zero,
                                     speed=220, dimmer=fade, color=COLOR_BLUE)
                dmx.send(frame)
                time.sleep(0.05)
                if elapsed >= app_state.sad_linger_s:
                    start_attract()
                    state = ATTRACT

        # Log state every 5 s
        if logger:
            now = time.time()
            if now - last_log_time >= 5.0:
                logger.log_state(app_state)
                last_log_time = now


# ---------------------------------------------------------------------------
# Bluetooth scanner — passive LE advertisement sweep
# ---------------------------------------------------------------------------

_MAC_RE  = re.compile(r'Address:\s+([0-9A-Fa-f]{2}(?::[0-9A-Fa-f]{2}){5})\s')
_RSSI_RE = re.compile(r'RSSI:\s+(-?\d+)\s+dBm')


def scan_bluetooth(app_state: AppState, stop_event: threading.Event):
    """Fallback: hcitool lescan only — no RSSI."""
    try:
        proc = subprocess.Popen(
            ['hcitool', 'lescan', '--duplicates'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
    except (FileNotFoundError, OSError):
        return
    try:
        while not stop_event.is_set():
            line = proc.stdout.readline()
            if not line:
                break
            parts = line.strip().split()
            if parts and len(parts[0]) == 17 and parts[0].count(':') == 5:
                with app_state.lock:
                    app_state.bt_macs[parts[0].upper()] = time.time()
    finally:
        try:
            proc.terminate()
        except OSError:
            pass


def scan_bluetooth_btmon(app_state: AppState, logger, stop_event: threading.Event):
    """hcitool lescan (to drive the controller) + btmon (to harvest RSSI).

    btmon is a passive HCI monitor — it sees every advertising event the
    kernel processes, including RSSI, without interfering with the scan.
    Falls back to no-RSSI hcitool path if btmon is unavailable.
    """
    scanner = btmon = None
    try:
        scanner = subprocess.Popen(
            ['hcitool', 'lescan', '--duplicates'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        btmon = subprocess.Popen(
            ['btmon', '--no-pager'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
    except (FileNotFoundError, OSError):
        for p in (scanner, btmon):
            if p:
                try: p.terminate()
                except OSError: pass
        scan_bluetooth(app_state, stop_event)
        return

    current_mac = None
    try:
        for line in btmon.stdout:
            if stop_event.is_set():
                break
            # A line starting with >, <, = marks a new HCI event
            if line and line[0] in ('>', '<', '='):
                current_mac = None

            stripped = line.strip()

            m = _MAC_RE.search(stripped)
            if m and 'type' not in stripped.lower():
                current_mac = m.group(1).upper()
                continue

            m = _RSSI_RE.search(stripped)
            if m and current_mac:
                rssi = int(m.group(1))
                now  = time.time()
                with app_state.lock:
                    app_state.bt_macs[current_mac] = now
                    app_state.bt_rssi[current_mac]  = rssi
                if logger:
                    logger.log_bt(current_mac, rssi)
                current_mac = None
    finally:
        for p in (btmon, scanner):
            if p:
                try: p.terminate()
                except OSError: pass


# ---------------------------------------------------------------------------
# Test shell
# ---------------------------------------------------------------------------

TEST_HELP = """
Commands:
  pan <0-255>          set pan position
  tilt <0-255>         set tilt position
  speed <0-255>        pan/tilt speed (0=fast, 255=slow)
  dim <0-255>          master dimmer
  strobe <0-255>       strobe (0=off, 4+=strobing)
  color <0-255>        color wheel (0=white, 9=red, 14=green, 19=blue, 24=yellow, 200+=spin)
  white                white light, full dimmer
  blue                 blue, full dimmer
  red                  red, full dimmer
  gobo <0-255>         solid gobo wheel
  colorful <0-255>     colorful gobo wheel
  prism <0-255>        prism (0-127=in/out, 128+=rotate)
  focus <0-255>        focus
  home                 run home sequence
  attract              run attract mode (Enter to stop)
  sad                  show sad fade-to-blue sequence
  aim <pan> <tilt>     aim by angle: pan deg from center, tilt deg from level
  track <az> <range>   simulate radar hit (azimuth degrees, range meters)
  ch <n> <v>           set raw DMX channel n (1-14) to value     [debug]
  clearall             zero every channel                        [debug]
  freeze               zero all except pan/tilt (safe probing)   [debug]
  dump                 show current frame values                  [debug]
  q / quit             exit
"""


def test_shell(dmx):
    pan      = PAN_ZERO
    tilt     = TILT_ZERO
    speed    = 0
    dimmer   = 255
    strobe   = 0
    color    = COLOR_WHITE
    gobo     = 0
    colorful = 0
    prism    = 0
    prism_rot= 0
    focus    = 128
    raw_overrides = {}
    lock = threading.Lock()

    def current_frame():
        frame = make_frame(pan=pan, tilt=tilt, speed=speed, dimmer=dimmer,
                           strobe=strobe, color=color, gobo=gobo,
                           colorful=colorful, prism=prism, prism_rot=prism_rot,
                           focus=focus)
        for idx, val in raw_overrides.items():
            frame[idx] = val
        return frame

    # Continuous DMX stream - fixture needs this or it returns to home
    stop_stream = threading.Event()
    def stream_loop():
        while not stop_stream.is_set():
            with lock:
                frame = current_frame()
            dmx.send(frame)
            time.sleep(1/30)

    stream_thread = threading.Thread(target=stream_loop, daemon=True)
    stream_thread.start()

    def send():
        pass  # stream loop picks up state changes automatically

    print(TEST_HELP)
    send()

    while True:
        try:
            raw = input("fakelight> ").strip()
        except EOFError:
            break
        if not raw:
            continue
        parts = raw.split()
        cmd = parts[0].lower()

        if cmd in ('q', 'quit'):
            stop_stream.set()
            break
        elif cmd == 'pan' and len(parts) == 2:
            with lock: pan = clamp(int(parts[1]))
        elif cmd == 'tilt' and len(parts) == 2:
            with lock: tilt = clamp(int(parts[1]))
        elif cmd == 'speed' and len(parts) == 2:
            with lock: speed = clamp(int(parts[1]))
        elif cmd == 'dim' and len(parts) == 2:
            with lock: dimmer = clamp(int(parts[1]))
        elif cmd == 'strobe' and len(parts) == 2:
            with lock: strobe = clamp(int(parts[1]))
        elif cmd == 'color' and len(parts) == 2:
            with lock: color = clamp(int(parts[1]))
        elif cmd == 'white':
            with lock: color, dimmer = COLOR_WHITE, 255
        elif cmd == 'blue':
            with lock: color, dimmer = COLOR_BLUE, 255
        elif cmd == 'red':
            with lock: color, dimmer = COLOR_RED, 255
        elif cmd == 'gobo' and len(parts) == 2:
            with lock: gobo = clamp(int(parts[1]))
        elif cmd == 'colorful' and len(parts) == 2:
            with lock: colorful = clamp(int(parts[1]))
        elif cmd == 'prism' and len(parts) == 2:
            with lock: prism = clamp(int(parts[1]))
        elif cmd == 'focus' and len(parts) == 2:
            with lock: focus = clamp(int(parts[1]))
        elif cmd == 'home':
            stop_stream.set(); stream_thread.join(timeout=1)
            home(dmx)
            with lock: pan, tilt = PAN_ZERO, TILT_ZERO
            stop_stream.clear()
            stream_thread = threading.Thread(target=stream_loop, daemon=True)
            stream_thread.start()
        elif cmd == 'attract':
            stop_stream.set(); stream_thread.join(timeout=1)
            stop = threading.Event()
            t = threading.Thread(target=attract_loop, args=(dmx, stop), daemon=True)
            t.start()
            print("Attract mode running - press Enter to stop")
            input()
            stop.set(); t.join(timeout=1)
            stop_stream.clear()
            stream_thread = threading.Thread(target=stream_loop, daemon=True)
            stream_thread.start()
        elif cmd == 'sad':
            stop_stream.set(); stream_thread.join(timeout=1)
            for fade in range(255, 0, -2):
                dmx.send(make_frame(pan=PAN_ZERO, tilt=TILT_ZERO, speed=220,
                                    dimmer=fade, color=COLOR_BLUE))
                time.sleep(0.02)
            with lock: color, dimmer = COLOR_BLUE, 0
            stop_stream.clear()
            stream_thread = threading.Thread(target=stream_loop, daemon=True)
            stream_thread.start()
        elif cmd == 'aim' and len(parts) == 3:
            pan_deg  = float(parts[1])
            tilt_deg = float(parts[2])
            p, t2 = aim_angles(pan_deg, tilt_deg)
            with lock:
                pan, tilt = p, t2
                color, dimmer = COLOR_WHITE, 255
            print(f"  pan {pan_deg:+.1f}° -> DMX {p}  |  tilt {tilt_deg:+.1f}° -> DMX {t2}")
        elif cmd == 'track' and len(parts) == 3:
            az, rng = float(parts[1]), float(parts[2])
            p, t2 = aim_light(az, rng)
            elev, slant = aim_info(az, rng)
            with lock:
                pan, tilt = p, t2
                color, dimmer = COLOR_WHITE, 255
            print(f"  az={az:+.1f}° elev={elev:+.1f}° | radar={rng:.2f}m slant={slant:.2f}m -> pan={p} tilt={t2}")
        elif cmd == 'clearall':
            with lock:
                raw_overrides.clear()
                color, dimmer, strobe, speed = COLOR_WHITE, 0, 0, 0
                gobo, colorful, prism, prism_rot = 0, 0, 0, 0
            print("All channels zeroed.")
        elif cmd == 'freeze':
            with lock:
                raw_overrides.clear()
                color, dimmer, strobe, speed = COLOR_WHITE, 255, 0, 0
                gobo, colorful, prism, prism_rot = 0, 0, 0, 0
            print(f"Frozen at pan={pan} tilt={tilt}")
        elif cmd == 'ch' and len(parts) == 3:
            idx = int(parts[1]) - 1
            val = clamp(int(parts[2]))
            with lock: raw_overrides[idx] = val
            print(f"  ch{parts[1]} (index {idx}) = {val}")
        elif cmd == 'dump':
            frame = make_frame(pan=pan, tilt=tilt, speed=speed, dimmer=dimmer,
                               strobe=strobe, color=color, gobo=gobo,
                               colorful=colorful, prism=prism, prism_rot=prism_rot,
                               focus=focus)
            for idx, val in raw_overrides.items():
                frame[idx] = val
            labels = ['pan','pan_fine','tilt','tilt_fine','speed','dimmer',
                      'strobe','color','gobo','colorful','prism','prism_rot',
                      'focus','reset']
            for i, (v, lbl) in enumerate(zip(frame, labels)):
                print(f"  ch{i+1:2d} {lbl:<12} = {v}")
        else:
            print(TEST_HELP)


# ---------------------------------------------------------------------------
# TUI  (--tui mode)
# ---------------------------------------------------------------------------

# Tunable constant descriptors:
# (attr, label, unit, step, big_step, min_val, max_val, is_float)
TUNABLE = [
    ('radar_height_m', 'RADAR_HEIGHT_M', 'm',  0.1,  1.0,  0.0, 20.0, True),
    ('light_offset_m', 'LIGHT_OFFSET_M', 'm',  0.1,  1.0,  0.0, 10.0, True),
    ('human_height_m', 'HUMAN_HEIGHT_M', 'm',  0.1,  1.0,  0.0,  3.0, True),
    ('pan_zero',       'PAN_ZERO',       '',    1,   10,    0,  255, False),
    ('tilt_zero',      'TILT_ZERO',      '',    1,   10,    0,  255, False),
    ('pan_range_deg',  'PAN_RANGE_DEG',  '°',  5,   45,   90,  720, True),
    ('tilt_range_deg', 'TILT_RANGE_DEG', '°',  5,   45,   45,  360, True),
    ('sad_linger_s',   'SAD_LINGER_S',   's',  0.5,  5.0,  0.5, 60.0, True),
    ('track_speed',    'TRACK_SPEED',    '',   5,   20,    0,  100, False),
    ('ema_alpha',      'EMA_ALPHA',      '',   0.05, 0.2, 0.05, 1.0, True),
    ('radar_min_m',    'RADAR_MIN_M',    'm',  0.1,  0.5, 0.0,  5.0, True),
    ('radar_max_m',    'RADAR_MAX_M',    'm',  0.5,  2.0, 1.0, 20.0, True),
]

_STATE_COLOR = {ATTRACT: 'yellow', TRACKING: 'green', SAD: 'dodger_blue1'}
_STATE_ICON  = {ATTRACT: '◐',  TRACKING: '●', SAD: '○'}


def run_tui(dmx, radar_port):
    try:
        from textual.app import App, ComposeResult
        from textual.containers import Horizontal
        from textual.widget import Widget
        from textual.widgets import Static, Footer
        from textual import events
        from rich.text import Text
    except ImportError:
        print("textual not installed — run: pip install textual", file=sys.stderr)
        sys.exit(1)

    class ConstantsPanel(Widget):
        can_focus = True
        DEFAULT_CSS = """
        ConstantsPanel {
            height: auto;
            border: solid $accent;
            padding: 0 2;
        }
        ConstantsPanel:focus {
            border: solid $success;
        }
        """

        def __init__(self, app_state: AppState, **kwargs):
            super().__init__(**kwargs)
            self._s   = app_state
            self._sel = 0

        def render(self) -> Text:
            t = Text()
            t.append('Constants', style='bold')
            t.append('  (↑↓ select   ←→ adjust   shift=10×)\n', style='dim')
            for i, (attr, label, unit, step, big, mn, mx, is_float) in enumerate(TUNABLE):
                val = getattr(self._s, attr)
                pfx = '▶ ' if i == self._sel else '  '
                vs  = f'{val:.1f}' if is_float else str(int(val))
                row = f'{pfx}{label:<18} {vs:>6} {unit}'
                style = 'bold reverse' if i == self._sel else ''
                t.append(row + '\n', style=style)
            return t

        def on_key(self, event: events.Key) -> None:
            key = event.key
            if key == 'up':
                self._sel = (self._sel - 1) % len(TUNABLE)
                self.refresh(); event.stop()
            elif key == 'down':
                self._sel = (self._sel + 1) % len(TUNABLE)
                self.refresh(); event.stop()
            elif key == 'left':
                self._nudge(-1, False); event.stop()
            elif key == 'right':
                self._nudge(1, False); event.stop()
            elif key == 'shift+left':
                self._nudge(-1, True); event.stop()
            elif key == 'shift+right':
                self._nudge(1, True); event.stop()

        def _nudge(self, direction: int, large: bool) -> None:
            attr, _, _, step, big, mn, mx, is_float = TUNABLE[self._sel]
            delta = big if large else step
            val   = getattr(self._s, attr)
            val   = max(mn, min(mx, val + direction * delta))
            if is_float:
                val = round(val, 2)
            setattr(self._s, attr, val)
            self.refresh()

    class FakeLightApp(App):
        CSS = """
Screen { layout: vertical; }
#state-bar {
    height: 3;
    border: solid $primary;
    content-align: center middle;
    text-style: bold;
}
#live-row { height: 12; }
#radar-panel {
    width: 1fr;
    border: solid $accent;
    padding: 0 1;
}
#light-panel {
    width: 1fr;
    border: solid $accent;
    padding: 0 1;
}
#bt-panel {
    height: 7;
    border: solid $accent;
    padding: 0 1;
}
"""

        def __init__(self, app_state: AppState, **kwargs):
            super().__init__(**kwargs)
            self._s = app_state

        def compose(self) -> ComposeResult:
            yield Static(id='state-bar')
            with Horizontal(id='live-row'):
                yield Static(id='radar-panel')
                yield Static(id='light-panel')
            yield Static(id='bt-panel')
            yield ConstantsPanel(self._s)
            yield Footer()

        def on_mount(self) -> None:
            self.set_interval(0.1, self._tick)
            self.query_one(ConstantsPanel).focus()

        def _tick(self) -> None:
            s   = self._s
            now = time.time()

            # --- State bar ---
            color   = _STATE_COLOR.get(s.sm_state, 'white')
            icon    = _STATE_ICON.get(s.sm_state, '?')
            elapsed = now - s.start_time
            h  = int(elapsed // 3600)
            m  = int((elapsed % 3600) // 60)
            sc = int(elapsed % 60)
            self.query_one('#state-bar', Static).update(
                f'[bold {color}]{icon} {s.sm_state.upper()}[/]   '
                f'happy: {s.happy_count}  sad: {s.sad_count}   '
                f'runtime: {h:02d}:{m:02d}:{sc:02d}'
            )

            # --- Radar ---
            tgt_lines = []
            for i, (az, dist, spd, px, sel) in enumerate(
                    s.radar_raw_targets or [(0,0,0,0,False)]*3):
                marker = '[bold green]→[/]' if sel else ' '
                if px > 0:
                    tgt_lines.append(
                        f'  {marker}T{i+1} az={az:+6.1f}°  '
                        f'rng={dist:5.2f}m  spd={spd:4.0f}  px={px}')
                else:
                    tgt_lines.append(f'   T{i+1} [dim](empty)[/]')
            self.query_one('#radar-panel', Static).update(
                f'[bold]Radar[/]  COM off: {s.radar_com_mm:+.0f}mm\n'
                + '\n'.join(tgt_lines) + '\n'
                f'  [dim]smooth → az={s.radar_az_sm:+6.1f}°  '
                f'rng={s.radar_range_sm:5.2f}m[/]'
            )

            # --- Light ---
            self.query_one('#light-panel', Static).update(
                f'[bold]Light target[/]\n'
                f'  pan:     {s.pan_dmx:>5}  DMX\n'
                f'  tilt:    {s.tilt_dmx:>5}  DMX\n'
                f'  elev:    {s.elev_deg:+7.1f}°\n'
                f'  slant:   {s.slant_m:7.2f} m'
            )

            # --- Bluetooth ---
            with s.lock:
                macs     = dict(s.bt_macs)
                rssi_map = dict(s.bt_rssi)
            total      = len(macs)
            active_10m = sum(1 for t in macs.values() if now - t < 600)
            last_60    = sum(1 for t in macs.values() if now - t < 60)
            last_5     = sum(1 for t in macs.values() if now - t < 5)
            s.bt_peak_10m = max(s.bt_peak_10m, active_10m)

            # RSSI stats — only for MACs active in the last 10 min
            active_rssis = sorted(
                rssi_map[m] for m in macs if m in rssi_map and now - macs[m] < 600
            )
            if active_rssis:
                med   = active_rssis[len(active_rssis) // 2]
                close = sum(1 for r in active_rssis if r > -80)  # rough <10 m threshold
                rssi_line = f'  RSSI median: {med} dBm   close (<~10m): {close}'
            else:
                rssi_line = '  RSSI: no data yet'

            recent   = sorted(macs.items(), key=lambda x: -x[1])[:5]
            mac_line = ('  ' + '  '.join(m[:11] + '…' for m, _ in recent)
                        if recent else '  (none yet)')

            self.query_one('#bt-panel', Static).update(
                f'[bold]Bluetooth[/]  '
                f'[bold green]~{active_10m} here[/] (10m)  '
                f'peak: {s.bt_peak_10m}  total: {total}  '
                f'last 60s: {last_60}  last 5s: {last_5}\n'
                f'{rssi_line}\n'
                f'{mac_line}'
            )

            # Flush logger once per minute (cheap — just commits a batch)
            if self._data_logger and now - self._last_flush >= 60:
                self._data_logger.flush()
                self._last_flush = now

    app_state  = AppState()
    stop_event = threading.Event()

    db_path = time.strftime('fakelight_%Y%m%d_%H%M%S.db')
    logger  = DataLogger(db_path)

    sm_thread = threading.Thread(
        target=run_with_state,
        args=(dmx, radar_port, app_state, stop_event, logger),
        daemon=True,
    )
    sm_thread.start()

    bt_thread = threading.Thread(
        target=scan_bluetooth_btmon,
        args=(app_state, logger, stop_event),
        daemon=True,
    )
    bt_thread.start()

    app                = FakeLightApp(app_state)
    app._data_logger   = logger
    app._last_flush    = time.time()

    try:
        app.run()
    finally:
        stop_event.set()
        sm_thread.join(timeout=2)
        bt_thread.join(timeout=2)
        logger.close()
        print(f"Session saved to {db_path}")


# ---------------------------------------------------------------------------
# Raw radar byte dump  (--raw-radar mode)
# ---------------------------------------------------------------------------

def dump_radar(port='/dev/ttyS0', baudrate=256000):
    """Print raw bytes and attempt frame decoding so we can see exactly what
    the radar hardware is sending — useful for diagnosing UART/baud issues."""
    uart = serial.Serial(port, baudrate, timeout=1.0)
    print(f"Raw radar dump  port={port}  baud={baudrate}")
    print("Green = valid-looking frame (AA FF … 55 CC, length OK)")
    print("Red   = header/footer match but wrong length")
    print("Grey  = orphan bytes (no frame detected)")
    print("Ctrl-C to stop\n")

    RESET  = '\033[0m'
    GREEN  = '\033[32m'
    RED    = '\033[31m'
    DIM    = '\033[2m'

    buf = b''
    frame_n = 0

    try:
        while True:
            chunk = uart.read(max(uart.in_waiting, 1))
            if not chunk:
                continue
            buf += chunk

            while True:
                # Scan for header
                start = -1
                for i in range(len(buf) - 1):
                    if buf[i] == 0xAA and buf[i + 1] == 0xFF:
                        start = i
                        break
                if start == -1:
                    if len(buf) > 4:
                        orphans = buf[:-2]
                        print(DIM + ' '.join(f'{b:02X}' for b in orphans) + RESET)
                        buf = buf[-2:]
                    break

                # Discard bytes before header
                if start > 0:
                    print(DIM + ' '.join(f'{b:02X}' for b in buf[:start]) + RESET)
                    buf = buf[start:]

                # Look for footer
                end = -1
                for i in range(2, len(buf) - 1):
                    if buf[i] == 0x55 and buf[i + 1] == 0xCC:
                        end = i + 2
                        break
                if end == -1:
                    break   # footer not yet received

                frame = buf[:end]
                buf   = buf[end:]
                frame_n += 1

                # rd03d.py only requires: len>=30, AA FF header, 55 CC footer
                ok = len(frame) == 30
                b23 = f'{frame[2]:02X} {frame[3]:02X}' if len(frame) >= 4 else '??'

                hex_str = ' '.join(f'{b:02X}' for b in frame)
                color   = GREEN if ok else RED
                print(f"{color}[{frame_n:04d}] len={len(frame)} hdr[2:4]={b23}  {hex_str}{RESET}")

                if ok:
                    # Decode and show target values
                    for i in range(3):
                        base = 4 + i * 8
                        raw_x  = (frame[base+1] << 8) + frame[base]
                        raw_y  = (frame[base+3] << 8) + frame[base+2]
                        raw_sp = (frame[base+5] << 8) + frame[base+4]
                        px     = frame[base+6] + (frame[base+7] << 8)
                        # Decode both sign conventions so user can see
                        def s16_stdbe(r):   # standard two's complement
                            return r - 65536 if r >= 32768 else r
                        def s16_signmag(r): # sign-magnitude (current code)
                            return (1 if r & 0x8000 else -1) * (r & 0x7FFF)
                        sx, sy = s16_stdbe(raw_x), s16_stdbe(raw_y)
                        mx, my = s16_signmag(raw_x), s16_signmag(raw_y)
                        if px > 0:
                            print(f"       T{i+1}: raw_x=0x{raw_x:04X} raw_y=0x{raw_y:04X} px={px:4d}"
                                  f"  2s-comp=({sx:+6d},{sy:+6d})mm"
                                  f"  sign-mag=({mx:+6d},{my:+6d})mm")

    except KeyboardInterrupt:
        pass
    finally:
        uart.close()


# ---------------------------------------------------------------------------
# Channel control board  (--ctrl mode)
# ---------------------------------------------------------------------------

# Per-channel metadata: (display label, safe default, hint string)
CHAN_INFO = [
    ('Pan (X-axis)',      128, '0–255 = 0–540°'),
    ('Pan fine',           0, ''),
    ('Tilt (Y-axis)',     64, '0–255 = 0–220°'),
    ('Tilt fine',          0, ''),
    ('XY Speed',           0, '0=fast  255=slow'),
    ('Dimmer',           255, '0=dark  255=bright'),
    ('Strobe',             0, '0–3=off  4–99=sync  100–149=pulse  200+=rand'),
    ('Color wheel',        0, '0=wht  9=red  14=grn  19=blu  24=yel  200+=spin'),
    ('Gobo solid',         0, '0=open  3–66=patterns  67+=shake'),
    ('Gobo colorful',      0, '0–127=static  128+=flow'),
    ('Prism',              0, '0–127=in/out  128+=rotate'),
    ('Prism rotation',     0, ''),
    ('Focus',            128, ''),
    ('Reset',              0, '⚠  200–205 = HARDWARE RESET'),
]


def run_ctrl(dmx):
    """14-channel DMX control board for discovering actual channel behaviour."""
    try:
        from textual.app import App, ComposeResult
        from textual.widget import Widget
        from textual.widgets import Static, Footer
        from textual import events
        from rich.text import Text
    except ImportError:
        print("textual not installed — run: pip install textual", file=sys.stderr)
        sys.exit(1)

    class Board(Widget):
        can_focus = True
        DEFAULT_CSS = """
        Board { height: auto; border: solid $accent; padding: 0 1; }
        Board:focus { border: solid $success; }
        """

        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self.vals    = [v for _, v, _ in CHAN_INFO]
            self._sel    = 0
            self._status = ''

        def render(self) -> Text:
            t = Text()
            t.append(f'  {"Ch":<4}  {"Channel":<20}  {"Val":>3}  '
                     f'{"":20}  Notes\n', style='dim')
            t.append('  ' + '─' * 76 + '\n', style='dim')
            for i, (label, _, notes) in enumerate(CHAN_INFO):
                v      = self.vals[i]
                bar    = '█' * int(v / 255 * 20)
                pfx    = '▶ ' if i == self._sel else '  '
                is_hot = (i == 13 and v >= 200)
                style  = ('bold reverse' if i == self._sel
                           else 'bold red'    if is_hot
                           else '')
                t.append(
                    f'{pfx}{i+1:<3}   {label:<20}  {v:>3}  {bar:<20}  {notes}\n',
                    style=style,
                )
            t.append('  ' + '─' * 76 + '\n', style='dim')
            t.append(
                '  ↑↓ select   ←→ ±1   shift+←→ ±10   '
                '0=zero ch   c=center(128)   s=safe defaults   '
                'z=zero ALL   h=home\n',
                style='dim',
            )
            if self._status:
                t.append(f'  {self._status}\n', style='bold yellow')
            return t

        def on_key(self, event: events.Key) -> None:
            k = event.key
            if k == 'up':
                self._sel = (self._sel - 1) % 14
                self.refresh(); event.stop()
            elif k == 'down':
                self._sel = (self._sel + 1) % 14
                self.refresh(); event.stop()
            elif k == 'left':
                self._nudge(-1, False); event.stop()
            elif k == 'right':
                self._nudge(1, False); event.stop()
            elif k == 'shift+left':
                self._nudge(-1, True); event.stop()
            elif k == 'shift+right':
                self._nudge(1, True); event.stop()
            elif k == '0':
                self.vals[self._sel] = 0
                self.refresh(); event.stop()
            elif k == 'c':
                self.vals[self._sel] = 128
                self.refresh(); event.stop()
            elif k == 's':
                self.vals = [v for _, v, _ in CHAN_INFO]
                self.refresh(); event.stop()
            elif k == 'z':
                self.vals = [0] * 14
                self.refresh(); event.stop()
            elif k == 'h':
                self._do_home(); event.stop()

        def _nudge(self, d: int, large: bool) -> None:
            delta = 10 if large else 1
            self.vals[self._sel] = clamp(self.vals[self._sel] + d * delta)
            self.refresh()

        def set_status(self, msg: str) -> None:
            self._status = msg
            self.refresh()

        def _do_home(self) -> None:
            self.set_status('Homing… (takes ~8s, fixture goes dark then re-centres)')

            def _run():
                home(dmx)
                self.vals = [v for _, v, _ in CHAN_INFO]
                self.set_status('Home complete.')

            threading.Thread(target=_run, daemon=True).start()

    class CtrlApp(App):
        CSS = """
Screen { layout: vertical; }
#hdr {
    height: 3;
    border: solid $primary;
    content-align: center middle;
    text-style: bold;
}
"""

        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            self._frames = 0

        def compose(self) -> ComposeResult:
            yield Static('', id='hdr')
            yield Board()
            yield Footer()

        def on_mount(self) -> None:
            self.set_interval(1 / 30, self._tick)
            self.query_one(Board).focus()

        def _tick(self) -> None:
            board = self.query_one(Board)
            dmx.send(board.vals)
            self._frames += 1
            v14  = board.vals[13]
            warn = (f'[bold red]⚠ RESET ARMED — ch14={v14}[/]'
                    if v14 >= 200 else f'ch14={v14}')
            self.query_one('#hdr', Static).update(
                f'[bold]DMX Channel Control Board[/]   '
                f'[green]● LIVE[/] {self._frames} frames sent   {warn}'
            )

    CtrlApp().run()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    # --raw-radar needs no DMX — handle before anything else
    if '--raw-radar' in sys.argv:
        idx  = sys.argv.index('--raw-radar')
        port = (sys.argv[idx + 1]
                if idx + 1 < len(sys.argv) and not sys.argv[idx + 1].startswith('--')
                else '/dev/ttyAMA0')
        dump_radar(port)
        return

    test_mode = '--test' in sys.argv
    tui_mode  = '--tui'  in sys.argv
    ctrl_mode = '--ctrl' in sys.argv

    dmx = DMXSender()

    if ctrl_mode:
        # No auto-home: start sending safe defaults immediately so you can
        # observe channel behaviour without the 8-second blackout.  Press h
        # inside the board to home manually.
        try:
            run_ctrl(dmx)
        finally:
            dmx.close()
        return

    home(dmx)

    if test_mode:
        print("Test shell ready.")
        try:
            test_shell(dmx)
        finally:
            dmx.close()
    elif tui_mode:
        radar = RD03D()
        radar.set_multi_mode(True)
        try:
            run_tui(dmx, radar)
        finally:
            dmx.close()
            radar.close()
        print("Shutting down.")
    else:
        radar = RD03D()
        radar.set_multi_mode(True)
        try:
            run(dmx, radar)
        finally:
            dmx.close()
            radar.close()
        print("Shutting down.")


if __name__ == '__main__':
    main()
