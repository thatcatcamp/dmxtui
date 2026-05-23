import math
import sys
import time
import threading
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
    return math.degrees(math.atan2(vertical, range_m))


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
    """Poll radar, return (angle_deg, distance_m) for primary target or None."""
    if radar.update():
        try:
            target = radar.get_target(1)
            if target.distance > 0:
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
# Entry point
# ---------------------------------------------------------------------------

def main():
    test_mode = '--test' in sys.argv

    dmx = DMXSender()
    home(dmx)

    if test_mode:
        print("Test shell ready.")
        try:
            test_shell(dmx)
        finally:
            dmx.close()
    else:
        radar = RD03D()
        radar.set_multi_mode(True)
        try:
            run(dmx, radar)
        finally:
            dmx.close()
        print("Shutting down.")


if __name__ == '__main__':
    main()
