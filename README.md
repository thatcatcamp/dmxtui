# FakeLight

Radar-guided DMX spotlight controller for a Burning Man flower sculpture.
The pistil assembly holds an RD03D mmWave radar and a Dragon King Star Beam
moving head. The radar finds the largest group of humans, the light tracks them.

## Hardware

| Part | Notes |
|------|-------|
| Dragon King Star Beam | 14-channel DMX moving head, gobo/prism/color wheel |
| Enttec Open DMX USB | FTDI FT232R-based USB→DMX adapter (vid=0403, pid=6001) |
| RD03D mmWave radar | Pi 5 UART on /dev/ttyAMA0, multi-target mode |
| Raspberry Pi (any) | Runs as root; Pi Zero 2W recommended for size |

### Physical geometry (measure at install, update constants in fakelight.py)

```
RADAR_HEIGHT_M = 4.0   # radar height above ground
LIGHT_OFFSET_M = 2.0   # light is this far BELOW the radar
HUMAN_HEIGHT_M = 1.7   # aim for center of mass
```

The light is mounted ~90° to the horizon on the pistil, approximately 2m below
the radar. Elevation to target is computed geometrically from range — no tilt
sensor needed.

## Pi Setup

```bash
# 1. Blacklist ftdi_sio so pyftdi can claim the USB adapter directly
echo "blacklist ftdi_sio" | sudo tee /etc/modprobe.d/ftdi-blacklist.conf
sudo modprobe -r ftdi_sio usbserial

# 2. Install Python deps (no OLA, no QLC+ needed)
pip install pyftdi rd03d

# 3. Run (must be root for radar UART access)
sudo python3 fakelight.py          # production mode
sudo python3 fakelight.py --test   # interactive test shell
```

## Fixture: Dragon King Star Beam — 14-channel DMX map

| Ch | Function | Key values |
|----|----------|------------|
| 1 | Pan | 0-255 = 0-540° |
| 2 | Pan fine | 0-255 |
| 3 | Tilt | 0-255 = 0-220° |
| 4 | Tilt fine | 0-255 |
| 5 | XY Speed | 0=fast, 255=slow |
| 6 | Dimmer | 0=dark, 255=bright |
| 7 | Strobe | 0-3=open, 4-99=sync, 100-149=pulse, 150+=random/consecutive |
| 8 | Color wheel | 0=white, 9=red, 14=green, 19=blue, 24=yellow, 200+=spin |
| 9 | Gobo solid | 0=open, 3-66=patterns 1-17, 67+=jitter/shake |
| 10 | Gobo colorful | 0-127=static, 128+=flowing effects |
| 11 | Prism | 0-127=cut in/out, 128-255=rotation |
| 12 | Prism rotation | 0-255 |
| 13 | Focus | 0-255 |
| 14 | Reset | 200-205=reset all |

**Fixture DMX address: 1** (set via on-body LCD menu, channel mode = 14)

## Calibration constants (top of fakelight.py)

```python
PAN_ZERO       = 128   # DMX value that points "forward" — tune at install
TILT_ZERO      = 64    # DMX value that's level for this mount — tune at install
PAN_RANGE_DEG  = 540   # physical pan range
TILT_RANGE_DEG = 220   # physical tilt range — measure hard stop to hard stop
```

To calibrate at install:
1. `sudo python3 fakelight.py --test`
2. `aim 0 0` — should point forward and level. Adjust PAN_ZERO / TILT_ZERO.
3. `aim 90 0` / `aim -90 0` — check left/right swing is symmetric.
4. `aim 0 -30` — should point down toward humans. Adjust TILT_RANGE_DEG if over/undershooting.

## Behavior states

```
ATTRACT  → slow pan sweep, spinning rainbow color wheel
             (no humans detected)
    ↓ human appears
TRACKING → white light, fast movement, tracks primary radar target
    ↓ human leaves
SAD      → blue light, slow drift to center, dimmer fades out over SAD_LINGER_S
    ↓ fade complete
ATTRACT  → (loop)
```

## Test shell commands

```
aim <pan_deg> <tilt_deg>   aim by real angles (0 0 = forward+level)
track <azimuth> <range_m>  simulate radar hit
white / blue / red         set color wheel
attract                    run attract mode (Enter to stop)
sad                        run sad fade sequence
gobo <0-255>               solid gobo patterns
colorful <0-255>           colorful gobo effects
prism <0-255>              prism in/out and rotation
focus <0-255>              focus adjustment
home                       hardware reset + go to PAN_ZERO/TILT_ZERO
dim <0-255>                master dimmer
speed <0-255>              pan/tilt speed (0=fast)
dump                       show all 14 DMX channel values
ch <n> <v>                 set raw channel n (1-14) — debug only
freeze                     hold position, zero everything else — safe probing
clearall                   zero all channels
```

## Known issues / TODO

- [ ] Confirm TILT_RANGE_DEG (measured ~220° but not verified against hard stops)
- [ ] Tune PAN_ZERO and TILT_ZERO at final install position
- [ ] Consider picking best of 3 radar targets instead of always using target1
- [ ] Add systemd service file for auto-start on boot
- [ ] Mount flex will shift PAN_ZERO slightly — re-calibrate after mount settles
