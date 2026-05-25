#!/bin/bash
# Pi OS setup for FakeLight controller (Raspberry Pi, 64-bit OS)
# Run once on a fresh image before transplanting the controller.
# Tested on: Raspberry Pi OS Bookworm 64-bit, Pi 3B and Pi 4/CM4

set -e

echo "=== FakeLight Pi Setup ==="

# --- System packages ---
sudo apt update
sudo apt install -y python3-pip python3-serial libftdi1-2

# --- Python packages ---
pip3 install -r "$(dirname "$0")/requirements.txt"

# --- User groups (dialout is required for /dev/ttyUSB0) ---
sudo usermod -aG dialout,plugdev,gpio,spi,i2c "$USER"

# --- /boot/firmware/config.txt tweaks ---
CONFIG=/boot/firmware/config.txt

add_if_missing() {
    grep -qF "$1" "$CONFIG" || echo "$1" | sudo tee -a "$CONFIG" > /dev/null
}

add_if_missing "enable_uart=1"

# Disable I2C (not used; frees GPIO pins)
sudo sed -i 's/^dtparam=i2c_arm=on/dtparam=i2c_arm=off/' "$CONFIG"
add_if_missing "dtparam=i2c_arm=off"

# Pi 3B: full hardware UART is claimed by Bluetooth by default.
# The mini UART (fallback) has a clock-linked baud rate that drifts with CPU
# frequency scaling and corrupts RD03D radar comms. Disable BT to fix this.
# On Pi 4+ this overlay is a no-op.
add_if_missing "dtoverlay=disable-bt"

# Pi 4+ only — safe to skip on Pi 3B (ignored)
PI_MODEL=$(cat /proc/device-tree/model 2>/dev/null || true)
if echo "$PI_MODEL" | grep -qv "Pi 3"; then
    add_if_missing "arm_boost=1"
fi

# On Pi 3B, disabling BT moves the full UART to /dev/ttyAMA0, but fakelight.py
# hardcodes /dev/ttyS0. Symlink so the code works without changes.
if echo "$PI_MODEL" | grep -q "Pi 3"; then
    cat <<'EOF' | sudo tee /etc/udev/rules.d/99-fakelight-uart.rules > /dev/null
KERNEL=="ttyAMA0", SYMLINK+="ttyS0"
EOF
    sudo udevadm control --reload-rules
    echo "NOTE: Pi 3B detected — udev symlink ttyAMA0 -> ttyS0 installed."
fi

# Force audio output to 3.5mm headphone jack (1=headphone, 2=HDMI, 0=auto)
amixer cset numid=3 1 > /dev/null 2>&1 || true
# Persist across reboots via asound config
mkdir -p /home/"$USER"/.config
cat <<'EOF' > /home/"$USER"/.asoundrc
pcm.!default {
    type hw
    card 0
    device 0
}
ctl.!default {
    type hw
    card 0
}
EOF

# Create audio directory for hippievoice.py
mkdir -p "$(dirname "$0")/audio"

echo ""
echo "=== Done ==="
echo "Expected devices after reboot:"
echo "  /dev/ttyUSB0  — Enttec Open DMX (FTDI)"
echo "  /dev/ttyAMA0  — RD03D mmWave radar (Pi 3B, BT disabled)"
echo "  /dev/ttyS0    — RD03D mmWave radar (Pi 4+)"
echo ""
echo "Drop .wav files in $(dirname "$0")/audio/ then:"
echo "  Terminal 1: sudo python3 fakelight.py"
echo "  Terminal 2: python3 hippievoice.py"
