#!/bin/bash
# Pi OS setup for FakeLight controller (Raspberry Pi, 64-bit OS)
# Run once on a fresh image before transplanting the controller.
# Tested on: Raspberry Pi OS Bookworm 64-bit

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
add_if_missing "arm_boost=1"

# Disable I2C (not used; frees GPIO pins)
sudo sed -i 's/^dtparam=i2c_arm=on/dtparam=i2c_arm=off/' "$CONFIG"
add_if_missing "dtparam=i2c_arm=off"

echo ""
echo "=== Done ==="
echo "Expected devices after reboot:"
echo "  /dev/ttyUSB0  — Enttec Open DMX (FTDI)"
echo "  /dev/ttyS0    — RD03D mmWave radar (UART)"
echo ""
echo "Reboot now, then run: sudo python3 fakelight.py"
