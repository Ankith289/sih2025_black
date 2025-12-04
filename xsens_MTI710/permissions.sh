#!/bin/bash

echo ">>> Creating udev rule for Xsens MTi-710..."

sudo bash -c 'cat > /etc/udev/rules.d/99-xsens.rules <<EOF
SUBSYSTEM=="usb", ATTR{idVendor}=="2639", ATTR{idProduct}=="0017", MODE="0666", GROUP="plugdev", SYMLINK+="mti710"
EOF'

echo ">>> Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

echo ">>> Adding user to plugdev group..."
sudo usermod -aG plugdev $USER

echo ">>> Disabling USB autosuspend globally..."
sudo bash -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'

sudo bash -c 'cat > /etc/modprobe.d/usbcore.conf <<EOF
options usbcore autosuspend=-1
EOF'

echo ">>> Reloading systemd and USB settings..."
sudo systemctl daemon-reload

echo ">>> DONE!"
echo "⚠️  IMPORTANT: Unplug & replug the MTi-710"
echo "⚠️  Log out and log in again so plugdev group takes effect."
