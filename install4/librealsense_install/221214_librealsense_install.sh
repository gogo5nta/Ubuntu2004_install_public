#!/bin/bash
# update 2022.12.12

# --------------------------------
# *** Remove all Realsense SDK ***
# --------------------------------
# https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
# Uninstalling the Packages
# Remove all RealSense™ SDK-related packages with:
# dpkg -l | grep "realsense" | cut -d " " -f 3 | xargs sudo dpkg --purge

# ----------------------------------------------
# *** Realsense SDK install from source code ***
# ----------------------------------------------
# https://rt-net.jp/humanoid/archives/4815
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

# Install the core packages
sudo apt -y install git cmake libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

# Ubuntu 18/20:
sudo apt -y install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at

# https://rt-net.jp/humanoid/archives/4815

# get source
git clone -b development --depth 1 https://github.com/IntelRealSense/librealsense

# Linux のカーネルバージョンに依存しないSUSB バックエンドを使用しcmake
cd librealsense
mkdir build && cd build
cmake .. -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release
make -j4
sudo make install

# install udev rules
cd ..
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
