#!/bin/bash
# update 2021.06.09

# make directry
mkdir spec

# hostnamectl
# https://inno-tech-life.com/dev/linux/kernel-tips/
hostnamectl > spec/hostnamectl.txt

# Check OS ver and Kernel ver
echo "# Check OS ver and Kernel ver"
cat /etc/os-release
cat /etc/os-release > spec/os-release.txt

uname -r
uname -r > spec/uname-r.txt

uname -a
uname -a > spec/uname-a.txt

# Ubuntuのスペックを調査
# https://qiita.com/DaisukeMiyamoto/items/98ef077ddf44b5727c29

# CPU
echo "# CPU" 
cat /proc/cpuinfo
cat /proc/cpuinfo > spec/cpuinfo.txt

# MEM
echo "# MEM"
cat /proc/meminfo
cat /proc/meminfo > spec/memoinfo.txt

# HW
echo "# HW"
lspci
lspci > spec/lspci.txt

# USB
echo "# USB"
lsusb
lsusb > spec/lsusb.txt

# Nvidia-smi
echo "# Nvidia-smi"
nvidia-smi
nvidia-smi > spec/nvidia-smi.txt

# HDD(df)
echo "# HDD(df)"
df -BG
df -BG > spec/df-BG.txt
