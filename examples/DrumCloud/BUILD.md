# DrumCloud Build Guide

DrumCloud is a granular sample instrument for Linux built with DPF.

This project can build the following plugin formats:

- CLAP
- VST3
- LV2
- VST2 (`.so`)

## Tested Hosts

DrumCloud has been tested on Linux in:

- Bitwig
- REAPER
- Carla

## Build Requirements

You need a Linux system with a working C++ toolchain and common development packages.

Typical packages on Ubuntu / Debian-based systems:

```bash
sudo apt install build-essential pkg-config \
  libgl1-mesa-dev libx11-dev libxcursor-dev libxext-dev \
  libxrandr-dev libxinerama-dev libasound2-dev \
  libjack-jackd2-dev

Repository Layout

This DrumCloud version is built inside the DPF tree.

The main working folder is:

~/Dev/DPF_v1_3_stable/examples/DrumCloud

Quick Build

From the DrumCloud folder:

cd ~/Dev/DPF_v1_3_stable/examples/DrumCloud
./build.sh

This script will:

clean old builds
build DrumCloud
build the LV2 TTL generator
generate LV2 .ttl files automatically

Quick Install

After building:

cd ~/Dev/DPF_v1_3_stable/examples/DrumCloud
./install.sh

This installs the plugin formats to:

~/.clap
~/.vst3
~/.lv2
~/.vst

Manual Build

If you want to build manually instead of using the helper script:

cd ~/Dev/DPF_v1_3_stable/examples/DrumCloud
make clean
make CONFIG=Release

Then generate LV2 TTL files:

cd ~/Dev/DPF_v1_3_stable
make -C utils/lv2-ttl-generator
./utils/generate-ttl.sh

Manual Install

Copy built plugins manually:

cp -av ~/Dev/DPF_v1_3_stable/bin/d_drumcloud.clap ~/.clap/
cp -av ~/Dev/DPF_v1_3_stable/bin/d_drumcloud.vst3 ~/.vst3/
cp -av ~/Dev/DPF_v1_3_stable/bin/d_drumcloud.lv2 ~/.lv2/
cp -av ~/Dev/DPF_v1_3_stable/bin/d_drumcloud-vst.so ~/.vst/

Output Files

Built files are placed in:

~/Dev/DPF_v1_3_stable/bin

Expected outputs include:

d_drumcloud.clap
d_drumcloud.vst3
d_drumcloud.lv2
d_drumcloud-vst.so

Notes

If LV2 does not appear in your host, make sure the LV2 bundle contains:

d_drumcloud.so
manifest.ttl
d_drumcloud.ttl

After installation, rescan plugins in your DAW.

In some hosts, the plugin may need a full rescan after updates.

If a prebuilt version does not work correctly on your Linux system, building locally may improve compatibility.

Known Notes
DrumCloud is Linux-focused at this stage.
Host behavior may vary slightly between Bitwig, REAPER and Carla.
Some hosts may differ in how GUI playhead updates are displayed.
Author

Fuimadane
https://linktr.ee/Fuimadane