DrumCloud
by Fuimadane

DrumCloud is a granular sample instrument for Linux, built with DPF.

This release includes native Linux plugin builds in the following formats:

- CLAP
- VST3
- LV2
- VST2 (.so)

Main features:
- NEW: Fully scalable Vector UI with a modern dark mode layout
- NEW: Built-in Cloud Reverb (FDN) for massive atmospheric tails
- NEW: Smooth State Variable Filter (SVF) with Cutoff and Resonance
- Load your own samples directly from the plugin UI
- Granular playback with multiple scan modes
- Working waveform preview with background artwork
- Sample state is saved and restored by the host
- Tested in Bitwig, REAPER, Ardour and Carla on Linux

Included plugin formats:
- d_drumcloud.clap
- d_drumcloud.vst3
- d_drumcloud.lv2
- d_drumcloud-vst.so

Build and install helpers:
- BUILD.md
- build.sh
- install.sh

Notes:
- LV2 support was tested in REAPER, Ardour and Carla
- CLAP and VST3 were tested in Bitwig and REAPER
- VST2 is included as a legacy Linux `.so` build
- Drag and drop may vary depending on host support
- If a plugin does not appear after installation, rescan plugins in your DAW
- If prebuilt binaries do not work correctly on your Linux system, building locally may improve compatibility

Author:
Fuimadane
https://linktr.ee/Fuimadane
