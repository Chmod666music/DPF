# DPF - DISTRHO Plugin Framework
[![makefile](https://github.com/DISTRHO/DPF/actions/workflows/makefile.yml/badge.svg)](https://github.com/DISTRHO/DPF/actions/workflows/makefile.yml)
[![cmake](https://github.com/DISTRHO/DPF/actions/workflows/cmake.yml/badge.svg)](https://github.com/DISTRHO/DPF/actions/workflows/cmake.yml)
[![example-plugins](https://github.com/DISTRHO/DPF/actions/workflows/example-plugins.yml/badge.svg)](https://github.com/DISTRHO/DPF/actions/workflows/example-plugins.yml)

DPF is designed to make development of new plugins an easy and enjoyable task.  
It allows developers to create plugins with custom UIs using a simple C++ API.  
The framework facilitates exporting various different plugin formats from the same code-base.

DPF can build for LADSPA, DSSI, LV2, VST2, VST3 and CLAP formats.  
A JACK/Standalone mode is also available, allowing you to quickly test plugins.

Plugin DSP and UI communication is done via key-value string pairs.  
You send messages from the UI to the DSP side, which is automatically saved in the host when required.  
(You can also store state internally if needed, but this breaks DSSI compatibility).

Getting time information from the host is possible.  
It uses the same format as the JACK Transport API, making porting some code easier.

Provided features and implementation status for specific plugin formats can be seen in [FEATURES.md](FEATURES.md).

## Licensing

DPF is released under ISC, which basically means you can do whatever you want as long as you credit the original authors.  
Some plugin formats may have additional restrictions, see [LICENSING.md](LICENSING.md) for details.


## Help and documentation

Bug reports happen on the [DPF github project](https://github.com/DISTRHO/DPF/issues).

Online documentation is available at [https://distrho.github.io/DPF/](https://distrho.github.io/DPF/).

Online help and discussion about DPF happens in the [DPF github discussions](https://github.com/DISTRHO/DPF/discussions).


## List of plugins made with DPF:

See [this wiki page](https://github.com/DISTRHO/DPF/wiki/Plugins-made-with-DPF) for a list of plugins made with DPF.

Plugin examples are also available in the `example/` folder inside this repo.

## DrumCloud by Fuimadane

DrumCloud is a granular sample instrument for Linux.

Available plugin formats:
- CLAP
- VST3
- LV2
- VST2 (.so)

Tested in:
- Bitwig
- REAPER
- Carla
- Ardour

### Quick Install

1. Go to the latest release:  
   https://github.com/Chmod666music/DPF/releases/tag/v1.7-build-scripts-and-vst2

2. Download the format you want:
   - DrumCloud-Fuimadane-CLAP-Linux-v1.7.zip
   - DrumCloud-Fuimadane-VST3-Linux-v1.7.zip
   - DrumCloud-Fuimadane-LV2-Linux-v1.7.zip
   - DrumCloud-Fuimadane-VST2-Linux-v1.7.zip

3. Extract the zip file.

4. Copy the plugin to the correct Linux folder:
   - CLAP → `~/.clap/`
   - VST3 → `~/.vst3/`
   - LV2 → `~/.lv2/`
   - VST2 → `~/.vst/`

If needed, create the folders first:

```bash
mkdir -p ~/.clap ~/.vst3 ~/.lv2 ~/.vst

5. Rescan plugins in your DAW and look for:
DrumCloud (Fuimadane)

Build From Source

git clone --recursive https://github.com/Chmod666music/DPF.git
cd DPF
git checkout drumcloud-clean-stable
cd examples/DrumCloud

sudo apt install build-essential pkg-config \
  libgl1-mesa-dev libx11-dev libxcursor-dev libxext-dev \
  libxrandr-dev libxinerama-dev libasound2-dev \
  libjack-jackd2-dev

./build.sh
./install.sh

For more detailed build instructions, see:
examples/DrumCloud/BUILD.md