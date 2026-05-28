# DrumCloud

<img width="820" height="404" alt="Screenshot_20260528_100702" src="https://github.com/user-attachments/assets/3742a4cb-bf83-4e89-ae4e-b2c9ba1314d9" />


**DrumCloud** is a granular sample instrument for Linux, built with the DISTRHO Plugin Framework (DPF) by Fuimadane. 

It allows you to load your own samples directly from the UI and manipulate them into massive, atmospheric soundscapes using granular synthesis, built-in cloud reverb, and state variable filtering.

## ✨ Features (v1.8)
- **Scalable Vector UI:** A modern dark mode layout with responsive custom knobs.
- **Granular Engine:** Deep control over Density, Grain Size, Pitch, Position, and Spread.
- **Multiple Scan Modes:** Manual hold, continuous scan, jump, and host-sync options.
- **Cloud Reverb (FDN):** Lush built-in reverb for massive atmospheric tails.
- **State Variable Filter (SVF):** Smooth sweepable cutoff (LP to HP) and resonance control.
- **Host Integration:** Perfectly saves and recalls your chosen audio samples within your DAW project.

## 📦 Installation (Pre-built Binaries)

The easiest way to install DrumCloud is to download the pre-built zip files from the [Releases page](https://github.com/Chmod666music/DPF/releases). 

1. Download the ZIP file for your preferred format (CLAP, VST3, LV2, or VST2).
2. Extract the archive.
3. Run the included `install.sh` script, or manually copy the plugin file to your standard Linux plugin folders (`~/.clap`, `~/.vst3`, `~/.lv2`, `~/.vst`).
4. Rescan plugins in your DAW.

*Tested heavily on Linux in Bitwig, REAPER, Ardour, and Carla.*

## 🛠️ Build From Source

If you prefer to compile DrumCloud yourself (or if the pre-built binaries do not match your system architecture), you can easily build it from source.

**1. Install dependencies (Ubuntu/Debian example):**
```bash
sudo apt install build-essential pkg-config \
  libgl1-mesa-dev libx11-dev libxcursor-dev libxext-dev \
  libxrandr-dev libxinerama-dev libasound2-dev \
  libjack-jackd2-dev
```

**2. Clone the repository:**
```bash
git clone --recursive [https://github.com/Chmod666music/DPF.git](https://github.com/Chmod666music/DPF.git)
cd DPF/examples/DrumCloud
```

**3. Build and install:**
```bash
./build.sh
./install.sh
```

## 📝 License & Credits
DrumCloud is developed by [Fuimadane](https://linktr.ee/Fuimadane).
Built using the amazing [DISTRHO Plugin Framework (DPF)](https://github.com/DISTRHO/DPF).
