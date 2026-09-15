# DrumCloud native: development after v1.8.1

The Linux C++/DPF instrument and DrumCloud JS are separate plugins. Use the native v1.8.1 release tag as the baseline; preserve existing parameter indices and plugin IDs so saved host projects still reopen.

## DrumCloud JS feature reference

The current file in [DrumCloud-ReaPack](https://github.com/Chmod666music/DrumCloud-ReaPack/blob/main/Effects/DrumCloud/DrumCloud_JS.jsfx) declares v0.27. Its source exposes sample start/end, five position modes, per-grain direction, pitch/density/stereo spread, grain attack/release, eight MIDI voices, automatic/manual root handling, delay, and room/hall/shimmer reverb.

## Native order of work

1. Move file decoding and waveform preparation out of the audio callback. Build a bounded decoded buffer, then transfer it at a safe point without blocking, allocating, or freeing in the audio callback. Keep the old sample playing on a failed load. Verify rapid sample changes and project restore in Bitwig and REAPER.
2. Add bounded sample start/end controls. Restrict grain start, snap/zero-cross seeking, playback, and all scan modes to the selected region. Handle short regions without reading out of bounds. Show the range and grains on the waveform.
3. Add selectable grain direction (forward/backward/alternate/random), then pitch spread and stereo spread. Keep defaults matching v1.8.1 so existing sessions sound the same.
4. Extend scan to backward, ping-pong, and random walk. Show scan mode and movement clearly in the GUI.
5. Add root note/fine tune, with optional detection performed outside the audio callback and a confidence display. Manual root must remain intact when detection is uncertain.
6. Add envelope, delay and reverb choices only after the sample and grain path is stable. Design a resizable UI with grouped sample, grain, movement, tone and space controls.

For each stage build at least CLAP and VST3 on Linux, confirm state recall and test long/invalid files plus rapid MIDI. Release binaries only after DAW smoke tests. Do not claim macOS binaries from a Linux build.
