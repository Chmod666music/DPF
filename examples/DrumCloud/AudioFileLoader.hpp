#pragma once
#include <cstdint>
#include <vector>
#include <string>

struct LoadedAudio
{
    std::vector<float> interleaved; // frames * channels
    uint32_t channels = 0;
    uint32_t sampleRate = 0;
    uint64_t frames = 0;
};

bool loadAudioFileToFloat(const char* path, LoadedAudio& out, std::string* err = nullptr);
