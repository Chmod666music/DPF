#include "AudioFileLoader.hpp"
#include <algorithm>
#include <cctype>

// dr_libs (implementation only here)
#define DR_WAV_IMPLEMENTATION
#include "thirdparty/dr_libs/dr_wav.h"

#define DR_FLAC_IMPLEMENTATION
#include "thirdparty/dr_libs/dr_flac.h"

#define DR_MP3_IMPLEMENTATION
#include "thirdparty/dr_libs/dr_mp3.h"

static std::string lowerExt(const char* path)
{
    std::string s(path ? path : "");
    const auto p = s.find_last_of('.');
    if (p == std::string::npos) return "";
    std::string ext = s.substr(p + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c){ return char(std::tolower(c)); });
    return ext;
}

bool loadAudioFileToFloat(const char* path, LoadedAudio& out, std::string* err)
{
    out = LoadedAudio{};
    if (!path || !path[0])
    {
        if (err) *err = "empty path";
        return false;
    }

    const std::string ext = lowerExt(path);

    if (ext == "wav")
    {
        unsigned int ch = 0, sr = 0;
        drwav_uint64 frames = 0;
        float* data = drwav_open_file_and_read_pcm_frames_f32(path, &ch, &sr, &frames, nullptr);
        if (!data) { if (err) *err = "dr_wav failed"; return false; }
        out.channels = ch; out.sampleRate = sr; out.frames = frames;
        out.interleaved.assign(data, data + size_t(frames * ch));
        drwav_free(data, nullptr);
        return true;
    }

    if (ext == "flac")
    {
        unsigned int ch = 0, sr = 0;
        drflac_uint64 frames = 0;
        float* data = drflac_open_file_and_read_pcm_frames_f32(path, &ch, &sr, &frames, nullptr);
        if (!data) { if (err) *err = "dr_flac failed"; return false; }
        out.channels = ch; out.sampleRate = sr; out.frames = frames;
        out.interleaved.assign(data, data + size_t(frames * ch));
        drflac_free(data, nullptr);
        return true;
    }

    if (ext == "mp3")
    {
        drmp3_config cfg{};
        drmp3_uint64 frames = 0;
        float* data = drmp3_open_file_and_read_pcm_frames_f32(path, &cfg, &frames, nullptr);
        if (!data) { if (err) *err = "dr_mp3 failed"; return false; }
        out.channels = cfg.channels; out.sampleRate = cfg.sampleRate; out.frames = frames;
        out.interleaved.assign(data, data + size_t(frames * cfg.channels));
        drmp3_free(data, nullptr);
        return true;
    }

    if (err) *err = "unsupported type";
    return false;
}
