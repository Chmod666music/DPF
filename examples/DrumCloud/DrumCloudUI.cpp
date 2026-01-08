#include "DistrhoUI.hpp"

#include <cstring>
#include <cstdio>
#include <cstdint>
#include <vector>
#include <algorithm>
#include <GL/gl.h>

START_NAMESPACE_DISTRHO

class DrumCloudUI : public UI
{
public:
    DrumCloudUI()
        : UI(360, 90) // størrelse på vinduet
    {
    }
    static constexpr int kWavePreviewSize = 1024;
    float fWaveMin[kWavePreviewSize]{};
    float fWaveMax[kWavePreviewSize]{};
    bool  fWaveValid = false;

    bool loadWavePreviewFromWav(const char* path);

protected:
    void onDisplay() override
{
    if (!fWaveValid)
        return;

    const float W = (float)getWidth();
    const float H = (float)getHeight();
    const float mid = H * 0.5f;

    // DGL / OpenGL simple waveform: vertical lines
    glLineWidth(1.0f);
    glColor4f(0.85f, 0.9f, 1.0f, 1.0f);
    glBegin(GL_LINES);

    for (int i = 0; i < kWavePreviewSize; ++i)
    {
        const float x  = (float)i / (kWavePreviewSize - 1) * W;
        const float y1 = mid - fWaveMin[i] * mid;
        const float y2 = mid - fWaveMax[i] * mid;

        // Convert UI coords to GL coords if needed:
        // In DPF UI, origin is usually top-left in UI helpers,
        // but raw GL uses bottom-left. So flip Y:
        const float gy1 = H - y1;
        const float gy2 = H - y2;

        glVertex2f(x, gy1);
        glVertex2f(x, gy2);
    }

    glEnd();
}



    bool onMouse(const MouseEvent& ev) override
{
    // Venstre klik -> åbn host file dialog for state key "samplePath"
    if (ev.press && ev.button == 1)
    {
        requestStateFile("samplePath");
        return true;
    }
    return false;
}

void stateChanged(const char* key, const char* value) override
{
    if (std::strcmp(key, "samplePath") == 0 && value && value[0] != '\0')
    {
        loadWavePreviewFromWav(value);
        repaint();
        return;
    }

    (void)key;
    (void)value;
}


};
static uint32_t readLE32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t readLE16(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

bool DrumCloudUI::loadWavePreviewFromWav(const char* path)
{
    std::fill_n(fWaveMin, kWavePreviewSize, 0.0f);
    std::fill_n(fWaveMax, kWavePreviewSize, 0.0f);
    fWaveValid = false;

    FILE* fp = std::fopen(path, "rb");
    if (!fp) return false;

    uint8_t hdr[12];
    if (std::fread(hdr, 1, 12, fp) != 12) { std::fclose(fp); return false; }
    if (std::memcmp(hdr, "RIFF", 4) != 0 || std::memcmp(hdr + 8, "WAVE", 4) != 0) {
        std::fclose(fp); return false;
    }

    uint16_t audioFormat = 0, numCh = 0, bits = 0;
    uint32_t dataSize = 0;
    long dataPos = 0;

    while (true) {
        uint8_t ch[8];
        if (std::fread(ch, 1, 8, fp) != 8) break;
        const uint32_t chunkSize = readLE32(ch + 4);

        if (std::memcmp(ch, "fmt ", 4) == 0) {
            std::vector<uint8_t> fmt(chunkSize);
            if (chunkSize > 0 && std::fread(fmt.data(), 1, chunkSize, fp) != chunkSize) { std::fclose(fp); return false; }

            audioFormat = readLE16(fmt.data() + 0);
            numCh       = readLE16(fmt.data() + 2);
            bits        = readLE16(fmt.data() + 14);

        } else if (std::memcmp(ch, "data", 4) == 0) {
            dataPos = std::ftell(fp);
            dataSize = chunkSize;
            std::fseek(fp, chunkSize, SEEK_CUR);

        } else {
            std::fseek(fp, chunkSize, SEEK_CUR);
        }

        if (chunkSize & 1) std::fseek(fp, 1, SEEK_CUR);
    }

    if (audioFormat != 1 || (numCh != 1 && numCh != 2) || bits != 16 || dataPos <= 0 || dataSize == 0) {
        std::fclose(fp); return false;
    }

    const uint32_t bytesPerFrame = (uint32_t)numCh * 2u;
    const uint32_t frameCount = dataSize / bytesPerFrame;
    if (frameCount == 0) { std::fclose(fp); return false; }

    const uint32_t step = std::max(1u, frameCount / (uint32_t)kWavePreviewSize);

    for (int i = 0; i < kWavePreviewSize; ++i) {
        const uint32_t start = (uint32_t)i * step;
        const uint32_t end   = std::min(start + step, frameCount);

        float mn =  1.0f;
        float mx = -1.0f;

        std::fseek(fp, dataPos + (long)(start * bytesPerFrame), SEEK_SET);

        for (uint32_t f = start; f < end; ++f) {
            int16_t sL = 0, sR = 0;
            if (numCh == 1) {
                if (std::fread(&sL, 2, 1, fp) != 1) break;
            } else {
                if (std::fread(&sL, 2, 1, fp) != 1) break;
                if (std::fread(&sR, 2, 1, fp) != 1) break;
                sL = (int16_t)(((int)sL + (int)sR) / 2);
            }

            const float v = (float)sL / 32768.0f;
            mn = std::min(mn, v);
            mx = std::max(mx, v);
        }

        fWaveMin[i] = mn;
        fWaveMax[i] = mx;
    }

    std::fclose(fp);
    fWaveValid = true;
    return true;
}

UI* createUI()
{
    return new DrumCloudUI();
}

END_NAMESPACE_DISTRHO
