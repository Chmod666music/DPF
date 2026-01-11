#include "DistrhoUI.hpp"

#include <cstring>
#include <cstdio>
#include <cstdarg>

#include <cstdint>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <cmath>
#include <GL/gl.h>

#include "DrumCloudParams.hpp"

// ------------------------------------------------------------
// UI debug logger
// ------------------------------------------------------------
static void uiLog(const char* fmt, ...)
{
    FILE* f = std::fopen("/tmp/drumcloud-ui.log", "a");
    if (!f) return;

    va_list args;
    va_start(args, fmt);
    std::vfprintf(f, fmt, args);
    std::fprintf(f, "\n");
    va_end(args);

    std::fclose(f);
}



// ------------------------------------------------------------
// UI-local sample path cache helpers (Bitwig-safe persistence)

static constexpr uint32_t kMax24 = 0xFFFFFFu;

static float idToNorm24(uint32_t id)
{
    id &= kMax24;
    // encode to the *center* of the bucket to avoid rounding drift
    return (float(id) + 0.5f) / float(kMax24);
}

static uint32_t norm24ToId(float v)
{
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return kMax24;

    // inverse of idToNorm24: v = (id + 0.5) / kMax24
    // => id ≈ v*kMax24 - 0.5
    const float f = v * float(kMax24) - 0.5f;

    // round to nearest integer id
    const uint32_t id = (uint32_t)std::lround(f) & kMax24;
    return id;
}



static uint32_t sampleIdFromPath(const char* path)
{
    if (path == nullptr || path[0] == '\0')
        return 0;

    uint32_t h = 2166136261u; // FNV-1a
    for (const unsigned char* p = (const unsigned char*)path; *p; ++p)
    {
        h ^= (uint32_t)(*p);
        h *= 16777619u;
    }

    h &= 0x00FFFFFFu; // 24-bit
    if (h == 0) h = 1;
    return h;
}


static uint32_t fnv1a32(const char* s)
{
    uint32_t h = 2166136261u;
    for (const unsigned char* p = (const unsigned char*)s; *p; ++p)
    {
        h ^= (uint32_t)(*p);
        h *= 16777619u;
    }
    if (h == 0) h = 1;
    return h;
}



static std::string getCachePath()
{
    const char* home = std::getenv("HOME");
    if (!home)
        return "/tmp/drumcloud-sample-cache.txt";

    return std::string(home) + "/.config/drumcloud-sample-cache.txt";
}

static void cacheWrite(uint32_t id, const std::string& path)
{
    const std::string fn = getCachePath();
    std::FILE* fp = std::fopen(fn.c_str(), "a");
    if (!fp) return;

    std::fprintf(fp, "%u\t%s\n", (unsigned)id, path.c_str());
    std::fclose(fp);
}
static bool cacheRead(uint32_t id, std::string& outPath)
{
    outPath.clear();
    if (id == 0) return false;

    const std::string fn = getCachePath();
    FILE* fp = std::fopen(fn.c_str(), "r");
    if (!fp) return false;

    char line[4096];
    while (std::fgets(line, sizeof(line), fp))
    {
        unsigned rid = 0;
        char pbuf[4096] = {};

        if (std::sscanf(line, "%u\t%4095[^\n]", &rid, pbuf) == 2)
        {
            if ((uint32_t)rid == id)
            {
                outPath = pbuf;
                std::fclose(fp);
                return true;
            }
        }
    }

    std::fclose(fp);
    return false;
}
static bool cacheReadNearby(uint32_t id, std::string& outPath)
{
    // prøv først præcis id
    if (cacheRead(id, outPath) && !outPath.empty())
        return true;

    // prøv nabo-id'er (typisk float/round drift)
    if (id > 0 && cacheRead(id - 1u, outPath) && !outPath.empty())
        return true;

    if (id < 0xFFFFFFu && cacheRead(id + 1u, outPath) && !outPath.empty())
        return true;

    return false;
}



namespace DISTRHO {


class DrumCloudUI : public UI
{
public:
    DrumCloudUI()
        : UI(360, 90)
    {
        uiLog("----- UI START -----");
    }

    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

protected:
    void onDisplay() override;
    bool onMouse(const MouseEvent& ev) override;

    // ---- data ----
    static constexpr int kWavePreviewSize = 1024;
    float fWaveMin[kWavePreviewSize]{};
    float fWaveMax[kWavePreviewSize]{};
    bool  fWaveValid = false;

    float fSamplePing = 0.0f;
    std::string fSamplePath;
    bool fRestoringFromParam = false;

    bool loadWavePreviewFromWav(const char* path);

};

void DrumCloudUI::onDisplay()
{
    const float W = (float)getWidth();
    const float H = (float)getHeight();
    const float mid = H * 0.5f;

    // ---- background ----
    glDisable(GL_TEXTURE_2D);
    glClearColor(0.06f, 0.06f, 0.07f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // ---- midline (always) ----
    glLineWidth(1.0f);
    glColor4f(0.25f, 0.25f, 0.28f, 1.0f);
    glBegin(GL_LINES);
        glVertex2f(0.0f, H - mid);
        glVertex2f(W,   H - mid);
    glEnd();

    // ---- if we have waveform, draw it ----
    if (fWaveValid)
    {
        glLineWidth(1.0f);
        glColor4f(0.85f, 0.9f, 1.0f, 1.0f);
        glBegin(GL_LINES);

        for (int i = 0; i < kWavePreviewSize; ++i)
        {
            const float x  = (float)i / (kWavePreviewSize - 1) * W;
            const float y1 = mid - fWaveMin[i] * mid;
            const float y2 = mid - fWaveMax[i] * mid;

            const float gy1 = H - y1;
            const float gy2 = H - y2;

            glVertex2f(x, gy1);
            glVertex2f(x, gy2);
        }

        glEnd();
        return;
    }

    // ---- placeholder (no waveform yet) ----
    glColor4f(0.15f, 0.15f, 0.18f, 1.0f);
    glBegin(GL_QUADS);
        glVertex2f(12.0f, H - 12.0f);
        glVertex2f(W - 12.0f, H - 12.0f);
        glVertex2f(W - 12.0f, 12.0f);
        glVertex2f(12.0f, 12.0f);
    glEnd();

    glColor4f(0.7f, 0.7f, 0.8f, 1.0f);
    glBegin(GL_LINES);
        glVertex2f(W*0.5f - 18.0f, H*0.5f);
        glVertex2f(W*0.5f + 18.0f, H*0.5f);
        glVertex2f(W*0.5f, H*0.5f - 18.0f);
        glVertex2f(W*0.5f, H*0.5f + 18.0f);
    glEnd();
}



    bool DrumCloudUI::onMouse(const MouseEvent& ev)
{
    if (ev.press && ev.button == 1)
    {
        requestStateFile("samplePath");
        return true;
    }
    return false;
}


void DrumCloudUI::stateChanged(const char* key, const char* value)
{
    uiLog("[UI] stateChanged key='%s' value='%s'",
          key, value ? value : "(null)");

    if (std::strcmp(key, "samplePath") != 0)
        return;

    // Cancel / tom sti
    if (value == nullptr || value[0] == '\0')
        return;

    // UI skal vide at der er sample (til + tegnet)
    fSamplePath = value;
    uiLog("[UI] samplePath selected='%s'", value);


    // cache til id->path restore
    uint32_t id = sampleIdFromPath(value);
    id &= 0xFFFFFFu;

    uiLog("[UI] cacheWrite id=%u path='%s'", id, value);
    cacheWrite(id, value);


    editParameter(paramSamplePath, true);
    setParameterValue(paramSamplePath, idToNorm24(id));
    editParameter(paramSamplePath, false);



    // UI preview
    loadWavePreviewFromWav(value);
    repaint();
}






void DrumCloudUI::parameterChanged(uint32_t index, float value)
{
    uiLog("[UI] parameterChanged index=%u value=%f", index, value);

    if (index != paramSamplePath)
        return;

    const uint32_t id = norm24ToId(value) & 0xFFFFFFu;
    uiLog("[UI] paramSamplePath id=%u", id);

    if (id == 0)
        return;

    std::string p;
    const bool ok = cacheReadNearby(id, p);
    uiLog("[UI] cacheReadNearby ok=%d path='%s'", ok, p.c_str());

    if (ok && !p.empty())
    {
        loadWavePreviewFromWav(p.c_str());
        repaint();
    }
    setState("samplePath", p.c_str());   // ✅ fortæl DSP/host igen ved restore

}








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

} // namespace DISTRHO




