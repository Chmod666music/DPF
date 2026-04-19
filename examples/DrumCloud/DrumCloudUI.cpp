#define DRUMCLOUD_UI_DEBUG 0

#include "DistrhoUI.hpp"
#include "AudioFileLoader.hpp"
#include "DrumCloudParams.hpp"


#include <cstring>
#include <cstdio>

#include <cstdint>
#include <vector>
#include <algorithm>
#include <string>
#include <cstdlib>
#include <cmath>
#include <GL/gl.h>

namespace DISTRHO {


// ------------------------------------------------------------
// UI-local sample path cache helpers (Bitwig-safe persistence)

static constexpr uint32_t kMax24 = 0xFFFFFFu;


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


static std::string getCachePath()
{
    const char* home = std::getenv("HOME");
    if (!home)
        return "/tmp/drumcloud-sample-cache.txt";

    return std::string(home) + "/.config/drumcloud-sample-cache.txt";
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


class DrumCloudUI : public UI
{
public:
    DrumCloudUI()
        : UI(360, 90)
    {
    }


protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;
    void onDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    


private:
    // ---- waveform preview (NUVÆRENDE) ----
    static constexpr int kWavePreviewSize = 1024;
    float fWaveMin[kWavePreviewSize]{};
    float fWaveMax[kWavePreviewSize]{};
    bool  fWaveValid = false;
    float fScanPosUI = 0.0f;


    // ---- Release UI ----
    float fReleaseMsUi = 250.0f;
    bool  fDragRelease = false;
    float fDragStartX = 0.0f;
    float fDragStartMs = 250.0f;
    bool fDragStartPos = false;


    // ---- Start/Spread UI ----
    float fStartPosUi = 0.0f;   // 0..1
    float fSpreadUi   = 0.0f;   // 0..1

    // ---- sample/persist ----
    float fSamplePing = 0.0f;
    std::string fSamplePath;
    bool fRestoringFromParam = false;

    bool loadWavePreviewFromAudioFile(const char* path);
    bool loadWavePreviewFromWav(const char* path);   // compatibility wrapper


    // ---- waveform preview loader ----
    
};
bool DrumCloudUI::loadWavePreviewFromAudioFile(const char* path)
{
    // reset
    std::fill_n(fWaveMin, kWavePreviewSize, 0.0f);
    std::fill_n(fWaveMax, kWavePreviewSize, 0.0f);
    fWaveValid = false;

    if (path == nullptr || path[0] == '\0')
        return false;

    LoadedAudio a;
    std::string err;
    const bool ok = loadAudioFileToFloat(path, a, &err);

    if (!ok || a.channels == 0 || a.frames == 0 || a.interleaved.empty())
    {
        return false;
    }

    const uint32_t ch = a.channels;
    const uint64_t frames = a.frames;
    const float* data = a.interleaved.data();

    // downsample: map frames -> kWavePreviewSize buckets
    const uint64_t step = std::max<uint64_t>(1u, frames / (uint64_t)kWavePreviewSize);

    for (int i = 0; i < kWavePreviewSize; ++i)
    {
        const uint64_t start = (uint64_t)i * step;
        const uint64_t end   = std::min<uint64_t>(start + step, frames);

        float mn =  1.0f;
        float mx = -1.0f;

        for (uint64_t f = start; f < end; ++f)
        {
            // pick channel with max abs, keep sign (similar to earlier)
            float v = 0.0f;

            if (ch == 1)
            {
                v = data[f];
            }
            else
            {
                const uint64_t base = f * (uint64_t)ch;
                float best = data[base];
                float bestAbs = std::fabs(best);

                for (uint32_t c = 1; c < ch; ++c)
                {
                    const float s = data[base + c];
                    const float ab = std::fabs(s);
                    if (ab > bestAbs)
                    {
                        bestAbs = ab;
                        best = s;
                    }
                }
                v = best;
            }

            mn = std::min(mn, v);
            mx = std::max(mx, v);
        }

        // clamp
        if (mn < -1.0f) mn = -1.0f;
        if (mx >  1.0f) mx =  1.0f;

        fWaveMin[i] = mn;
        fWaveMax[i] = mx;
    }

    fWaveValid = true;
    return true;
}


 // <-- vi erstatter denne senere

void DrumCloudUI::onDisplay()
{
    const float W = (float)getWidth();
    const float H = (float)getHeight();
    const float mid = H * 0.5f;

    // ---- background ----
    glDisable(GL_TEXTURE_2D);
    glClearColor(0.06f, 0.06f, 0.07f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    static int sTop = 0;
if ((sTop++ % 60) == 0)
{
    FILE* f = std::fopen("/tmp/drumcloud-ui-scan.log", "a");
    if (f)
    {
        std::fprintf(f, "onDisplay() fWaveValid=%d\n", (int)fWaveValid);
        std::fclose(f);
    }
}


    // ---- midline (always) ----
    glLineWidth(1.0f);
    glColor4f(0.25f, 0.25f, 0.28f, 1.0f);
    glBegin(GL_LINES);
        glVertex2f(12.0f, H - mid);
        glVertex2f(W - 12.0f, H - mid);

    glEnd();

    


    // ---- if we have waveform, draw it ----
    if (fWaveValid)
{
    // --- layout ---
    const float pad = 12.0f;
    const float x0  = pad;
    const float x1  = W - pad;
    const float y0  = pad;
    const float y1  = H - pad;

    // --- scan position ---
    const float scanPos = std::clamp(fScanPosUI, 0.0f, 1.0f);
    const float scanX   = x0 + scanPos * (x1 - x0);

    // DEBUG til fil (hver 30. repaint)
    static int sCount = 0;
    if ((sCount++ % 30) == 0)
    {
        FILE* f = std::fopen("/tmp/drumcloud-ui-scan.log", "a");
        if (f)
        {
            std::fprintf(f, "scanPos=%f scanX=%f fWaveValid=%d\n",
                         scanPos, scanX, (int)fWaveValid);
            std::fclose(f);
        }
    }

    
    // --- waveform ---
glLineWidth(1.0f);
glColor4f(0.85f, 0.9f, 1.0f, 1.0f);
glBegin(GL_LINES);

const float yTop = pad;
const float yBot = H - pad;
const float midY = 0.5f * (yTop + yBot);
const float ampY = 0.5f * (yBot - yTop);

for (int i = 0; i < kWavePreviewSize; ++i)
{
    const float t = (kWavePreviewSize > 1)
        ? (float)i / (float)(kWavePreviewSize - 1)
        : 0.0f;

    const float x = x0 + t * (x1 - x0);

    // fWaveMin/Max forventes at være i -1..+1
    const float mn = std::clamp(fWaveMin[i], -1.0f, 1.0f);
    const float mx = std::clamp(fWaveMax[i], -1.0f, 1.0f);

    const float y1n = midY + mn * ampY;
    const float y2n = midY + mx * ampY;

    // Vi bruger samme "H - y" convention som resten af din UI
    glVertex2f(x, H - y1n);
    glVertex2f(x, H - y2n);
}
glEnd();


    // --- scan playhead (thin) ---
    glLineWidth(2.0f);
    glColor4f(0.1f, 0.95f, 0.3f, 0.9f);
    glBegin(GL_LINES);
        glVertex2f(scanX, y0);
        glVertex2f(scanX, y1);
    glEnd();

    // ---- overlays: Start Position + Spread + glow playhead ----
    {
        // Start position line
        const float startX = x0 + fStartPosUi * (x1 - x0);

        // Spread band around start
        const float halfW = 0.5f * fSpreadUi * (x1 - x0);
        const float sx0 = std::max(x0, startX - halfW);
        const float sx1 = std::min(x1, startX + halfW);

        // Spread fill (subtle)
        if (fSpreadUi > 0.0001f)
        {
            glColor4f(0.25f, 0.25f, 0.35f, 0.25f);
            glBegin(GL_QUADS);
                glVertex2f(sx0, y1);
                glVertex2f(sx1, y1);
                glVertex2f(sx1, y0);
                glVertex2f(sx0, y0);
            glEnd();
        }

        // Glow playhead (fat, transparent)
        glLineWidth(6.0f);
        glColor4f(0.1f, 0.95f, 0.3f, 0.15f);
        glBegin(GL_LINES);
            glVertex2f(scanX, y0);
            glVertex2f(scanX, y1);
        glEnd();
        glLineWidth(1.0f);
    } // overlays-scope
} // if (fWaveValid)
else
{
    // placeholder...
}


    


    // ---- Release slider (simple bar) ----
{
    const float minMs = 5.0f;
    const float maxMs = 5000.0f;

    float t = 0.0f;
    if (maxMs > minMs)
        t = (fReleaseMsUi - minMs) / (maxMs - minMs);

    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // placering (bunden af UI)
    const float x = 10.0f;
    const float y = 72.0f;
    const float w = 160.0f;
    const float h = 12.0f;

    // baggrund
    glColor4f(0.15f, 0.15f, 0.18f, 1.0f);
    glBegin(GL_QUADS);
        glVertex2f(x,     H - y);
        glVertex2f(x + w, H - y);
        glVertex2f(x + w, H - (y + h));
        glVertex2f(x,     H - (y + h));
    glEnd();

    // fyld (værdi)
    glColor4f(0.45f, 0.75f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
        glVertex2f(x,           H - y);
        glVertex2f(x + w * t,   H - y);
        glVertex2f(x + w * t,   H - (y + h));
        glVertex2f(x,           H - (y + h));
    glEnd();

    // outline
    glColor4f(0.35f, 0.35f, 0.40f, 1.0f);
    glBegin(GL_LINE_LOOP);
        glVertex2f(x,     H - y);
        glVertex2f(x + w, H - y);
        glVertex2f(x + w, H - (y + h));
        glVertex2f(x,     H - (y + h));
    glEnd();
}

}


    bool DrumCloudUI::onMotion(const MotionEvent& ev)
{
    const float mx = (float)ev.pos.getX();

    // ---- Start Position drag ----
    if (fDragStartPos)
    {
        const float pad = 12.0f;
        const float wx0 = pad;
        const float wx1 = (float)getWidth() - pad;

        float norm = 0.0f;
        if (wx1 > wx0)
            norm = (mx - wx0) / (wx1 - wx0);

        norm = std::clamp(norm, 0.0f, 1.0f);

        // ✅ dead zone near zero
        if (norm < 0.005f)
            norm = 0.0f;

        fStartPosUi = norm;

        editParameter(paramStartPosition, true);
        setParameterValue(paramStartPosition, norm);
        editParameter(paramStartPosition, false);

        repaint();
        return true;
    }

    // ---- Release drag (din eksisterende) ----
    if (!fDragRelease)
        return false;

    const float minMs = 5.0f;
    const float maxMs = 5000.0f;
    const float w = 160.0f;

    const float dx = mx - fDragStartX;

    float t0 = 0.0f;
    if (maxMs > minMs)
        t0 = (fDragStartMs - minMs) / (maxMs - minMs);

    float t = t0 + (dx / w);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    const float newMs = minMs + t * (maxMs - minMs);
    fReleaseMsUi = newMs;

    editParameter(paramReleaseMs, true);
    setParameterValue(paramReleaseMs, newMs);
    editParameter(paramReleaseMs, false);

    repaint();
    return true;
}


    bool DrumCloudUI::onMouse(const MouseEvent& ev)
{
    // Release slider rect
    const float rx = 10.0f;
    const float ry = 72.0f;
    const float rw = 160.0f;
    const float rh = 12.0f;

    const float mx = (float)ev.pos.getX();
    const float my = (float)ev.pos.getY();

    const float H  = (float)getHeight();
    const float rry0 = H - ry;
    const float rry1 = H - (ry + rh);

    const bool hitRelease =
        (mx >= rx && mx <= rx + rw) &&
        (my >= rry1 && my <= rry0);

    // Waveform rect
    const float pad = 12.0f;
    const float wx0 = pad;
    const float wx1 = (float)getWidth() - pad;
    const float wy0 = pad;
    const float wy1 = H - pad;

    const bool hitWave =
        (mx >= wx0 && mx <= wx1) &&
        (my >= wy0 && my <= wy1);

    // Start Position overlay zone: nederste del af waveform-området
    const float startZoneH = 16.0f; // px (justér gerne)
    const bool hitStartPosZone =
        hitWave && (my >= (wy1 - startZoneH) && my <= wy1);

    if (ev.button == 1 && ev.press)
    {
        // 1) Release drag
        if (hitRelease)
        {
            fDragRelease = true;
            fDragStartX  = mx;
            fDragStartMs = fReleaseMsUi;

            const float minMs = 5.0f;
            const float maxMs = 5000.0f;

            float t = (mx - rx) / rw;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;

            const float newMs = minMs + t * (maxMs - minMs);
            fReleaseMsUi = newMs;

            editParameter(paramReleaseMs, true);
            setParameterValue(paramReleaseMs, newMs);
            editParameter(paramReleaseMs, false);

            repaint();
            return true;
        }

        // 2) Start Position drag (SKAL ligge før hitWave->load!)
        if (hitStartPosZone)
        {
            fDragStartPos = true;

            float norm = 0.0f;
            if (wx1 > wx0)
                norm = (mx - wx0) / (wx1 - wx0);

            norm = std::clamp(norm, 0.0f, 1.0f);

            // dead zone near zero
            if (norm < 0.005f)
                norm = 0.0f;

            fStartPosUi = norm;

            editParameter(paramStartPosition, true);
            setParameterValue(paramStartPosition, norm);
            editParameter(paramStartPosition, false);

            repaint();
            return true;
        }

        // 3) Click in waveform area => load sample
        if (hitWave)
        {
            requestStateFile("samplePath");
            return true;
        }

        return false;
    }

    if (ev.button == 1 && !ev.press)
    {
        if (fDragRelease)
        {
            fDragRelease = false;
            return true;
        }

        if (fDragStartPos)
        {
            fDragStartPos = false;
            return true;
        }
    }

    return false;
}


    


void DrumCloudUI::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "samplePath") == 0)
    {
        const std::string newPath = value ? value : "";

        // undgå spam/loop hvis samme path kommer igen
        if (newPath == fSamplePath)
            return;

        fSamplePath = newPath;

        // UI preview (AudioFileLoader - alle formater du understøtter)
        fWaveValid = false;
        if (!fSamplePath.empty())
        {
            fWaveValid = loadWavePreviewFromAudioFile(fSamplePath.c_str());
        }

        // Ping host så projektet gemmer state (hidden param trick)
        // ❗ MEN: ikke under restore
        if (!fRestoringFromParam)
        {
            fSamplePing = (fSamplePing < 0.5f) ? 1.0f : 0.0f;
            editParameter(paramSamplePath, true);
            setParameterValue(paramSamplePath, fSamplePing);
            editParameter(paramSamplePath, false);
        }

        repaint();
        return;
    }

    UI::stateChanged(key, value);
}


void DrumCloudUI::parameterChanged(uint32_t index, float value)
{
    if (index == paramSamplePath)
    {
        const uint32_t id = norm24ToId(value) & 0xFFFFFFu;
        if (id == 0)
            return;

        std::string p;
        const bool ok = cacheReadNearby(id, p);

        if (ok && !p.empty())
        {
            fRestoringFromParam = true;

            fWaveValid = loadWavePreviewFromAudioFile(p.c_str());
            repaint();

            setState("samplePath", p.c_str()); // push to DSP/state

            fRestoringFromParam = false;
        }
        return;
    }

    // --- Release ---
    if (index == paramReleaseMs)
    {
        fReleaseMsUi = value;  // ms (eller norm, alt efter din UI mapping)
        repaint();
        return;
    }

    // --- Start Position ---
    if (index == paramStartPosition)
    {
        fStartPosUi = value;   // 0..1
        repaint();
        return;
    }

    // --- Position Spread ---
    if (index == paramPositionSpread)
    {
        fSpreadUi = value;     // 0..1
        repaint();
        return;
    }
    if (index == paramScanPos)
    {
    fScanPosUI = value;     // 0..1
    repaint();              // sørg for redraw
    return;
    }
    


    // (evt. andre parametre...)
}


bool DrumCloudUI::loadWavePreviewFromWav(const char* path)
{
    return loadWavePreviewFromAudioFile(path);
}


UI* createUI()
{
    return new DrumCloudUI();
}

} // namespace DISTRHO
