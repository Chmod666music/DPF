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
#include <chrono>
#include <atomic>
#include <GL/gl.h>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace DISTRHO {

extern std::atomic<float> gDrumCloudUiScanPos;
extern std::atomic<int>   gDrumCloudUiScanMode;

static constexpr uint32_t kMax24 = 0xFFFFFFu;

static uint32_t norm24ToId(float v)
{
    if (v <= 0.0f) return 0;
    if (v >= 1.0f) return kMax24;
    const float f = v * float(kMax24) - 0.5f;
    return (uint32_t)std::lround(f) & kMax24;
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

static std::string trimWhitespace(std::string s)
{
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '\t'))
        s.pop_back();

    std::size_t start = 0;
    while (start < s.size() && (s[start] == ' ' || s[start] == '\t'))
        ++start;

    return s.substr(start);
}

static std::string runCommandCapture(const char* cmd)
{
    if (cmd == nullptr || *cmd == '\0')
        return {};

    FILE* pipe = popen(cmd, "r");
    if (pipe == nullptr)
        return {};

    char buffer[1024];
    std::string out;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
        out += buffer;

    const int rc = pclose(pipe);
    if (rc != 0)
        return {};

    return trimWhitespace(out);
}

static std::string openNativeSampleDialog()
{
    const char* home = std::getenv("HOME");
    const char* startDir = (home != nullptr && *home != '\0') ? home : ".";

    std::string cmd;

    cmd = std::string("kdialog --getopenfilename \"") + startDir +
          "\" '*.wav *.flac *.mp3|Audio files' 2>/dev/null";
    std::string path = runCommandCapture(cmd.c_str());
    if (!path.empty())
        return path;

    cmd = std::string("zenity --file-selection --title='Load sample' --filename='") + startDir +
          "/' --file-filter='Audio files | *.wav *.flac *.mp3' 2>/dev/null";
    path = runCommandCapture(cmd.c_str());
    if (!path.empty())
        return path;

    cmd = std::string("yad --file --title='Load sample' --filename='") + startDir +
          "/' --file-filter='*.wav *.flac *.mp3' 2>/dev/null";
    path = runCommandCapture(cmd.c_str());
    if (!path.empty())
        return path;

    return {};
}

class DrumCloudUI : public UI
{
public:
    DrumCloudUI()
        : UI(760, 300)
    {
        const char* home = std::getenv("HOME");
        if (home != nullptr)
        {
            const std::string devPath = std::string(home) + "/Dev/DPF/examples/DrumCloud/UI/fuimadane_wave_bg.png";
            fWaveBgLoaded = loadWaveBgTexture(devPath.c_str());
        }

        if (!fWaveBgLoaded)
            fWaveBgLoaded = loadWaveBgTexture("UI/fuimadane_wave_bg.png");
    }

    ~DrumCloudUI() override
    {
        freeWaveBgTexture();
    }

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;
    void onDisplay() override;
    void uiIdle() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;

private:
    static constexpr int kWavePreviewSize = 1024;
    float fWaveMin[kWavePreviewSize]{};
    float fWaveMax[kWavePreviewSize]{};
    bool  fWaveValid = false;

    GLuint fWaveBgTex = 0;
    int    fWaveBgTexW = 0;
    int    fWaveBgTexH = 0;
    bool   fWaveBgLoaded = false;

    float fScanPosUI = 0.0f;
    int   fScanModeUi = 0;
    
    // UI Parameter values
    float fVolumeUi = 0.8f;
    float fDensityUi = 0.25f;
    float fReleaseMsUi = 452.5f; 
    float fStartPosUi = 0.0f;
    float fSpreadUi = 0.0f;
    float fScanSpeedUi = 0.12f;
    float fFilterUi = 0.5f;     
    float fResoUi = 0.0f;       
    float fReverbSizeUi = 0.8f; 
    float fReverbMixUi = 0.0f;  
    float fVelToDensityUi = 0.35f;
    float fVelToGrainUi = 0.50f;
    float fPitchRateUi = 1.00f;
    float fJumpRateUi = 1.20f;
    float fJumpAmountUi = 0.18f;
    float fJumpSmoothMsUi = 140.0f;
    float fSyncRateUi = 1.0f;

    // Interaction states
    bool  fDragKnob = false;
    uint32_t fDragKnobParam = 0xffffffffu;
    float fKnobDragStartX = 0.0f;
    float fKnobDragStartValue = 0.0f;
    bool  fDragStartPos = false;
    
    // Hover & Double click
    uint32_t fHoverKnobParam = 0xffffffffu;
    uint32_t fLastClickParam = 0xffffffffu;
    std::chrono::steady_clock::time_point fLastClickTime;

    float fSamplePing = 0.0f;
    std::string fSamplePath;
    bool fRestoringFromParam = false;
    bool fChoosingSample = false;

    bool loadWavePreviewFromAudioFile(const char* path);
    bool loadWaveBgTexture(const char* path);
    void freeWaveBgTexture();
    
    float getParamMin(uint32_t param) const;
    float getParamMax(uint32_t param) const;
    float getParamDef(uint32_t param) const;
    float getParamUiValue(uint32_t param) const;
    void  setParamUiValue(uint32_t param, float value);
    bool  hitKnob(float mx, float my, float cx, float cy, float r) const;
    void  drawStrokeChar(char c, float x, float y, float s) const;
    void  drawStrokeText(const char* txt, float x, float y, float s) const;
    void  drawPixelGlyph(char c, float x, float y, float scale) const;
    void  drawPixelText(const char* txt, float x, float y, float scale) const;
    void  drawModernKnob(float cx, float cy, float r, float value, const char* label, bool isHovered) const;
};

float DrumCloudUI::getParamMin(uint32_t param) const
{
    switch (param) {
        case paramVolume: return 0.0f;
        case paramDensity: return 0.0f;
        case paramRelease: return 5.0f;
        case paramStartPosition: return 0.0f;
        case paramPositionSpread: return 0.0f;
        case paramScanSpeed: return 0.0f;
        case paramFilter: return 0.0f;
        case paramResonance: return 0.0f;
        case paramReverbSize: return 0.0f;
        case paramReverbMix: return 0.0f;
        case paramVelocityToDensity: return 0.0f;
        case paramVelocityToGrainSize: return 0.0f;
        case paramPitchRate: return 0.5f;
        case paramScanJumpRate: return 0.1f;
        case paramScanJumpAmount: return 0.0f;
        case paramScanJumpSmoothMs: return 0.0f;
        case paramSyncRate: return 0.0f;
        default: return 0.0f;
    }
}

float DrumCloudUI::getParamMax(uint32_t param) const
{
    switch (param) {
        case paramVolume: return 1.0f;
        case paramDensity: return 1.0f;
        case paramRelease: return 5000.0f;
        case paramStartPosition: return 1.0f;
        case paramPositionSpread: return 1.0f;
        case paramScanSpeed: return 2.0f;
        case paramFilter: return 1.0f;
        case paramResonance: return 1.0f;
        case paramReverbSize: return 1.0f;
        case paramReverbMix: return 1.0f;
        case paramVelocityToDensity: return 1.0f;
        case paramVelocityToGrainSize: return 1.0f;
        case paramPitchRate: return 2.0f;
        case paramScanJumpRate: return 40.0f;
        case paramScanJumpAmount: return 1.0f;
        case paramScanJumpSmoothMs: return 500.0f;
        case paramSyncRate: return 1.0f;
        default: return 1.0f;
    }
}

float DrumCloudUI::getParamDef(uint32_t param) const
{
    switch (param) {
        case paramVolume: return 1.0f;
        case paramDensity: return 0.72f;
        case paramRelease: return 452.5f;
        case paramStartPosition: return 0.0f;
        case paramPositionSpread: return 0.0f;
        case paramScanSpeed: return 0.5f;
        case paramFilter: return 0.5f;
        case paramResonance: return 0.0f;
        case paramReverbSize: return 0.8f;
        case paramReverbMix: return 0.0f;
        case paramVelocityToDensity: return 0.42f;
        case paramVelocityToGrainSize: return 0.58f;
        case paramPitchRate: return 0.71f;
        case paramScanJumpRate: return 27.0f;
        case paramScanJumpAmount: return 0.69f;
        case paramScanJumpSmoothMs: return 272.0f;
        case paramSyncRate: return 1.0f;
        default: return 0.0f;
    }
}

float DrumCloudUI::getParamUiValue(uint32_t param) const
{
    switch (param) {
        case paramVolume: return fVolumeUi;
        case paramDensity: return fDensityUi;
        case paramRelease: return fReleaseMsUi;
        case paramStartPosition: return fStartPosUi;
        case paramPositionSpread: return fSpreadUi;
        case paramScanSpeed: return fScanSpeedUi;
        case paramFilter: return fFilterUi;
        case paramResonance: return fResoUi;
        case paramReverbSize: return fReverbSizeUi;
        case paramReverbMix: return fReverbMixUi;
        case paramVelocityToDensity: return fVelToDensityUi;
        case paramVelocityToGrainSize: return fVelToGrainUi;
        case paramPitchRate: return fPitchRateUi;
        case paramScanJumpRate: return fJumpRateUi;
        case paramScanJumpAmount: return fJumpAmountUi;
        case paramScanJumpSmoothMs: return fJumpSmoothMsUi;
        case paramSyncRate: return fSyncRateUi;
        default: return 0.0f;
    }
}

void DrumCloudUI::setParamUiValue(uint32_t param, float value)
{
    switch (param) {
        case paramVolume: fVolumeUi = value; break;
        case paramDensity: fDensityUi = value; break;
        case paramRelease: fReleaseMsUi = value; break;
        case paramStartPosition: fStartPosUi = value; break;
        case paramPositionSpread: fSpreadUi = value; break;
        case paramScanSpeed: fScanSpeedUi = value; break;
        case paramFilter: fFilterUi = value; break;
        case paramResonance: fResoUi = value; break;
        case paramReverbSize: fReverbSizeUi = value; break;
        case paramReverbMix: fReverbMixUi = value; break;
        case paramVelocityToDensity: fVelToDensityUi = value; break;
        case paramVelocityToGrainSize: fVelToGrainUi = value; break;
        case paramPitchRate: fPitchRateUi = value; break;
        case paramScanJumpRate: fJumpRateUi = value; break;
        case paramScanJumpAmount: fJumpAmountUi = value; break;
        case paramScanJumpSmoothMs: fJumpSmoothMsUi = value; break;
        case paramSyncRate: fSyncRateUi = value; break;
        default: break;
    }
}

bool DrumCloudUI::hitKnob(float mx, float my, float cx, float cy, float r) const
{
    const float dx = mx - cx;
    const float dy = my - cy;
    return (dx*dx + dy*dy) <= (r*r);
}

static const uint8_t* getPixelGlyphRows(char c)
{
    static const uint8_t SPACE[7] = {0,0,0,0,0,0,0};
    static const uint8_t DOT[7]   = {0,0,0,0,0,0x0C,0x0C};
    static const uint8_t DASH[7]  = {0,0,0,0x1E,0,0,0};
    static const uint8_t COLON[7] = {0,0x0C,0x0C,0,0x0C,0x0C,0};
    static const uint8_t A[7] = {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11};
    static const uint8_t B[7] = {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E};
    static const uint8_t C[7] = {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E};
    static const uint8_t D[7] = {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E};
    static const uint8_t E[7] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F};
    static const uint8_t F[7] = {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10};
    static const uint8_t G[7] = {0x0E,0x11,0x10,0x17,0x11,0x11,0x0E};
    static const uint8_t H[7] = {0x11,0x11,0x11,0x1F,0x11,0x11,0x11};
    static const uint8_t I[7] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x1F};
    static const uint8_t J[7] = {0x07,0x02,0x02,0x02,0x12,0x12,0x0C};
    static const uint8_t K[7] = {0x11,0x12,0x14,0x18,0x14,0x12,0x11};
    static const uint8_t L[7] = {0x10,0x10,0x10,0x10,0x10,0x10,0x1F};
    static const uint8_t M[7] = {0x11,0x1B,0x15,0x15,0x11,0x11,0x11};
    static const uint8_t N[7] = {0x11,0x19,0x15,0x13,0x11,0x11,0x11};
    static const uint8_t O[7] = {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E};
    static const uint8_t P[7] = {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10};
    static const uint8_t Q[7] = {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D};
    static const uint8_t R[7] = {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11};
    static const uint8_t S[7] = {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E};
    static const uint8_t T[7] = {0x1F,0x04,0x04,0x04,0x04,0x04,0x04};
    static const uint8_t U[7] = {0x11,0x11,0x11,0x11,0x11,0x11,0x0E};
    static const uint8_t V[7] = {0x11,0x11,0x11,0x11,0x11,0x0A,0x04};
    static const uint8_t W[7] = {0x11,0x11,0x11,0x15,0x15,0x15,0x0A};
    static const uint8_t X[7] = {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11};
    static const uint8_t Y[7] = {0x11,0x11,0x0A,0x04,0x04,0x04,0x04};
    static const uint8_t Z[7] = {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F};
    static const uint8_t N0[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E};
    static const uint8_t N1[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E};
    static const uint8_t N2[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F};
    static const uint8_t N3[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E};
    static const uint8_t N4[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02};
    static const uint8_t N5[7]={0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E};
    static const uint8_t N6[7]={0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E};
    static const uint8_t N7[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08};
    static const uint8_t N8[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E};
    static const uint8_t N9[7]={0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E};
    switch (std::toupper(static_cast<unsigned char>(c)))
    {
    case 'A': return A; case 'B': return B; case 'C': return C; case 'D': return D;
    case 'E': return E; case 'F': return F; case 'G': return G; case 'H': return H;
    case 'I': return I; case 'J': return J; case 'K': return K; case 'L': return L;
    case 'M': return M; case 'N': return N; case 'O': return O; case 'P': return P;
    case 'Q': return Q; case 'R': return R; case 'S': return S; case 'T': return T;
    case 'U': return U; case 'V': return V; case 'W': return W; case 'X': return X;
    case 'Y': return Y; case 'Z': return Z;
    case '0': return N0; case '1': return N1; case '2': return N2; case '3': return N3;
    case '4': return N4; case '5': return N5; case '6': return N6; case '7': return N7;
    case '8': return N8; case '9': return N9;
    case '.': return DOT; case '-': return DASH; case ':': return COLON; case ' ': return SPACE;
    default: return SPACE;
    }
}

void DrumCloudUI::drawPixelGlyph(char c, float x, float y, float scale) const
{
    const uint8_t* rows = getPixelGlyphRows(c);
    const float px = std::max(1.0f, scale);
    const float py = std::max(1.0f, scale);
    glBegin(GL_QUADS);
    for (int r = 0; r < 7; ++r)
    {
        for (int col = 0; col < 5; ++col)
        {
            if (rows[r] & (1 << (4 - col)))
            {
                const float x0 = x + col * px;
                const float y0 = y + r * py;
                glVertex2f(x0, y0);
                glVertex2f(x0 + px, y0);
                glVertex2f(x0 + px, y0 + py);
                glVertex2f(x0, y0 + py);
            }
        }
    }
    glEnd();
}

void DrumCloudUI::drawPixelText(const char* txt, float x, float y, float scale) const
{
    float pen = x;
    for (const char* p = txt; *p; ++p)
    {
        drawPixelGlyph(*p, pen, y, scale);
        pen += 6.0f * scale;
    }
}

void DrumCloudUI::drawModernKnob(float cx, float cy, float r, float value, const char* label, bool isHovered) const
{
    const float angleStart = -135.0f * (float)M_PI / 180.0f;
    const float angleEnd   =  135.0f * (float)M_PI / 180.0f;
    const float angleVal   = angleStart + (angleEnd - angleStart) * value;

    if (isHovered) glColor4f(0.18f, 0.19f, 0.22f, 1.0f);
    else glColor4f(0.12f, 0.13f, 0.16f, 1.0f);

    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for(int i = 0; i <= 32; ++i) {
        float a = angleStart + (angleEnd - angleStart) * (i / 32.0f);
        glVertex2f(cx + std::cos(a) * r, cy + std::sin(a) * r);
    }
    glEnd();

    if (isHovered) glColor4f(0.35f, 0.95f, 1.0f, 1.0f); 
    else glColor4f(0.15f, 0.80f, 0.95f, 1.0f); 

    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for(int i = 0; i <= 32; ++i) {
        float t = i / 32.0f;
        float a = angleStart + (angleVal - angleStart) * t;
        glVertex2f(cx + std::cos(a) * r, cy + std::sin(a) * r);
    }
    glEnd();

    glColor4f(0.06f, 0.06f, 0.07f, 1.0f); 
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for(int i = 0; i <= 32; ++i) {
        float a = i * ((float)M_PI * 2.0f) / 32.0f;
        glVertex2f(cx + std::cos(a) * (r - 4.5f), cy + std::sin(a) * (r - 4.5f));
    }
    glEnd();

    glColor4f(0.9f, 0.9f, 0.95f, 1.0f);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glVertex2f(cx + std::cos(angleVal) * (r - 6.0f), cy + std::sin(angleVal) * (r - 6.0f));
    glVertex2f(cx + std::cos(angleVal) * r, cy + std::sin(angleVal) * r);
    glEnd();

    if (isHovered) glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    else glColor4f(0.88f, 0.91f, 0.97f, 0.98f);
    drawPixelText(label, cx - (float)std::strlen(label) * 3.3f, cy + r + 10.0f, 1.20f);
}

bool DrumCloudUI::loadWaveBgTexture(const char* path)
{
    int w = 0, h = 0, comp = 0;
    unsigned char* pixels = stbi_load(path, &w, &h, &comp, 4);
    if (!pixels || w <= 0 || h <= 0) return false;

    if (fWaveBgTex != 0) glDeleteTextures(1, &fWaveBgTex);

    glGenTextures(1, &fWaveBgTex);
    glBindTexture(GL_TEXTURE_2D, fWaveBgTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glBindTexture(GL_TEXTURE_2D, 0);

    stbi_image_free(pixels);

    fWaveBgTexW = w;
    fWaveBgTexH = h;
    return true;
}

void DrumCloudUI::freeWaveBgTexture()
{
    if (fWaveBgTex != 0) { glDeleteTextures(1, &fWaveBgTex); fWaveBgTex = 0; }
    fWaveBgTexW = 0;
    fWaveBgTexH = 0;
    fWaveBgLoaded = false;
}

bool DrumCloudUI::loadWavePreviewFromAudioFile(const char* path)
{
    std::fill_n(fWaveMin, kWavePreviewSize, 0.0f);
    std::fill_n(fWaveMax, kWavePreviewSize, 0.0f);
    fWaveValid = false;

    if (path == nullptr || path[0] == '\0') return false;

    LoadedAudio a;
    std::string err;
    const bool ok = loadAudioFileToFloat(path, a, &err);
    if (!ok || a.channels == 0 || a.frames == 0 || a.interleaved.empty()) return false;

    const uint32_t ch = a.channels;
    const uint64_t frames = a.frames;
    const float* data = a.interleaved.data();
    const uint64_t step = std::max<uint64_t>(1u, frames / (uint64_t)kWavePreviewSize);

    for (int i = 0; i < kWavePreviewSize; ++i)
    {
        const uint64_t start = (uint64_t)i * step;
        const uint64_t end   = std::min<uint64_t>(start + step, frames);

        float mn =  1.0f;
        float mx = -1.0f;

        for (uint64_t f = start; f < end; ++f)
        {
            float v = 0.0f;
            if (ch == 1) { v = data[f]; }
            else
            {
                const uint64_t base = f * (uint64_t)ch;
                float best = data[base];
                float bestAbs = std::fabs(best);
                for (uint32_t c = 1; c < ch; ++c)
                {
                    const float s = data[base + c];
                    const float ab = std::fabs(s);
                    if (ab > bestAbs) { bestAbs = ab; best = s; }
                }
                v = best;
            }
            mn = std::min(mn, v);
            mx = std::max(mx, v);
        }

        fWaveMin[i] = std::clamp(mn, -1.0f, 1.0f);
        fWaveMax[i] = std::clamp(mx, -1.0f, 1.0f);
    }

    fWaveValid = true;
    return true;
}

void DrumCloudUI::onDisplay()
{
    const float W = (float)getWidth();
    const float waveTop = 12.0f;
    const float waveBottom = 112.0f;
    const float mid = 0.5f * (waveTop + waveBottom);

    glDisable(GL_TEXTURE_2D);
    glClearColor(0.06f, 0.06f, 0.07f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    glLineWidth(1.0f);
    glColor4f(0.25f, 0.25f, 0.28f, 1.0f);
    glBegin(GL_LINES);
        glVertex2f(12.0f, mid);
        glVertex2f(W - 12.0f, mid);
    glEnd();

    if (fWaveValid)
    {
        const float x0 = 12.0f;
        const float x1 = W - 12.0f;
        const float y0 = waveTop;
        const float y1 = waveBottom;
        const float scanPos = std::clamp(fScanPosUI, 0.0f, 1.0f);
        const float scanX = x0 + scanPos * (x1 - x0);
        const float ampY = 0.5f * (y1 - y0);

        if (fWaveBgLoaded && fWaveBgTex != 0 && fWaveBgTexW > 0 && fWaveBgTexH > 0)
        {
            const float waveW = x1 - x0;
            const float waveH = y1 - y0;
            const float imgAspect = (float)fWaveBgTexW / (float)fWaveBgTexH;

            const float zoomY = 3.0f;
            const float alpha = 0.45f;

            const float drawH = waveH * zoomY;
            const float drawW = drawH * imgAspect;

            const float centerX = x0 + 0.5f * waveW;
            const float centerY = y0 + 0.5f * waveH - 0.08f * drawH;

            const float bx0 = centerX - 0.5f * drawW;
            const float bx1 = centerX + 0.5f * drawW;
            const float by0 = centerY - 0.5f * drawH;
            const float by1 = centerY + 0.5f * drawH;

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, fWaveBgTex);
            glColor4f(1.0f, 1.0f, 1.0f, alpha);
            glBegin(GL_QUADS);
                glTexCoord2f(0.0f, 1.0f); glVertex2f(bx0, by1);
                glTexCoord2f(1.0f, 1.0f); glVertex2f(bx1, by1);
                glTexCoord2f(1.0f, 0.0f); glVertex2f(bx1, by0);
                glTexCoord2f(0.0f, 0.0f); glVertex2f(bx0, by0);
            glEnd();
            glBindTexture(GL_TEXTURE_2D, 0);
            glDisable(GL_TEXTURE_2D);
        }

        glLineWidth(1.0f);
        glColor4f(0.62f, 0.70f, 0.82f, 1.0f);
        glBegin(GL_LINES);
        for (int i = 0; i < kWavePreviewSize; ++i)
        {
            const float t = (kWavePreviewSize > 1) ? (float)i / (float)(kWavePreviewSize - 1) : 0.0f;
            const float x = x0 + t * (x1 - x0);
            glVertex2f(x, mid + std::clamp(fWaveMin[i], -1.0f, 1.0f) * ampY);
            glVertex2f(x, mid + std::clamp(fWaveMax[i], -1.0f, 1.0f) * ampY);
        }
        glEnd();

        const float startX = x0 + fStartPosUi * (x1 - x0);
        const float halfW = 0.5f * fSpreadUi * (x1 - x0);
        const float sx0 = std::max(x0, startX - halfW);
        const float sx1 = std::min(x1, startX + halfW);

        if (fSpreadUi > 0.0001f)
        {
            glBegin(GL_QUADS);
                glColor4f(0.32f, 0.34f, 0.48f, 0.08f);
                glVertex2f(sx0, y0);
                glVertex2f(sx1, y0);
                glColor4f(0.36f, 0.40f, 0.56f, 0.18f);
                glVertex2f(sx1, y1);
                glVertex2f(sx0, y1);
            glEnd();

            glLineWidth(1.0f);
            glColor4f(0.55f, 0.62f, 0.88f, 0.35f);
            glBegin(GL_LINES);
                glVertex2f(sx0, y0); glVertex2f(sx0, y1);
                glVertex2f(sx1, y0); glVertex2f(sx1, y1);
            glEnd();
        }

        const bool showStartMarker = (fScanModeUi <= 0 || fDragStartPos);
        if (showStartMarker)
        {
            glLineWidth(6.0f);
            glColor4f(1.0f, 0.72f, 0.18f, 0.12f);
            glBegin(GL_LINES);
                glVertex2f(startX, y0);
                glVertex2f(startX, y1);
            glEnd();

            glLineWidth(2.0f);
            glColor4f(1.0f, 0.76f, 0.22f, 0.92f);
            glBegin(GL_LINES);
                glVertex2f(startX, y0);
                glVertex2f(startX, y1);
            glEnd();

            glColor4f(1.0f, 0.80f, 0.28f, 0.90f);
            glBegin(GL_TRIANGLES);
                glVertex2f(startX, y0 - 1.0f);
                glVertex2f(startX - 4.0f, y0 - 8.0f);
                glVertex2f(startX + 4.0f, y0 - 8.0f);
            glEnd();
        }

        glLineWidth(6.0f);
        glColor4f(0.20f, 0.85f, 1.0f, 0.18f);
        glBegin(GL_LINES);
            glVertex2f(scanX, y0);
            glVertex2f(scanX, y1);
        glEnd();

        glLineWidth(2.0f);
        glColor4f(0.20f, 0.85f, 1.0f, 0.95f);
        glBegin(GL_LINES);
            glVertex2f(scanX, y0);
            glVertex2f(scanX, y1);
        glEnd();
    }

    // Top Row Knobs (8 knapper)
    const float r = 24.0f;
    const float cy = 146.0f;
    const float cx[8] = { 48.0f, 142.0f, 236.0f, 330.0f, 424.0f, 518.0f, 612.0f, 706.0f };
    const uint32_t params[8] = { paramVolume, paramDensity, paramRelease, paramStartPosition, paramPositionSpread, paramScanSpeed, paramFilter, paramResonance };
    const char* labels[8] = { "VOL", "DENS", "REL", "START", "SPREAD", "SCAN", "FILTER", "RESO" };

    for (int i = 0; i < 8; ++i)
    {
        const float vmin = getParamMin(params[i]);
        const float vmax = getParamMax(params[i]);
        const float val  = getParamUiValue(params[i]);
        const float t = (vmax > vmin) ? std::clamp((val - vmin) / (vmax - vmin), 0.0f, 1.0f) : 0.0f;
        
        bool isHovered = (fHoverKnobParam == params[i]);
        drawModernKnob(cx[i], cy, r, t, labels[i], isHovered);
    }

    // Bottom Row Knobs (9 KNAPPER MED RUMKLANG!)
    const float r2 = 19.0f;
    const float cy2 = 236.0f;
    const float cx2[9] = { 45.0f, 130.0f, 215.0f, 300.0f, 385.0f, 470.0f, 555.0f, 640.0f, 725.0f };
    const uint32_t params2[9] = { paramVelocityToDensity, paramVelocityToGrainSize, paramPitchRate, paramScanJumpRate, paramScanJumpAmount, paramScanJumpSmoothMs, paramSyncRate, paramReverbSize, paramReverbMix };
    const char* labels2[9] = { "V DENS", "V GSIZ", "PITCH", "J RATE", "J AMNT", "J SMTH", "SYNC", "RVB SZ", "RVB MX" };

    for (int i = 0; i < 9; ++i)
    {
        const float vmin = getParamMin(params2[i]);
        const float vmax = getParamMax(params2[i]);
        const float val  = getParamUiValue(params2[i]);
        const float t = (vmax > vmin) ? std::clamp((val - vmin) / (vmax - vmin), 0.0f, 1.0f) : 0.0f;

        bool isHovered = (fHoverKnobParam == params2[i]);
        drawModernKnob(cx2[i], cy2, r2, t, labels2[i], isHovered);
    }

    // Data Boxes
    {
        char scanBuf[24];
        std::snprintf(scanBuf, sizeof(scanBuf), "SCAN %.2f", fScanPosUI);
        const float bx0 = W - 118.0f;
        const float by0 = 46.0f;
        const float bw = 90.0f;
        const float bh = 22.0f;
        glColor4f(0.10f, 0.11f, 0.15f, 0.92f);
        glBegin(GL_QUADS);
            glVertex2f(bx0, by0); glVertex2f(bx0 + bw, by0);
            glVertex2f(bx0 + bw, by0 + bh); glVertex2f(bx0, by0 + bh);
        glEnd();
        glColor4f(0.30f, 0.34f, 0.44f, 0.9f);
        glBegin(GL_LINE_LOOP);
            glVertex2f(bx0, by0); glVertex2f(bx0 + bw, by0);
            glVertex2f(bx0 + bw, by0 + bh); glVertex2f(bx0, by0 + bh);
        glEnd();
        glColor4f(0.88f, 0.91f, 0.97f, 0.98f);
        drawPixelText(scanBuf, bx0 + 8.0f, by0 + 6.0f, 1.35f);
    }

    {
        const char* modeName = "HOLD";
        switch (fScanModeUi)
        {
        case 1: modeName = "SCAN"; break;
        case 2: modeName = "JUMP"; break;
        case 3: modeName = "SYNC"; break;
        default: break;
        }
        char modeBuf[20];
        std::snprintf(modeBuf, sizeof(modeBuf), "%s %d", modeName, fScanModeUi);
        const float bx0 = W - 118.0f;
        const float by0 = 18.0f;
        const float bw = 90.0f;
        const float bh = 22.0f;
        glColor4f(0.10f, 0.11f, 0.15f, 0.92f);
        glBegin(GL_QUADS);
            glVertex2f(bx0, by0); glVertex2f(bx0 + bw, by0);
            glVertex2f(bx0 + bw, by0 + bh); glVertex2f(bx0, by0 + bh);
        glEnd();
        glColor4f(0.30f, 0.34f, 0.44f, 0.9f);
        glBegin(GL_LINE_LOOP);
            glVertex2f(bx0, by0); glVertex2f(bx0 + bw, by0);
            glVertex2f(bx0 + bw, by0 + bh); glVertex2f(bx0, by0 + bh);
        glEnd();
        glColor4f(0.88f, 0.91f, 0.97f, 0.98f);
        drawPixelText(modeBuf, bx0 + 9.0f, by0 + 6.0f, 1.35f);
    }
}

bool DrumCloudUI::onMotion(const MotionEvent& ev)
{
    const float mx = (float)ev.pos.getX();
    const float my = (float)ev.pos.getY();

    if (fDragStartPos)
    {
        const float wx0 = 12.0f;
        const float wx1 = (float)getWidth() - 12.0f;
        float norm = (wx1 > wx0) ? (mx - wx0) / (wx1 - wx0) : 0.0f;
        norm = std::clamp(norm, 0.0f, 1.0f);
        if (norm < 0.005f) norm = 0.0f;
        fStartPosUi = norm;
        setParameterValue(paramStartPosition, norm);
        repaint();
        return true;
    }

    if (fDragKnob)
    {
        const float minV = getParamMin(fDragKnobParam);
        const float maxV = getParamMax(fDragKnobParam);
        const float range = maxV - minV;
        float newValue = fKnobDragStartValue + ((mx - fKnobDragStartX) / 260.0f) * range;
        newValue = std::clamp(newValue, minV, maxV);
        setParamUiValue(fDragKnobParam, newValue);
        setParameterValue(fDragKnobParam, newValue);
        repaint();
        return true;
    }

    uint32_t hoverNow = 0xffffffffu;
    if (!fDragKnob && !fDragStartPos)
    {
        const float cy1 = 146.0f;
        const float cx1[8] = { 48.0f, 142.0f, 236.0f, 330.0f, 424.0f, 518.0f, 612.0f, 706.0f };
        const uint32_t p1[8] = { paramVolume, paramDensity, paramRelease, paramStartPosition, paramPositionSpread, paramScanSpeed, paramFilter, paramResonance };
        for (int i = 0; i < 8; ++i) {
            if (hitKnob(mx, my, cx1[i], cy1, 24.0f)) hoverNow = p1[i];
        }

        const float cy2 = 236.0f;
        const float cx2[9] = { 45.0f, 130.0f, 215.0f, 300.0f, 385.0f, 470.0f, 555.0f, 640.0f, 725.0f };
        const uint32_t p2[9] = { paramVelocityToDensity, paramVelocityToGrainSize, paramPitchRate, paramScanJumpRate, paramScanJumpAmount, paramScanJumpSmoothMs, paramSyncRate, paramReverbSize, paramReverbMix };
        for (int i = 0; i < 9; ++i) {
            if (hitKnob(mx, my, cx2[i], cy2, 19.0f)) hoverNow = p2[i];
        }
    }

    if (fHoverKnobParam != hoverNow)
    {
        fHoverKnobParam = hoverNow;
        repaint();
    }

    return false;
}

void DrumCloudUI::uiIdle()
{
    const float scan = std::clamp(gDrumCloudUiScanPos.load(std::memory_order_relaxed), 0.0f, 1.0f);

    if (std::fabs(scan - fScanPosUI) > 0.0005f)
    {
        fScanPosUI = scan;
        repaint();
    }
}

bool DrumCloudUI::onMouse(const MouseEvent& ev)
{
    const float mx = (float)ev.pos.getX();
    const float my = (float)ev.pos.getY();
    const float wx0 = 12.0f;
    const float wx1 = (float)getWidth() - 12.0f;
    const float wy0 = 12.0f;
    const float wy1 = 112.0f;
    const bool hitWave = (mx >= wx0 && mx <= wx1 && my >= wy0 && my <= wy1);
    const bool hitStartPosZone = hitWave && (my >= (wy1 - 16.0f) && my <= wy1);

    if (ev.button == 1 && ev.press)
    {
        const float modeBx0 = (float)getWidth() - 118.0f;
        const float modeBy0 = 18.0f;
        const float modeBw  = 90.0f;
        const float modeBh  = 22.0f;

        if (mx >= modeBx0 && mx <= modeBx0 + modeBw &&
            my >= modeBy0 && my <= modeBy0 + modeBh)
        {
            const int newMode = (fScanModeUi + 1) % 4;
            fScanModeUi = newMode;
            editParameter(paramScanMode, true);
            setParameterValue(paramScanMode, (float)newMode);
            editParameter(paramScanMode, false);
            repaint();
            return true;
        }
    
        if (hitStartPosZone)
        {
            fDragStartPos = true;
            float norm = (wx1 > wx0) ? (mx - wx0) / (wx1 - wx0) : 0.0f;
            norm = std::clamp(norm, 0.0f, 1.0f);
            if (norm < 0.005f) norm = 0.0f;
            fStartPosUi = norm;
            editParameter(paramStartPosition, true);
            setParameterValue(paramStartPosition, norm);
            repaint();
            return true;
        }

        const float cy = 146.0f;
        const float cx[8] = { 48.0f, 142.0f, 236.0f, 330.0f, 424.0f, 518.0f, 612.0f, 706.0f };
        const uint32_t params[8] = { paramVolume, paramDensity, paramRelease, paramStartPosition, paramPositionSpread, paramScanSpeed, paramFilter, paramResonance };
        for (int i = 0; i < 8; ++i)
        {
            if (hitKnob(mx, my, cx[i], cy, 24.0f))
            {
                auto now = std::chrono::steady_clock::now();
                bool isDoubleClick = (fLastClickParam == params[i]) && 
                    (std::chrono::duration_cast<std::chrono::milliseconds>(now - fLastClickTime).count() < 300);
                
                fLastClickTime = now;
                fLastClickParam = params[i];

                if (isDoubleClick) {
                    float defVal = getParamDef(params[i]);
                    editParameter(params[i], true);
                    setParamUiValue(params[i], defVal);
                    setParameterValue(params[i], defVal);
                    editParameter(params[i], false);
                    repaint();
                } else {
                    fDragKnob = true;
                    fDragKnobParam = params[i];
                    fKnobDragStartX = mx;
                    fKnobDragStartValue = getParamUiValue(params[i]);
                    editParameter(params[i], true);
                }
                return true;
            }
        }

        const float cy2 = 236.0f;
        const float cx2[9] = { 45.0f, 130.0f, 215.0f, 300.0f, 385.0f, 470.0f, 555.0f, 640.0f, 725.0f };
        const uint32_t params2[9] = { paramVelocityToDensity, paramVelocityToGrainSize, paramPitchRate, paramScanJumpRate, paramScanJumpAmount, paramScanJumpSmoothMs, paramSyncRate, paramReverbSize, paramReverbMix };
        for (int i = 0; i < 9; ++i)
        {
            if (hitKnob(mx, my, cx2[i], cy2, 19.0f))
            {
                auto now = std::chrono::steady_clock::now();
                bool isDoubleClick = (fLastClickParam == params2[i]) && 
                    (std::chrono::duration_cast<std::chrono::milliseconds>(now - fLastClickTime).count() < 300);
                
                fLastClickTime = now;
                fLastClickParam = params2[i];

                if (isDoubleClick) {
                    float defVal = getParamDef(params2[i]);
                    editParameter(params2[i], true);
                    setParamUiValue(params2[i], defVal);
                    setParameterValue(params2[i], defVal);
                    editParameter(params2[i], false);
                    repaint();
                } else {
                    fDragKnob = true;
                    fDragKnobParam = params2[i];
                    fKnobDragStartX = mx;
                    fKnobDragStartValue = getParamUiValue(params2[i]);
                    editParameter(params2[i], true);
                }
                return true;
            }
        }

        if (hitWave)
        {
            const std::string chosen = openNativeSampleDialog();
            if (!chosen.empty())
            {
                fSamplePath = chosen;
                fWaveValid = false;
                if (!fSamplePath.empty())
                    fWaveValid = loadWavePreviewFromAudioFile(fSamplePath.c_str());

                fChoosingSample = false;
                setState("samplePath", chosen.c_str());
                repaint();
            }
            else
            {
                fChoosingSample = true;
                requestStateFile("samplePath");
            }
            return true;
        }
        return false;
    }

    if (ev.button == 1 && !ev.press)
    {
        if (fDragKnob)
        {
            editParameter(fDragKnobParam, false);
            fDragKnob = false;
            fDragKnobParam = 0xffffffffu;
            return true;
        }
        if (fDragStartPos)
        {
            editParameter(paramStartPosition, false);
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
        if (newPath.empty() || newPath == fSamplePath)
        {
            fChoosingSample = false;
            return;
        }

        fSamplePath = newPath;
        fWaveValid = false;
        if (!fSamplePath.empty())
            fWaveValid = loadWavePreviewFromAudioFile(fSamplePath.c_str());

        if (fChoosingSample && !fRestoringFromParam)
        {
            setState("samplePath", fSamplePath.c_str());
        }

        fChoosingSample = false;
        repaint();
        return;
    }

    UI::stateChanged(key, value);
}

void DrumCloudUI::parameterChanged(uint32_t index, float value)
{
    if (index == paramVolume) { fVolumeUi = value; repaint(); return; }
    if (index == paramDensity) { fDensityUi = value; repaint(); return; }
    if (index == paramRelease) { fReleaseMsUi = value; repaint(); return; }
    if (index == paramStartPosition) { fStartPosUi = value; repaint(); return; }
    if (index == paramPositionSpread) { fSpreadUi = value; repaint(); return; }
    if (index == paramScanSpeed) { fScanSpeedUi = value; repaint(); return; }
    if (index == paramFilter) { fFilterUi = value; repaint(); return; }
    if (index == paramResonance) { fResoUi = value; repaint(); return; }
    if (index == paramReverbSize) { fReverbSizeUi = value; repaint(); return; }
    if (index == paramReverbMix) { fReverbMixUi = value; repaint(); return; }
    if (index == paramVelocityToDensity) { fVelToDensityUi = value; repaint(); return; }
    if (index == paramVelocityToGrainSize) { fVelToGrainUi = value; repaint(); return; }
    if (index == paramPitchRate) { fPitchRateUi = value; repaint(); return; }
    if (index == paramScanJumpRate) { fJumpRateUi = value; repaint(); return; }
    if (index == paramScanJumpAmount) { fJumpAmountUi = value; repaint(); return; }
    if (index == paramScanJumpSmoothMs) { fJumpSmoothMsUi = value; repaint(); return; }
    if (index == paramScanMode) { fScanModeUi = (int)std::lround(value); repaint(); return; }
    if (index == paramScanPos) { repaint(); return; }
}

UI* createUI()
{
    return new DrumCloudUI();
}

} // namespace DISTRHO