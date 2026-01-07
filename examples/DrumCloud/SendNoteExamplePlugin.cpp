/*
 * DISTRHO Plugin Framework (DPF)
 * Copyright (C) 2012-2024 Filipe Coelho <falktx@falktx.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any purpose with
 * or without fee is hereby granted, provided that the above copyright notice and this
 * permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD
 * TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER
 * IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "DistrhoPlugin.hpp"

#include <cmath>
#include <cstring>
#include <algorithm>
#include <cstdint>
#include <cstdio>

//#define DRUMCLOUD_DEBUG 1

#ifdef DRUMCLOUD_DEBUG
  #define DCLOG(...) std::fprintf(stderr, __VA_ARGS__)
#else
  #define DCLOG(...) do{}while(0)
#endif




START_NAMESPACE_DISTRHO

// -----------------------------------------------------------------------------------------------------------
// Granular v1 (CPU-light) - loop source + snap-to markers (percussion oriented)

static inline uint32_t xorshift32(uint32_t& s) {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
}
static inline float frand01(uint32_t& s) {
    return (xorshift32(s) & 0x00FFFFFFu) / float(0x01000000u); // [0..1)
}

struct Grain
{
    float   pos = 0.0f;
    float   step = 1.0f;
    float   amp = 0.0f;
    int32_t age = 0;
    int32_t dur = 0;
    int32_t delay = 0;
    bool    active = false;
    uint8_t note = 60;
    bool    releasing = false;
    float   rel = 1.0f;      // 1 -> 0
    float   relDec = 0.0f;   // pr sample

};


class GranularEngine
{
public:
    void init(double sampleRate)
{
    sr = (sampleRate > 1.0) ? float(sampleRate) : 48000.0f;

    for (int i = 0; i < kWinSize; ++i) {
        const float ph = float(i) / float(kWinSize - 1);
        win[i] = 0.5f - 0.5f * std::cos(2.0f * float(M_PI) * ph);
    }

    makeTestLoop();
    makeMarkers();

    fDensityNorm = 0.25f;
    updateDensityFromNorm();

    reset(); // ✅ semikolon
}

void reset()
{
    // NOTE state
    std::memset(noteIsHeld, 0, sizeof(noteIsHeld));
    lastNoteOn  = 60;
    lastNoteOff = 0;

    // Tail / spawn state
    fSpawnAcc    = 0.0f;
    fReleaseLeft = 0;

    // Current note (for mono behavior)
    fCurrentNote = 60;
    fCurrentVel  = 100;

    // Active grain bookkeeping
    activeCount = 0;
    for (int i = 0; i < kMaxGrains; ++i) {
        activeIdx[i] = 0;
        grains[i].active = false;   // (lad resten være)
    }

    // Optional: hvis du vil have deterministisk reset
    // fPitchRate = 1.0f;
    // rng = 0x12345678u;
}

    bool isSilent() const
    {
        for (int i = 0; i < kMaxGrains; ++i)
            if (grains[i].active)
                return false;
        return fReleaseLeft <= 0;
    }



    void setVelToGrainSize(float amt) noexcept
{
    if (amt < 0.0f) amt = 0.0f;
    if (amt > 1.0f) amt = 1.0f;
    fVelToGrainSize = amt;
}

void setReleaseMs(float ms)
{
    if (ms < 5.0f) ms = 5.0f;
    if (ms > 5000.0f) ms = 5000.0f;
    fReleaseMs = ms;
}




        
void trigger(int note, int vel)
{
    lastNoteOn = uint8_t(note & 0x7F);
    noteIsHeld[lastNoteOn] = true;

    fCurrentNote = lastNoteOn;
    fCurrentVel  = uint8_t(vel & 0x7F);
    fReleaseLeft = 0;

    // NOTE → PITCH
    const float noteNorm = (float(note) - 60.0f) / 12.0f;
    fPitchRate = std::pow(2.0f, noteNorm);

    // velocity -> density
    const float velNorm   = float(vel) / 127.0f;
    const float velShaped = velNorm * velNorm;
    fDensityNorm = velShaped;
    updateDensityFromNorm();

    // burst ved NOTE-ON
    const float burstWindowSec = 0.060f;
    int burst = int(std::round(fGrainsPerSec * burstWindowSec));
    if (burst < 1) burst = 1;

    for (int i = 0; i < burst; ++i)
        spawnOneGrain(note, vel, 1.0f);
}
 // ✅ VIGTIG: trigger slutter her





    void noteOff(int note)
{
    lastNoteOff = uint8_t(note & 0x7F);
    noteIsHeld[lastNoteOff] = false;

    // tail-spawn (valgfrit om du vil gate den)
    if (lastNoteOff == fCurrentNote)
        fReleaseLeft = int32_t((fReleaseMs * 0.001f) * sr);

    const int32_t relSamples = int32_t((fReleaseMs * 0.001f) * sr);
    const float dec = (relSamples > 1) ? (1.0f / float(relSamples)) : 1.0f;

    DCLOG("[noteOff] note=%d relSamples=%d activeCount=%d\n",
          int(lastNoteOff), int(relSamples), int(activeCount));

    for (int i = 0; i < activeCount; ++i)
    {
        Grain& g = grains[activeIdx[i]];

        if (g.active && g.note == lastNoteOff && !g.releasing)
        {
            g.releasing = true;
            g.rel = 1.0f;
            g.relDec = dec;
        }
    }
}





void process(float* outL, float* outR, uint32_t frames)
{
    if (loopLen <= 0)
        return;

    for (uint32_t f = 0; f < frames; ++f)
    {
        float l = 0.0f;
        float r = 0.0f;

        // ----- continuous spawn (tail) -----
        const bool held = noteIsHeld[fCurrentNote];
        const bool tail = (!held && fReleaseLeft > 0);

        float tailMul = 1.0f;

    if (tail)
{
    const float total = (fReleaseMs * 0.001f) * sr;
    tailMul = (total > 1.0f) ? (float(fReleaseLeft) / total) : 0.0f;
    tailMul = std::clamp(tailMul, 0.0f, 1.0f);

}


        if (held || tail)
        {
            const float gps = fGrainsPerSec * tailMul;
            fSpawnAcc += gps / sr;

            while (fSpawnAcc >= 1.0f)
            {
                fSpawnAcc -= 1.0f;
                spawnOneGrain(fCurrentNote, fCurrentVel, tailMul);
            }
        }

        if (tail)
            --fReleaseLeft;
        // ----- end continuous spawn -----

        int i = 0;
        while (i < activeCount)
        {
            Grain& g = grains[activeIdx[i]];

            if (!g.active)
            {
                swapRemove(i);
                continue;
            }

            if (g.delay > 0)
            {
                --g.delay;
                ++i;
                continue;
            }

            if (g.age >= g.dur)
            {
                g.active = false;
                swapRemove(i);
                continue;
            }

            float sL = 0.0f;
            float sR = 0.0f;
            readLoopLinear(g.pos, sL, sR);

            const float w = windowAt(g.age, g.dur);

            float relGain = 1.0f;
            if (g.releasing)
            {
                g.rel -= g.relDec;
                if (g.rel <= 0.0f)
                {
                    g.active = false;
                    swapRemove(i);
                    continue;
                }
                relGain = g.rel * g.rel; // smooth fade-out curve
            }

            const float vv = (sL + sR) * 0.5f * w * g.amp * relGain;
            l += vv;
            r += vv;

            g.pos = wrapPos(g.pos + g.step);
            ++g.age;

            ++i;
        }

        outL[f] += l;
        outR[f] += r;
    }
}


private:
void updateDensityFromNorm()
{
    const float minD = 2.0f;
    const float maxD = 80.0f;

    const float n = (fDensityNorm < 0.0f) ? 0.0f : (fDensityNorm > 1.0f ? 1.0f : fDensityNorm);
    fGrainsPerSec = minD + (maxD - minD) * n;

    if (fGrainsPerSec < 0.001f)
        fGrainsPerSec = 0.001f;
}

    void swapRemove(int idx)
    {
        const int last = activeCount - 1;
        if (idx != last)
            activeIdx[idx] = activeIdx[last];
        activeCount = std::max(0, last);
    }

    float wrapPos(float p) const
    {
        const float len = float(loopLen);
        while (p < 0.0f) p += len;
        while (p >= len) p -= len;
        return p;
    }

    void readLoopLinear(float pos, float& oL, float& oR) const
    {
        const int32_t i0 = int32_t(pos);
        const float frac = pos - float(i0);

        int32_t i1 = i0 + 1;
        if (i1 >= loopLen) i1 -= loopLen;

        const float aL = loopL[i0];
        const float bL = loopL[i1];
        const float aR = loopR[i0];
        const float bR = loopR[i1];

        oL = aL + (bL - aL) * frac;
        oR = aR + (bR - aR) * frac;
    }

    float windowAt(int32_t age, int32_t dur) const
    {
        if (dur <= 1) return 0.0f;
        const float ph = float(age) / float(dur - 1);
        const float x  = ph * float(kWinSize - 1);
        const int i0 = int(x);
        const int i1 = std::min(i0 + 1, kWinSize - 1);
        const float frac = x - float(i0);
        return win[i0] + (win[i1] - win[i0]) * frac;
    }

    int32_t snapBackward(int32_t raw, int32_t radius) const
    {
        if (markerCount <= 0) return raw;

        int lo = 0, hi = markerCount;
        while (lo < hi) {
            const int mid = (lo + hi) >> 1;
            if (markers[mid] <= raw) lo = mid + 1;
            else hi = mid;
        }
        int idx = lo - 1;
        if (idx < 0) idx = 0;

        const int32_t best = markers[idx];
        return (std::abs(best - raw) <= radius) ? best : raw;
    }

    void makeMarkers()
    {
        markerCount = 0;
        if (loopLen <= 0) return;

        const float a = 0.01f;
        float envPrev = 0.0f;
        float env = 0.0f;

        const int32_t minDist = int32_t(sr * 0.030f);
        int32_t lastMarker = -minDist;

        float avg = 0.0f;
        const float avgA = 0.0015f;

        float currOn = 0.0f;

        for (int32_t i = 0; i < loopLen; ++i)
        {
            const float m = 0.5f * (loopL[i] + loopR[i]);
            const float absM = std::fabs(m);

            env += a * (absM - env);

            const float d = env - envPrev;
            envPrev = env;

            const float on = (d > 0.0f) ? d : 0.0f;

            avg += avgA * (on - avg);
            currOn = on;

            if (i < 2) continue;

            const float thr = avg * 6.0f;

            if (currOn > thr && (i - lastMarker) >= minDist)
            {
                if (markerCount < kMaxMarkers)
                {
                    markers[markerCount++] = i;
                    lastMarker = i;
                }
            }
        }

        if (markerCount == 0)
        {
            const int32_t step = int32_t(sr * 0.125f);
            for (int32_t p = 0; p < loopLen && markerCount < kMaxMarkers; p += step)
                markers[markerCount++] = p;
        }
    }

    void makeTestLoop()
    {
        loopLen = int32_t(sr * 2.0f);
        if (loopLen > kMaxLoop) loopLen = kMaxLoop;

        for (int i = 0; i < loopLen; ++i) {
            loopL[i] = 0.0f;
            loopR[i] = 0.0f;
        }

        auto addPulse = [&](int32_t start, float freq, float decayMs, float amp)
        {
            const int32_t N = std::min(loopLen - start, int32_t(sr * 0.30f));
            const float decay = std::exp(-1.0f / (sr * (decayMs / 1000.0f)));
            float env = 1.0f;
            float ph = 0.0f;
            const float inc = 2.0f * float(M_PI) * freq / sr;

            for (int32_t n = 0; n < N; ++n) {
                const float s = std::sin(ph) * env * amp;
                loopL[start + n] += s;
                loopR[start + n] += s;
                ph += inc;
                env *= decay;
            }
        };

        addPulse(0,               55.0f, 80.0f, 0.9f);
        addPulse(int32_t(sr),     55.0f, 80.0f, 0.9f);
        addPulse(int32_t(sr*0.5f), 180.0f, 60.0f, 0.6f);
        addPulse(int32_t(sr*1.5f), 180.0f, 60.0f, 0.6f);

        for (int i = 0; i < loopLen; ++i) {
            if (loopL[i] > 1.0f) loopL[i] = 1.0f;
            else if (loopL[i] < -1.0f) loopL[i] = -1.0f;

            if (loopR[i] > 1.0f) loopR[i] = 1.0f;
            else if (loopR[i] < -1.0f) loopR[i] = -1.0f;
        }
    }




    Grain& allocGrain()
{
    for (int i = 0; i < kMaxGrains; ++i)
        if (!grains[i].active)
            return grains[i];

    if (activeCount > 0) {
        Grain& g = grains[activeIdx[0]];
        swapRemove(0);
        return g;
    }
    return grains[0];

}

void spawnOneGrain(int note, int vel, float densityMul)
{
    const int32_t sliceStart = 0;
    const int32_t snapRadius = int32_t(sr * 0.010f);
    const float d = 1.0f * densityMul;

    Grain& g = allocGrain();
    g.active = true;

    g.step = fPitchRate;

    const int32_t raw = sliceStart + int32_t((frand01(rng) - 0.5f) * 2.0f * snapRadius);
    const int32_t snapped = snapBackward(raw, snapRadius);

    const float snapAmount = 0.85f;
    const float start = (1.0f - snapAmount) * float(raw) + snapAmount * float(snapped);
    g.pos = wrapPos(start);

    const float grainMsBase = 10.0f + frand01(rng) * 35.0f;

    const float velNorm  = float(vel) / 127.0f;
    const float velCurve = velNorm * velNorm;

    const float minMs = 12.0f;
    const float maxMs = 220.0f;

    const float grainMsTarget = minMs + (maxMs - minMs) * velCurve;

    const float amt = fVelToGrainSize;
    const float grainMs = grainMsBase + (grainMsTarget - grainMsBase) * amt;

    g.dur = std::max(16, int32_t(sr * (grainMs / 1000.0f)));
    g.age = 0;

    g.amp = 0.08f + 0.35f * d;

    const float semis  = (frand01(rng) - 0.5f) * 1.0f;
    const float detune = std::pow(2.0f, semis / 12.0f);
    g.step *= detune;

    g.note = uint8_t(note & 0x7F);
    g.releasing = false;
    g.rel = 1.0f;
    g.relDec = 0.0f;

    g.delay = int32_t(sr * (frand01(rng) * 0.012f));

    const int idx = int(&g - grains);
if (activeCount < kMaxGrains)
{
    activeIdx[activeCount++] = idx;
}

} // end spawnOneGrain

    // ---- member variables ----
    static constexpr int kMaxGrains = 64;
    static constexpr int kWinSize   = 1024;

    float sr = 48000.0f;
    float fPitchRate = 1.0f;
    bool noteIsHeld[128]{};
    uint8_t lastNoteOn  = 60;
    uint8_t lastNoteOff = 0;

    static constexpr int kMaxLoop = 48000 * 4;
    float loopL[kMaxLoop]{};
    float loopR[kMaxLoop]{};
    int32_t loopLen = 0;
    uint32_t rng = 0x12345678u;

    float fDensityNorm  = 0.25f;
    float fGrainsPerSec = 10.0f;

    float fVelToGrainSize = 0.5f;

    static constexpr int kMaxMarkers = 64;
    int32_t markers[kMaxMarkers]{};
    int markerCount = 0;

    Grain grains[kMaxGrains]{};
    int   activeIdx[kMaxGrains]{};
    int   activeCount = 0;

    float win[kWinSize]{};
    float fReleaseMs = 3000.0f;
    float   fSpawnAcc = 0.0f;
    int32_t fReleaseLeft = 0;
    uint8_t fCurrentNote = 60;
    uint8_t fCurrentVel  = 100;


    
};






    

class SendNoteExamplePlugin : public Plugin
{
    enum Parameters {
        paramVolume = 0,
        paramVelocityAmount,
        paramVelocityGrainSize,
        paramCount
    };

public:
    SendNoteExamplePlugin()
    : Plugin(paramCount, 0, 0)
    {
    }




protected:
   /* 
    * Information */

   /**
      Get the plugin label.
      This label is a short restricted name consisting of only _, a-z, A-Z and 0-9 characters.
    */
    const char* getLabel() const override
    {
        return "SendNote";
    }

   /**
      Get an extensive comment/description about the plugin.
    */
    const char* getDescription() const override
    {
        return "Plugin that demonstrates sending notes from the editor in DPF.";
    }

   /**
      Get the plugin author/maker.
    */
    const char* getMaker() const override
    {
        return "DISTRHO";
    }

   /**
      Get the plugin homepage.
    */
    const char* getHomePage() const override
    {
        return "https://github.com/DISTRHO/DPF";
    }

   /**
      Get the plugin license name (a single line of text).
      For commercial plugins this should return some short copyright information.
    */
    const char* getLicense() const override
    {
        return "ISC";
    }

   /**
      Get the plugin version, in hexadecimal.
    */
    uint32_t getVersion() const override
{
    return d_version(1, 0, 0);
}
float getParameterValue(uint32_t index) const override
{
    if (index == paramVolume)
        return fVolume;
    if (index == paramVelocityAmount)
        return fVelocityAmount;
    if (index == paramVelocityGrainSize)
        return fVelocityGrainSize;

    return 0.0f;
}


void setParameterValue(uint32_t index, float value) override
{
    if (index == paramVolume)
    {
        fVolume = value;
    }
    else if (index == paramVelocityAmount)
    {
        fVelocityAmount = value;
    }
    else if (index == paramVelocityGrainSize)
    {
        fVelocityGrainSize = value;
        fGran.setVelToGrainSize(value);
    }
}







// 🔹 TRIN 3 – Parameter-definition
void initParameter(uint32_t index, Parameter& parameter) override
{
    if (index == paramVolume)
    {
        parameter.hints = kParameterIsAutomatable;

        parameter.name = "Volume";
        parameter.symbol = "volume";
        parameter.ranges.def = 0.8f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
    }
    else if (index == paramVelocityAmount)
    {
        parameter.hints = kParameterIsAutomatable;

        parameter.name = "Velocity → Density";
        parameter.symbol = "velDensity";
        parameter.unit = "%";
        parameter.ranges.def = 0.75f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
    }
    else if (index == paramVelocityGrainSize)
{
    parameter.hints = kParameterIsAutomatable;
    parameter.name = "Velocity → Grain Size";
    parameter.symbol = "velGrainSize";
    parameter.ranges.def = 0.50f;
    parameter.ranges.min = 0.0f;
    parameter.ranges.max = 1.0f;
}



}




   /* --------------------------------------------------------------------------------------------------------
    * Init */

   /**
      Initialize the audio port @a index.@n
      This function will be called once, shortly after the plugin is created.
    */
    void initAudioPort(bool input, uint32_t index, AudioPort& port) override
    {
        // treat meter audio ports as stereo
        port.groupId = kPortGroupMono;

        // everything else is as default
        Plugin::initAudioPort(input, index, port);
    }

   /* --------------------------------------------------------------------------------------------------------
    * Audio/MIDI Processing */

   /**
      Run/process function for plugins with MIDI input.
      This synthesizes the MIDI voices with a sum of sine waves.
    */
    void run(const float**, float** outputs, uint32_t frames,
         const MidiEvent* midiEvents, uint32_t midiEventCount) override
{
    float* outL = outputs[0];
    float* outR = outputs[1];

    if (!fGranInit) {
        fGran.init(getSampleRate());
        fGranInit = true;
    }

    // clear outputs
    for (uint32_t f = 0; f < frames; ++f) {
        outL[f] = 0.0f;
        outR[f] = 0.0f;
    }

    // MIDI -> trigger / noteoff
    for (uint32_t i = 0; i < midiEventCount; ++i)
    {
        const MidiEvent& ev(midiEvents[i]);
        if (ev.size < 3)
            continue;

        const uint8_t status = ev.data[0] & 0xF0;
        const uint8_t note   = ev.data[1] & 0x7F;
        const uint8_t vel    = ev.data[2] & 0x7F;

        if (status == 0x90 && vel > 0) // Note On
            fGran.trigger(note, vel);
        else if (status == 0x80 || (status == 0x90 && vel == 0)) // Note Off (inkl NoteOn vel=0)
            fGran.noteOff(note);
    }

    // render grains
    fGran.process(outL, outR, frames);

    // Apply output volume (simple taper)
    const float vol = fVolume * fVolume;
    for (uint32_t f = 0; f < frames; ++f) {
        outL[f] *= vol;
        outR[f] *= vol;
    }
}




private:
    GranularEngine fGran;
    bool fGranInit = false;
    float fVolume = 0.8f;
    float fVelocityAmount = 0.75f;      // velocity → density
    float fVelocityGrainSize = 0.5f;    // velocity → grain size  👈 5C.2
    



    



    


};








    // -------------------------------------------------------------------------------------------------------

/* ------------------------------------------------------------------------------------------------------------
 * Plugin entry point, called by DPF to create a new plugin instance. */

Plugin* createPlugin()
{
    return new SendNoteExamplePlugin();
}

END_NAMESPACE_DISTRHO

