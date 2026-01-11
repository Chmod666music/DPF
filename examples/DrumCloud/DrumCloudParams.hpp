#pragma once
#include <cstdint>

enum Parameters : uint32_t
{
    paramVolume = 0,
    paramReleaseMs,        // 🎛 Release (ms)
    paramVelocityAmount,
    paramVelocityGrainSize,
    paramStartPosition,
    paramPositionSpread,
    paramSnapMs,
    paramSamplePath,
    paramCount
};

