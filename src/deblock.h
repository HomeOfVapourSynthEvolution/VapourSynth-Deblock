#pragma once

#include <VapourSynth4.h>
#include <VSHelper4.h>

#ifdef DEBLOCK_X86
#include "vectorclass/vectorclass.h"
#endif

struct DeblockData final {
    VSNode* node;
    const VSVideoInfo* vi;
    bool process[3];
    int alpha, beta, c0, c1, peak;
    float alphaF, betaF, c0F, c1F;
    void (*filter)(VSFrame* dst, const DeblockData* VS_RESTRICT d, const VSAPI* vsapi) noexcept;
};
