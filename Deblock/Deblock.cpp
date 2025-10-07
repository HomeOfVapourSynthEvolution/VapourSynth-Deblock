// VapourSynth port by HolyWu
//
// DeBlock plugin for Avisynth 2.5 - takes a clip, and deblock it using H264 deblocking
// Copyright(c)2004 Manao as a function in MVTools v.0.9.6.2
// Copyright(c)2006 Alexander Balakhnin aka Fizick - separate plugin, YUY2 support
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA, or visit
// http://www.gnu.org/copyleft/gpl.html .

#include <cmath>
#include <cstdlib>

#include <algorithm>
#include <string>
#include <memory>

#include <VapourSynth4.h>
#include <VSHelper4.h>

using namespace std::string_literals;

static constexpr int QUANT_MAX = 60; // generalized by Fizick (was max=51)

static constexpr int alphas[] = {
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 4, 4,
    5, 6, 7, 8, 9, 10,
    12, 13, 15, 17, 20,
    22, 25, 28, 32, 36,
    40, 45, 50, 56, 63,
    71, 80, 90, 101, 113,
    127, 144, 162, 182,
    203, 226, 255, 255,
    255, 255, 255, 255, 255, 255, 255, 255, 255 // added by Fizick 
};

static constexpr int betas[] = {
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 2,
    2, 3, 3, 3, 3, 4,
    4, 4, 6, 6,
    7, 7, 8, 8, 9, 9,
    10, 10, 11, 11, 12,
    12, 13, 13, 14, 14,
    15, 15, 16, 16, 17,
    17, 18, 18,
    19, 20, 21, 22, 23, 24, 25, 26, 27 // added by Fizick 
};

static constexpr int cs[] = {
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 1, 1,
    1, 1, 1, 1, 1, 1,
    1, 2, 2, 2, 2, 3,
    3, 3, 4, 4, 5, 5,
    6, 7, 8, 8, 10,
    11, 12, 13, 15, 17,
    19, 21, 23, 25, 27, 29, 31, 33, 35 // added by Fizick for really strong deblocking :)
};

struct DeblockData final {
    VSNode* node;
    const VSVideoInfo* vi;
    bool process[3];
    int alpha, beta, c0, c1, peak;
    float alphaF, betaF, c0F, c1F;
    void (*filter)(VSFrame* dst, const DeblockData* VS_RESTRICT d, const VSAPI* vsapi) noexcept;
};

template<typename pixel_t>
static inline void deblockHorEdge(pixel_t* VS_RESTRICT dstp, const ptrdiff_t stride, const DeblockData* VS_RESTRICT d) noexcept {
    const int alpha = d->alpha;
    const int beta = d->beta;
    const int c0 = d->c0;
    const int c1 = d->c1;

    pixel_t* VS_RESTRICT sq0 = dstp;
    pixel_t* VS_RESTRICT sq1 = dstp + stride;
    const pixel_t* sq2 = dstp + stride * 2;
    pixel_t* VS_RESTRICT sp0 = dstp - stride;
    pixel_t* VS_RESTRICT sp1 = dstp - stride * 2;
    const pixel_t* sp2 = dstp - stride * 3;

    for (int i = 0; i < 4; i++) {
        if (std::abs(sp0[i] - sq0[i]) < alpha && std::abs(sp1[i] - sp0[i]) < beta && std::abs(sq0[i] - sq1[i]) < beta) {
            const int ap = std::abs(sp2[i] - sp0[i]);
            const int aq = std::abs(sq2[i] - sq0[i]);

            int c = c0;
            if (aq < beta)
                c += c1;
            if (ap < beta)
                c += c1;

            const int avg = (sp0[i] + sq0[i] + 1) >> 1;
            const int delta = std::clamp(((sq0[i] - sp0[i]) * 4 + sp1[i] - sq1[i] + 4) >> 3, -c, c);
            const int deltap1 = std::clamp((sp2[i] + avg - sp1[i] * 2) >> 1, -c0, c0);
            const int deltaq1 = std::clamp((sq2[i] + avg - sq1[i] * 2) >> 1, -c0, c0);

            sp0[i] = std::clamp(sp0[i] + delta, 0, d->peak);
            sq0[i] = std::clamp(sq0[i] - delta, 0, d->peak);
            if (ap < beta)
                sp1[i] += deltap1;
            if (aq < beta)
                sq1[i] += deltaq1;
        }
    }
}

template<>
inline void deblockHorEdge(float* VS_RESTRICT dstp, const ptrdiff_t stride, const DeblockData* VS_RESTRICT d) noexcept {
    const float alpha = d->alphaF;
    const float beta = d->betaF;
    const float c0 = d->c0F;
    const float c1 = d->c1F;

    float* VS_RESTRICT sq0 = dstp;
    float* VS_RESTRICT sq1 = dstp + stride;
    const float* sq2 = dstp + stride * 2;
    float* VS_RESTRICT sp0 = dstp - stride;
    float* VS_RESTRICT sp1 = dstp - stride * 2;
    const float* sp2 = dstp - stride * 3;

    for (int i = 0; i < 4; i++) {
        if (std::abs(sp0[i] - sq0[i]) < alpha && std::abs(sp1[i] - sp0[i]) < beta && std::abs(sq0[i] - sq1[i]) < beta) {
            const float ap = std::abs(sp2[i] - sp0[i]);
            const float aq = std::abs(sq2[i] - sq0[i]);

            float c = c0;
            if (aq < beta)
                c += c1;
            if (ap < beta)
                c += c1;

            const float avg = (sp0[i] + sq0[i]) / 2.0f;
            const float delta = std::clamp(((sq0[i] - sp0[i]) * 4.0f + sp1[i] - sq1[i]) / 8.0f, -c, c);
            const float deltap1 = std::clamp((sp2[i] + avg - sp1[i] * 2.0f) / 2.0f, -c0, c0);
            const float deltaq1 = std::clamp((sq2[i] + avg - sq1[i] * 2.0f) / 2.0f, -c0, c0);

            sp0[i] += delta;
            sq0[i] -= delta;
            if (ap < beta)
                sp1[i] += deltap1;
            if (aq < beta)
                sq1[i] += deltaq1;
        }
    }
}

template<typename pixel_t>
static inline void deblockVerEdge(pixel_t* VS_RESTRICT dstp, const ptrdiff_t stride, const DeblockData* VS_RESTRICT d) noexcept {
    const int alpha = d->alpha;
    const int beta = d->beta;
    const int c0 = d->c0;
    const int c1 = d->c1;

    for (int i = 0; i < 4; i++) {
        if (std::abs(dstp[0] - dstp[-1]) < alpha && std::abs(dstp[1] - dstp[0]) < beta && std::abs(dstp[-1] - dstp[-2]) < beta) {
            const int ap = std::abs(dstp[2] - dstp[0]);
            const int aq = std::abs(dstp[-3] - dstp[-1]);

            int c = c0;
            if (aq < beta)
                c += c1;
            if (ap < beta)
                c += c1;

            const int avg = (dstp[0] + dstp[-1] + 1) >> 1;
            const int delta = std::clamp(((dstp[0] - dstp[-1]) * 4 + dstp[-2] - dstp[1] + 4) >> 3, -c, c);
            const int deltaq1 = std::clamp((dstp[2] + avg - dstp[1] * 2) >> 1, -c0, c0);
            const int deltap1 = std::clamp((dstp[-3] + avg - dstp[-2] * 2) >> 1, -c0, c0);

            dstp[0] = std::clamp(dstp[0] - delta, 0, d->peak);
            dstp[-1] = std::clamp(dstp[-1] + delta, 0, d->peak);
            if (ap < beta)
                dstp[1] += deltaq1;
            if (aq < beta)
                dstp[-2] += deltap1;
        }

        dstp += stride;
    }
}

template<>
inline void deblockVerEdge(float* VS_RESTRICT dstp, const ptrdiff_t stride, const DeblockData* VS_RESTRICT d) noexcept {
    const float alpha = d->alphaF;
    const float beta = d->betaF;
    const float c0 = d->c0F;
    const float c1 = d->c1F;

    for (int i = 0; i < 4; i++) {
        if (std::abs(dstp[0] - dstp[-1]) < alpha && std::abs(dstp[1] - dstp[0]) < beta && std::abs(dstp[-1] - dstp[-2]) < beta) {
            const float ap = std::abs(dstp[2] - dstp[0]);
            const float aq = std::abs(dstp[-3] - dstp[-1]);

            float c = c0;
            if (aq < beta)
                c += c1;
            if (ap < beta)
                c += c1;

            const float avg = (dstp[0] + dstp[-1]) / 2.0f;
            const float delta = std::clamp(((dstp[0] - dstp[-1]) * 4.0f + dstp[-2] - dstp[1]) / 8.0f, -c, c);
            const float deltaq1 = std::clamp((dstp[2] + avg - dstp[1] * 2.0f) / 2.0f, -c0, c0);
            const float deltap1 = std::clamp((dstp[-3] + avg - dstp[-2] * 2.0f) / 2.0f, -c0, c0);

            dstp[0] -= delta;
            dstp[-1] += delta;
            if (ap < beta)
                dstp[1] += deltaq1;
            if (aq < beta)
                dstp[-2] += deltap1;
        }

        dstp += stride;
    }
}

template<typename pixel_t>
static void filter(VSFrame* dst, const DeblockData* VS_RESTRICT d, const VSAPI* vsapi) noexcept {
    for (int plane = 0; plane < d->vi->format.numPlanes; plane++) {
        if (d->process[plane]) {
            const int width = vsapi->getFrameWidth(dst, plane);
            const int height = vsapi->getFrameHeight(dst, plane);
            const ptrdiff_t stride = vsapi->getStride(dst, plane) / sizeof(pixel_t);
            pixel_t* VS_RESTRICT dstp = reinterpret_cast<pixel_t*>(vsapi->getWritePtr(dst, plane));

            for (int x = 4; x < width; x += 4)
                deblockVerEdge(dstp + x, stride, d);

            dstp += stride * 4;

            for (int y = 4; y < height; y += 4) {
                deblockHorEdge(dstp, stride, d);

                for (int x = 4; x < width; x += 4) {
                    deblockHorEdge(dstp + x, stride, d);
                    deblockVerEdge(dstp + x, stride, d);
                }

                dstp += stride * 4;
            }
        }
    }
}

static const VSFrame* VS_CC deblockGetFrame(int n, int activationReason, void* instanceData, [[maybe_unused]] void** frameData, VSFrameContext* frameCtx,
                                            VSCore* core, const VSAPI* vsapi) {
    auto d = static_cast<const DeblockData*>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame* src = vsapi->getFrameFilter(n, d->node, frameCtx);
        VSFrame* dst = vsapi->copyFrame(src, core);

        d->filter(dst, d, vsapi);

        vsapi->freeFrame(src);
        return dst;
    }

    return nullptr;
}

static void VS_CC deblockFree(void* instanceData, [[maybe_unused]] VSCore* core, const VSAPI* vsapi) {
    auto d = static_cast<DeblockData*>(instanceData);
    vsapi->freeNode(d->node);
    delete d;
}

static void VS_CC deblockCreate(const VSMap* in, VSMap* out, [[maybe_unused]] void* userData, VSCore* core, const VSAPI* vsapi) {
    auto d = std::make_unique<DeblockData>();

    d->node = vsapi->mapGetNode(in, "clip", 0, nullptr);
    d->vi = vsapi->getVideoInfo(d->node);

    const int padWidth = (d->vi->width & 7) ? 8 - d->vi->width % 8 : 0;
    const int padHeight = (d->vi->height & 7) ? 8 - d->vi->height % 8 : 0;

    try {
        int err;

        if (!vsh::isConstantVideoFormat(d->vi) ||
            (d->vi->format.sampleType == stInteger && d->vi->format.bitsPerSample > 16) ||
            (d->vi->format.sampleType == stFloat && d->vi->format.bitsPerSample != 32))
            throw "only constant format 8-16 bit integer and 32 bit float input supported"s;

        int quant = vsapi->mapGetIntSaturated(in, "quant", 0, &err);
        if (err)
            quant = 25;

        int aOffset = vsapi->mapGetIntSaturated(in, "aoffset", 0, &err);

        int bOffset = vsapi->mapGetIntSaturated(in, "boffset", 0, &err);

        if (quant < 0 || quant > QUANT_MAX)
            throw "quant must be between 0 and " + std::to_string(QUANT_MAX) + " (inclusive)";

        const int m = vsapi->mapNumElements(in, "planes");

        for (int i = 0; i < 3; i++)
            d->process[i] = (m <= 0);

        for (int i = 0; i < m; i++) {
            const int n = vsapi->mapGetIntSaturated(in, "planes", i, nullptr);

            if (n < 0 || n >= d->vi->format.numPlanes)
                throw "plane index out of range"s;

            if (d->process[n])
                throw "plane specified twice"s;

            d->process[n] = true;
        }

        aOffset = std::clamp(aOffset, -quant, QUANT_MAX - quant);
        bOffset = std::clamp(bOffset, -quant, QUANT_MAX - quant);
        const int aIndex = std::clamp(quant + aOffset, 0, QUANT_MAX);
        const int bIndex = std::clamp(quant + bOffset, 0, QUANT_MAX);
        d->alpha = alphas[aIndex];
        d->beta = betas[bIndex];
        d->c0 = cs[aIndex];

        if (d->vi->format.bytesPerSample == 1)
            d->filter = filter<uint8_t>;
        else if (d->vi->format.bytesPerSample == 2)
            d->filter = filter<uint16_t>;
        else
            d->filter = filter<float>;

        if (d->vi->format.sampleType == stInteger) {
            d->peak = (1 << d->vi->format.bitsPerSample) - 1;
            const int scale = 1 << (d->vi->format.bitsPerSample - 8);
            d->alpha *= scale;
            d->beta *= scale;
            d->c0 *= scale;
            d->c1 = scale;
        } else {
            d->alphaF = d->alpha / 255.0f;
            d->betaF = d->beta / 255.0f;
            d->c0F = d->c0 / 255.0f;
            d->c1F = 1.0f / 255.0f;
        }

        if (padWidth || padHeight) {
            VSMap* args = vsapi->createMap();
            vsapi->mapConsumeNode(args, "clip", d->node, maReplace);
            vsapi->mapSetInt(args, "width", d->vi->width + padWidth, maReplace);
            vsapi->mapSetInt(args, "height", d->vi->height + padHeight, maReplace);
            vsapi->mapSetFloat(args, "src_width", d->vi->width + padWidth, maReplace);
            vsapi->mapSetFloat(args, "src_height", d->vi->height + padHeight, maReplace);

            VSMap* ret = vsapi->invoke(vsapi->getPluginByID("com.vapoursynth.resize", core), "Point", args);
            if (vsapi->mapGetError(ret)) {
                vsapi->mapSetError(out, vsapi->mapGetError(ret));
                vsapi->freeMap(args);
                vsapi->freeMap(ret);
                return;
            }

            d->node = vsapi->mapGetNode(ret, "clip", 0, nullptr);
            d->vi = vsapi->getVideoInfo(d->node);
            vsapi->freeMap(args);
            vsapi->freeMap(ret);
        }
    } catch (const std::string& error) {
        vsapi->mapSetError(out, ("Deblock: " + error).c_str());
        vsapi->freeNode(d->node);
        return;
    }

    VSFilterDependency deps[] = { {d->node, rpStrictSpatial} };
    vsapi->createVideoFilter(out, "Deblock", d->vi, deblockGetFrame, deblockFree, fmParallel, deps, 1, d.get(), core);
    d.release();

    if (padWidth || padHeight) {
        VSNode* node = vsapi->mapGetNode(out, "clip", 0, nullptr);
        vsapi->clearMap(out);

        VSMap* args = vsapi->createMap();
        vsapi->mapConsumeNode(args, "clip", node, maReplace);
        vsapi->mapSetInt(args, "right", padWidth, maReplace);
        vsapi->mapSetInt(args, "bottom", padHeight, maReplace);

        VSMap* ret = vsapi->invoke(vsapi->getPluginByID("com.vapoursynth.std", core), "Crop", args);
        if (vsapi->mapGetError(ret)) {
            vsapi->mapSetError(out, vsapi->mapGetError(ret));
            vsapi->freeMap(args);
            vsapi->freeMap(ret);
            return;
        }

        node = vsapi->mapGetNode(ret, "clip", 0, nullptr);
        vsapi->freeMap(args);
        vsapi->freeMap(ret);
        vsapi->mapConsumeNode(out, "clip", node, maReplace);
    }
}

//////////////////////////////////////////
// Init

VS_EXTERNAL_API(void) VapourSynthPluginInit2(VSPlugin* plugin, const VSPLUGINAPI* vspapi) {
    vspapi->configPlugin("com.holywu.deblock",
                         "deblock",
                         "It does a deblocking of the picture, using the deblocking filter of h264",
                         VS_MAKE_VERSION(7, 1),
                         VAPOURSYNTH_API_VERSION,
                         0,
                         plugin);

    vspapi->registerFunction("Deblock",
                             "clip:vnode;quant:int:opt;aoffset:int:opt;boffset:int:opt;planes:int[]:opt;",
                             "clip:vnode;",
                             deblockCreate,
                             nullptr,
                             plugin);
}
