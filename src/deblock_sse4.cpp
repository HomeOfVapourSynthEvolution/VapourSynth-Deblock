#ifdef DEBLOCK_X86
#include <type_traits>

#include "deblock.h"

template<typename pixel_t>
static inline auto load(const pixel_t* srcp) noexcept {
    if constexpr (std::is_same_v<pixel_t, uint8_t>)
        return Vec4i().load_4uc(srcp);
    else if constexpr (std::is_same_v<pixel_t, uint16_t>)
        return Vec4i().load_4us(srcp);
    else
        return Vec4f().load(srcp);
}

template<typename vector_t, typename pixel_t>
static inline void store(const vector_t srcp, pixel_t* dstp, const int peak) noexcept {
    if constexpr (std::is_same_v<pixel_t, uint8_t>) {
        const auto result = compress_saturated_s2u(compress_saturated(srcp, zero_si128()), zero_si128());
        result.store_si32(dstp);
    } else if constexpr (std::is_same_v<pixel_t, uint16_t>) {
        const auto result = compress_saturated_s2u(srcp, zero_si128());
        min(result, peak).storel(dstp);
    } else {
        srcp.store(dstp);
    }
}

template<typename vector_t>
static inline void transpose(vector_t& r0, vector_t& r1, vector_t& r2, vector_t& r3) noexcept {
    const vector_t t0 = blend4<0, 4, 1, 5>(r0, r1);
    const vector_t t1 = blend4<0, 4, 1, 5>(r2, r3);
    const vector_t t2 = blend4<2, 6, 3, 7>(r0, r1);
    const vector_t t3 = blend4<2, 6, 3, 7>(r2, r3);
    r0 = blend4<0, 1, 4, 5>(t0, t1);
    r1 = blend4<2, 3, 6, 7>(t0, t1);
    r2 = blend4<0, 1, 4, 5>(t2, t3);
    r3 = blend4<2, 3, 6, 7>(t2, t3);
}

template<typename pixel_t, typename vector_t>
static inline void deblockCore(const vector_t p2, vector_t& p1, vector_t& p0, vector_t& q0, vector_t& q1, const vector_t q2,
                               const DeblockData* VS_RESTRICT d) noexcept {
    if constexpr (std::is_integral_v<pixel_t>) {
        const vector_t alpha = d->alpha;
        const vector_t beta = d->beta;
        const vector_t c0 = d->c0;
        const vector_t c1 = d->c1;

        const auto mask = (abs(p0 - q0) < alpha) & (abs(p1 - p0) < beta) & (abs(q0 - q1) < beta);

        const auto apMask = abs(p2 - p0) < beta;
        const auto aqMask = abs(q2 - q0) < beta;

        const vector_t c = c0 + select(apMask, c1, zero_si128()) + select(aqMask, c1, zero_si128());

        const vector_t avg = (p0 + q0 + 1) >> 1;
        const vector_t delta = min(max((((q0 - p0) << 2) + p1 - q1 + 4) >> 3, -c), c);
        const vector_t deltap1 = min(max((p2 + avg - (p1 << 1)) >> 1, -c0), c0);
        const vector_t deltaq1 = min(max((q2 + avg - (q1 << 1)) >> 1, -c0), c0);

        p0 = select(mask, p0 + delta, p0);
        q0 = select(mask, q0 - delta, q0);
        p1 = select(mask & apMask, p1 + deltap1, p1);
        q1 = select(mask & aqMask, q1 + deltaq1, q1);
    } else {
        const vector_t alpha = d->alphaF;
        const vector_t beta = d->betaF;
        const vector_t c0 = d->c0F;
        const vector_t c1 = d->c1F;

        const auto mask = (abs(p0 - q0) < alpha) & (abs(p1 - p0) < beta) & (abs(q0 - q1) < beta);

        const auto apMask = abs(p2 - p0) < beta;
        const auto aqMask = abs(q2 - q0) < beta;

        const vector_t c = c0 + select(apMask, c1, zero_4f()) + select(aqMask, c1, zero_4f());

        const vector_t avg = (p0 + q0) * 0.5f;
        const vector_t delta = min(max(((q0 - p0) * 4.0f + p1 - q1) * 0.125f, -c), c);
        const vector_t deltap1 = min(max((p2 + avg - p1 * 2.0f) * 0.5f, -c0), c0);
        const vector_t deltaq1 = min(max((q2 + avg - q1 * 2.0f) * 0.5f, -c0), c0);

        p0 = select(mask, p0 + delta, p0);
        q0 = select(mask, q0 - delta, q0);
        p1 = select(mask & apMask, p1 + deltap1, p1);
        q1 = select(mask & aqMask, q1 + deltaq1, q1);
    }
}

template<typename pixel_t>
static inline void deblockHorEdge(pixel_t* VS_RESTRICT dstp, const ptrdiff_t stride, const DeblockData* VS_RESTRICT d) noexcept {
    const auto p2 = load(dstp - 3 * stride);
    auto p1 = load(dstp - 2 * stride);
    auto p0 = load(dstp - stride);
    auto q0 = load(dstp);
    auto q1 = load(dstp + stride);
    const auto q2 = load(dstp + 2 * stride);

    deblockCore<pixel_t>(p2, p1, p0, q0, q1, q2, d);

    store(p1, dstp - 2 * stride, d->peak);
    store(p0, dstp - stride, d->peak);
    store(q0, dstp, d->peak);
    store(q1, dstp + stride, d->peak);
}

template<typename pixel_t>
static inline void deblockVerEdge(pixel_t* VS_RESTRICT dstp, const ptrdiff_t stride, const DeblockData* VS_RESTRICT d) noexcept {
    auto a0 = load(dstp - 3);
    auto a1 = load(dstp - 3 + stride);
    auto a2 = load(dstp - 3 + 2 * stride);
    auto a3 = load(dstp - 3 + 3 * stride);
    transpose(a0, a1, a2, a3);

    auto b0 = load(dstp - 1);
    auto b1 = load(dstp - 1 + stride);
    auto b2 = load(dstp - 1 + 2 * stride);
    auto b3 = load(dstp - 1 + 3 * stride);
    transpose(b0, b1, b2, b3);

    const auto p2 = a0;
    auto p1 = a1;
    auto p0 = a2;
    auto q0 = a3;
    auto q1 = b2;
    const auto q2 = b3;

    deblockCore<pixel_t>(p2, p1, p0, q0, q1, q2, d);

    transpose(p1, p0, q0, q1);
    store(p1, dstp - 2, d->peak);
    store(p0, dstp - 2 + stride, d->peak);
    store(q0, dstp - 2 + 2 * stride, d->peak);
    store(q1, dstp - 2 + 3 * stride, d->peak);
}

template<typename pixel_t>
void filterSSE4(VSFrame* dst, const DeblockData* VS_RESTRICT d, const VSAPI* vsapi) noexcept {
    for (int plane = 0; plane < d->vi->format.numPlanes; plane++) {
        if (d->process[plane]) {
            const int width = vsapi->getFrameWidth(dst, plane);
            const int height = vsapi->getFrameHeight(dst, plane);
            const ptrdiff_t stride = vsapi->getStride(dst, plane) / sizeof(pixel_t);
            pixel_t* VS_RESTRICT dstp = reinterpret_cast<pixel_t*>(vsapi->getWritePtr(dst, plane));

            for (int x = 4; x < width; x += 4)
                deblockVerEdge(dstp + x, stride, d);

            dstp += 4 * stride;

            for (int y = 4; y < height; y += 4) {
                deblockHorEdge(dstp, stride, d);

                for (int x = 4; x < width; x += 4) {
                    deblockHorEdge(dstp + x, stride, d);
                    deblockVerEdge(dstp + x, stride, d);
                }

                dstp += 4 * stride;
            }
        }
    }
}

template void filterSSE4<uint8_t>(VSFrame* dst, const DeblockData* VS_RESTRICT d, const VSAPI* vsapi) noexcept;
template void filterSSE4<uint16_t>(VSFrame* dst, const DeblockData* VS_RESTRICT d, const VSAPI* vsapi) noexcept;
template void filterSSE4<float>(VSFrame* dst, const DeblockData* VS_RESTRICT d, const VSAPI* vsapi) noexcept;
#endif
