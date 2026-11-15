// Standalone, dependency-free micro-benchmark for the pluggable attribute structure.
// Intentionally not added to CMakeLists.txt (the build scripts must stay untouched). The same
// source is compiled twice — once against the pre-change header (OLD_API) and once against the
// changed header — so every number comes from the identical input set on the same binary shape:
//
//   g++ -std=c++17 -O2 -DOLD_API -I<old-include> -DEARCUT_TILES_BIN=<tiles.bin>
//       test/fixtures/mvt_fixture.cpp test/vertex_attributes_bench.cpp -o bench_old
//   g++ -std=c++17 -O2 -Iinclude -DEARCUT_TILES_BIN=<tiles.bin>
//       test/fixtures/mvt_fixture.cpp test/vertex_attributes_bench.cpp -o bench_new
//
// OLD_API only exposes mapbox::earcut(poly); that build measures pure triangulation with zero
// attributes (the historical behavior). The new build measures pure triangulation at 0/3/8
// attributes plus the full index+attribute gather path, so the attribute overhead is isolated.
#include <mapbox/earcut.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <string>
#include <vector>

#include "fixtures/mvt_fixture.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using Polygon = mapbox::fixtures::MvtPolygon;
using Point = mapbox::fixtures::MvtPoint;

volatile float g_gatherSink = 0.0f;

double secondsSince(Clock::time_point t0) {
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

// One large single-ring polygon: an R x C grid outline traced as a spiral-ish simple ring is
// complex to build, so use a deterministic star-shaped simple polygon with `n` vertices instead
// (concave, exercises the hashed ear path once n > 80).
Polygon starPolygon(std::size_t n) {
    Polygon poly(1);
    poly[0].reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double angle = 2.0 * 3.14159265358979323846 * double(i) / double(n);
        const double radius = (i % 2 == 0) ? 1000.0 : 400.0;
        poly[0].push_back(Point(static_cast<int>(radius * std::cos(angle) + 2000.0),
                                static_cast<int>(radius * std::sin(angle) + 2000.0)));
    }
    return poly;
}

std::size_t totalVertices(const Polygon& poly) {
    std::size_t n = 0;
    for (const auto& ring : poly) n += ring.size();
    return n;
}

// Per-attribute backing store. Each attribute is components-per-vertex floats in an AoS buffer.
struct AttributeStore {
    explicit AttributeStore(std::size_t vertexCount, int attributeCount, int components = 3)
        : buffers(static_cast<std::size_t>(attributeCount)) {
        for (auto& buffer : buffers) {
            buffer.resize(vertexCount * static_cast<std::size_t>(components));
            for (std::size_t i = 0; i < buffer.size(); ++i) buffer[i] = static_cast<float>(i) * 0.5f;
        }
    }
    std::vector<std::vector<float>> buffers;
};

#ifndef OLD_API
mapbox::VertexData makeTable(const AttributeStore& store, std::size_t vertexCount, int components = 3) {
    mapbox::VertexData table(vertexCount);
    for (const auto& buffer : store.buffers) {
        table.addAttribute(
            mapbox::VertexAttribute{buffer.data(), static_cast<std::size_t>(components), sizeof(float),
                                    static_cast<std::size_t>(components) * sizeof(float)});
    }
    return table;
}
#endif

struct Timing {
    double triangulate = 0; // earcut() only
    double gather = 0;      // index -> attribute resolution for every emitted index (new build only)
    std::size_t indices = 0;
};

#ifdef OLD_API
Timing runOnce(const Polygon& poly) {
    Timing t;
    const auto t0 = Clock::now();
    auto indices = mapbox::earcut<uint32_t>(poly);
    t.triangulate = secondsSince(t0);
    t.indices = indices.size();
    return t;
}
#else
Timing runOnce(const Polygon& poly, const mapbox::VertexData& table, int attributeCount) {
    Timing t;
    const auto t0 = Clock::now();
    auto result = mapbox::earcut<uint32_t>(poly, table);
    t.triangulate = secondsSince(t0);

    if (attributeCount > 0) {
        const auto g0 = Clock::now();
        float sink = 0.0f;
        for (std::size_t i = 0; i < result.indices().size(); ++i)
            sink += *result.attributeAt<float>(i, static_cast<std::size_t>(attributeCount - 1), 0);
        t.gather = secondsSince(g0);
        // global volatile sink keeps the gather from being optimized out
        g_gatherSink = sink;
    }
    t.indices = result.indices().size();
    return t;
}
#endif

// Repeat the whole polygon set `iterations` times; return mean milliseconds per polygon and
// total throughput. The first repetition is a warmup (allocation caches, branch predictors).
void report(const char* label,
            const std::vector<Polygon*>& polys,
            int attributeCount,
            int iterations,
            const std::vector<AttributeStore>& stores) {
#ifndef OLD_API
    std::vector<mapbox::VertexData> tables;
    tables.reserve(polys.size());
    for (std::size_t k = 0; k < polys.size(); ++k)
        tables.push_back(makeTable(stores[k], totalVertices(*polys[k])));
#endif

    double sumTri = 0, sumGather = 0;
    std::size_t sumIndices = 0, sumVerts = 0;
    for (const auto* p : polys) sumVerts += totalVertices(*p);

    for (int it = 0; it < iterations; ++it) {
        for (std::size_t k = 0; k < polys.size(); ++k) {
#ifdef OLD_API
            (void)stores;
            Timing tm = runOnce(*polys[k]);
#else
            Timing tm = runOnce(*polys[k], tables[k], attributeCount);
#endif
            if (it > 0) { // skip warmup
                sumTri += tm.triangulate;
                sumGather += tm.gather;
                sumIndices += tm.indices;
            }
        }
    }
    const int measured = iterations - 1;
    const double perPolyUs = 1e6 * sumTri / (static_cast<double>(measured) * static_cast<double>(polys.size()));
    const double gatherUs = 1e6 * sumGather / (static_cast<double>(measured) * static_cast<double>(polys.size()));
    std::printf("%-28s attrs=%d  verts=%9zu  tris=%9zu  tri/poly=%8.3fus  gather/poly=%8.3fus\n",
                label, attributeCount, sumVerts, sumIndices / static_cast<std::size_t>(measured) / 3, perPolyUs,
                gatherUs);
}

} // namespace

int main() {
    std::printf("mode=%s",
#ifdef OLD_API
                "OLD_API"
#else
                "NEW_API"
#endif
    );
#ifndef OLD_API
    std::printf("  sizeof(VertexAttribute)=%zu", sizeof(mapbox::VertexAttribute));
    std::printf(" sizeof(VertexData)=%zu sizeof(EarcutResult<uint32_t>)=%zu", sizeof(mapbox::VertexData),
                sizeof(mapbox::EarcutResult<uint32_t>));
#endif
    std::printf("\n");

    // Workload A: the real MVT production fixture (119,680 polygons).
    const auto& features = mapbox::fixtures::mvtFixture();
    std::vector<Polygon*> mvtPolys;
    for (const auto& f : features) mvtPolys.push_back(const_cast<Polygon*>(&f.polygon));

    // Workload B: one large concave polygon per benchmark, giving a stable big-N data point.
    const Polygon big = starPolygon(20000);
    std::vector<Polygon*> bigPolys = {const_cast<Polygon*>(&big)};

    const int attributeCounts[] = {0, 3, 8};

    for (int attrs : attributeCounts) {
        // backing stores are built but never touched by the OLD_API triangulation
        std::vector<AttributeStore> mvtStores;
        mvtStores.reserve(mvtPolys.size());
        for (const auto* p : mvtPolys) mvtStores.emplace_back(totalVertices(*p), attrs);
        report("MVT-all", mvtPolys, attrs, 6, mvtStores);

        std::vector<AttributeStore> bigStore;
        bigStore.emplace_back(totalVertices(big), attrs);
        report("star-20k", bigPolys, attrs, 40, bigStore);
    }
    return 0;
}
