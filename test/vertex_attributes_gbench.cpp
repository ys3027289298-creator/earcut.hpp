// Google Benchmark harness for the attribute-structure change. Kept out of CMakeLists.txt (the
// build scripts must stay untouched); compile directly against the fetched benchmark library,
// once per header version:
//
//   g++ -std=c++17 -O2 -DOLD_API -I<old-include> -I test -DEARCUT_TILES_BIN=<tiles.bin>
//       test/fixtures/mvt_fixture.cpp test/vertex_attributes_gbench.cpp libbenchmark.a -lpthread
//   g++ -std=c++17 -O2 -Iinclude -I test -DEARCUT_TILES_BIN=<tiles.bin>
//       test/fixtures/mvt_fixture.cpp test/vertex_attributes_gbench.cpp libbenchmark.a -lpthread
#include <benchmark/benchmark.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mapbox/earcut.hpp>
#include <vector>

#include "fixtures/mvt_fixture.hpp"

namespace {

using Polygon = mapbox::fixtures::MvtPolygon;
using Point = mapbox::fixtures::MvtPoint;

std::size_t totalVertices(const Polygon& poly) {
    std::size_t n = 0;
    for (const auto& ring : poly) n += ring.size();
    return n;
}

Polygon makeStar(std::size_t n) {
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

#ifndef OLD_API
// Build an A-attribute float table (3 components each) for a polygon's vertex count.
struct Attrs {
    explicit Attrs(std::size_t vertexCount, int attributeCount) {
        for (int a = 0; a < attributeCount; ++a) {
            buffers.emplace_back(vertexCount * 3);
            for (std::size_t i = 0; i < vertexCount * 3; ++i) buffers.back()[i] = float(i) * 0.5f;
        }
    }
    std::vector<std::vector<float>> buffers;
};

mapbox::VertexData tableFor(const Attrs& attrs, std::size_t vertexCount) {
    mapbox::VertexData table(vertexCount);
    for (const auto& buffer : attrs.buffers)
        table.addAttribute(mapbox::VertexAttribute{buffer.data(), 3, sizeof(float), 3 * sizeof(float)});
    return table;
}
#endif

// ---- MVT production workload: every polygon, 0/3/8 attributes ------------------------------
void bm_mvt(benchmark::State& state) {
    const int attributeCount = state.range(0);
    const auto& features = mapbox::fixtures::mvtFixture();

#ifndef OLD_API
    std::vector<Attrs> stores;
    std::vector<mapbox::VertexData> tables;
    for (const auto& f : features) {
        stores.emplace_back(f.vertexCount, attributeCount);
        tables.push_back(tableFor(stores.back(), f.vertexCount));
    }
#endif

    for (auto _ : state) {
        std::size_t triangles = 0;
        for (std::size_t k = 0; k < features.size(); ++k) {
#ifdef OLD_API
            auto indices = mapbox::earcut<uint32_t>(features[k].polygon);
            triangles += indices.size();
#else
            auto result = mapbox::earcut<uint32_t>(features[k].polygon, tables[k]);
            triangles += result.indices().size();
#endif
        }
        benchmark::DoNotOptimize(triangles);
    }
    state.SetItemsProcessed(state.iterations() * static_cast<int64_t>(features.size()));
}

// ---- One large concave polygon, 0/3/8 attributes -------------------------------------------
void bm_star(benchmark::State& state) {
    const int attributeCount = state.range(0);
    const Polygon poly = makeStar(20000);
    const std::size_t vertexCount = totalVertices(poly);

#ifndef OLD_API
    Attrs attrs(vertexCount, attributeCount);
    const mapbox::VertexData table = tableFor(attrs, vertexCount);
#endif

    for (auto _ : state) {
#ifdef OLD_API
        auto indices = mapbox::earcut<uint32_t>(poly);
        benchmark::DoNotOptimize(indices.data());
#else
        auto result = mapbox::earcut<uint32_t>(poly, table);
        benchmark::DoNotOptimize(result.indices().data());
#endif
    }
}

#ifndef OLD_API
// The added capability's own cost: resolve one attribute element for every emitted index.
void bm_gather(benchmark::State& state) {
    const int attributeCount = state.range(0);
    const Polygon poly = makeStar(20000);
    const std::size_t vertexCount = totalVertices(poly);
    Attrs attrs(vertexCount, attributeCount);
    const mapbox::VertexData table = tableFor(attrs, vertexCount);
    auto result = mapbox::earcut<uint32_t>(poly, table);

    for (auto _ : state) {
        float sink = 0.0f;
        for (std::size_t i = 0; i < result.indices().size(); ++i)
            sink += *result.attributeAt<float>(i, static_cast<std::size_t>(attributeCount - 1));
        benchmark::DoNotOptimize(sink);
    }
}
#endif

} // namespace

BENCHMARK(bm_mvt)->Arg(0)->Arg(3)->Arg(8)->Unit(benchmark::kMillisecond);
BENCHMARK(bm_star)->Arg(0)->Arg(3)->Arg(8)->Unit(benchmark::kMicrosecond);
#ifndef OLD_API
BENCHMARK(bm_gather)->Arg(3)->Arg(8)->Unit(benchmark::kMicrosecond);
#endif

BENCHMARK_MAIN();
