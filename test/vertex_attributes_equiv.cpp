// Whole-fixture equivalence + memory-bound probe for the attribute structure. Compiled once
// against the changed header (no OLD_API variant is needed: it compares the two overloads of the
// same header). Not wired into CMakeLists.txt; build directly:
//
//   g++ -std=c++17 -O2 -Iinclude -I test -DEARCUT_TILES_BIN=<tiles.bin>
//       test/fixtures/mvt_fixture.cpp test/vertex_attributes_equiv.cpp -o vertex_attributes_equiv
//
// For every polygon in the 119,680-feature MVT fixture and the 0/3/8 attribute tiers it asserts
// the attribute-aware result is bit-identical (memcmp over the whole index vector) to the plain
// mapbox::earcut() call, then reports peak RSS so the memory upper bound can be checked.
#include <mapbox/earcut.hpp>

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#if defined(__linux__)
#include <sys/resource.h>
#endif

#include "fixtures/mvt_fixture.hpp"

namespace {

using Polygon = mapbox::fixtures::MvtPolygon;

std::size_t totalVertices(const Polygon& poly) {
    std::size_t n = 0;
    for (const auto& ring : poly) n += ring.size();
    return n;
}

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

long peakRssKb() {
#if defined(__linux__)
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    return usage.ru_maxrss;
#else
    return -1;
#endif
}

} // namespace

int main() {
    const auto& features = mapbox::fixtures::mvtFixture();
    const int tiers[] = {0, 3, 8};

    std::size_t polygonsChecked = 0;
    std::size_t indicesChecked = 0;
    bool allIdentical = true;

    for (int attrs : tiers) {
        std::size_t tierMismatches = 0;
        std::size_t tierIndices = 0;

        for (const auto& feature : features) {
            const Polygon& poly = feature.polygon;
            const auto reference = mapbox::earcut<uint32_t>(poly);

            Attrs store(totalVertices(poly), attrs);
            const mapbox::VertexData table = tableFor(store, totalVertices(poly));
            const auto result = mapbox::earcut<uint32_t>(poly, table);

            if (result.indices().size() != reference.size() ||
                std::memcmp(result.indices().data(), reference.data(),
                            reference.size() * sizeof(uint32_t)) != 0) {
                ++tierMismatches;
                allIdentical = false;
            }
            tierIndices += reference.size();
        }

        std::printf("attrs=%d: %zu polygons, %zu indices compared, mismatches=%zu\n", attrs, features.size(),
                    tierIndices, tierMismatches);
        polygonsChecked += features.size();
        indicesChecked += tierIndices;
    }

    std::printf("TOTAL: %zu polygon-calls, %zu indices; bit-identical=%s\n", polygonsChecked, indicesChecked,
                allIdentical ? "YES" : "NO");
    std::printf("sizeof(VertexAttribute)=%zu sizeof(VertexData)=%zu sizeof(EarcutResult<uint32_t>)=%zu\n",
                sizeof(mapbox::VertexAttribute), sizeof(mapbox::VertexData),
                sizeof(mapbox::EarcutResult<uint32_t>));
    std::printf("peak RSS=%ld kB\n", peakRssKb());
    return allIdentical ? 0 : 1;
}
