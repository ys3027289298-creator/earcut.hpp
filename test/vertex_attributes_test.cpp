// Standalone (CMake-agnostic) verification for the pluggable per-vertex attribute structure.
// Builds with the existing googletest fetch but is intentionally not wired into CMakeLists.txt,
// which must stay untouched. Compile, for example:
//   g++ -std=c++17 -Iinclude -I<googletest>/googletest/include
//       test/vertex_attributes_test.cpp libgtest.a libgtest_main.a -o vertex_attributes_test
#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <cstddef>
#include <cstdint>
#include <mapbox/earcut.hpp>
#include <stdexcept>
#include <vector>

using mapbox::EarcutResult;
using mapbox::VertexAttribute;
using mapbox::VertexData;

namespace {

using Point = std::array<double, 2>;
using Polygon = std::vector<std::vector<Point>>;

// Square with a square hole: 8 input vertices, 8 output triangles (24 indices).
Polygon squareWithHole() {
    return Polygon{{{0, 0}, {100, 0}, {100, 100}, {0, 100}},
                   {{25, 25}, {75, 25}, {75, 75}, {25, 75}}};
}

std::size_t totalVertices(const Polygon& poly) {
    std::size_t n = 0;
    for (const auto& ring : poly) n += ring.size();
    return n;
}

// A concave ring (with a notch) so the triangulation is non-trivial.
Polygon concavePolygon() {
    return Polygon{{{0, 0}, {10, 0}, {10, 10}, {6, 10}, {6, 6}, {4, 6}, {4, 10}, {0, 10}}};
}

// Build n-attribute table over `vertexCount` vertices. Each attribute is a distinct element
// type so alignment mismatches cannot hide behind a common type.
template <typename T>
VertexData makeTable(std::size_t vertexCount,
                     const std::vector<std::vector<T>>& buffers,
                     std::size_t components) {
    VertexData data(vertexCount);
    for (const auto& buffer : buffers) {
        EXPECT_EQ(buffer.size(), vertexCount * components);
        data.addAttribute(VertexAttribute{buffer.data(), components, sizeof(T), components * sizeof(T)});
    }
    return data;
}

} // namespace

// ------------------------------------------------------------------------------------------------
// sizeof / offsetof / alignment of the public structures (LP64 assumptions are guarded).
// ------------------------------------------------------------------------------------------------
TEST(VertexLayoutTest, SizeAndOffsets) {
    // VertexAttribute is four fields: pointer + three size_t => 32 bytes on 64-bit, naturally
    // aligned to 8.
    if (sizeof(void*) == 8) {
        EXPECT_EQ(sizeof(VertexAttribute), 32u);
        EXPECT_EQ(alignof(VertexAttribute), 8u);
        EXPECT_EQ(offsetof(VertexAttribute, data), 0u);
        EXPECT_EQ(offsetof(VertexAttribute, count), 8u);
        EXPECT_EQ(offsetof(VertexAttribute, elementSize), 16u);
        EXPECT_EQ(offsetof(VertexAttribute, stride), 24u);
    } else {
        EXPECT_EQ(alignof(VertexAttribute), alignof(std::size_t));
        EXPECT_EQ(sizeof(VertexAttribute),
                  alignof(VertexAttribute) + 3 * sizeof(std::size_t));
    }

    // VertexData holds a vector (24 bytes on LP64) plus a size_t.
    if (sizeof(void*) == 8) {
        EXPECT_EQ(sizeof(std::vector<VertexAttribute>), 24u);
        // vector(24) + size_t(8), no trailing padding: the size_t fills the vector's tail.
        EXPECT_EQ(sizeof(VertexData), 32u);
        EXPECT_EQ(alignof(VertexData), 8u);
    }

    // EarcutResult: vector<N> + pointer + size_t = 24 + 8 + 8 for N=uint32_t on LP64.
    if (sizeof(void*) == 8) {
        EXPECT_EQ(sizeof(EarcutResult<uint32_t>), 40u);
        EXPECT_EQ(alignof(EarcutResult<uint32_t>), 8u);
    }
}

TEST(VertexLayoutTest, ByteOffsetArithmetic) {
    // Interleaved AoS: stride covers all components.
    const VertexAttribute aos{nullptr, 3, sizeof(float), 3 * sizeof(float)};
    EXPECT_EQ(aos.byteOffset(0, 0), 0u);
    EXPECT_EQ(aos.byteOffset(2, 1), std::size_t(2 * 12 + 4));
    EXPECT_EQ(aos.byteOffset(5, 2), std::size_t(5 * 12 + 8));

    // Planar SoA: stride equals one element.
    const VertexAttribute soa{nullptr, 1, sizeof(double), sizeof(double)};
    EXPECT_EQ(soa.byteOffset(7, 0), 7 * sizeof(double));
}

// ------------------------------------------------------------------------------------------------
// Bit-identical 2D output for 0 / 3 / 8 attributes, over several polygon shapes.
// ------------------------------------------------------------------------------------------------
TEST(VertexAttributesTest, BitIdenticalIndicesForZeroThreeEightAttributes) {
    const std::vector<Polygon> polygons = {squareWithHole(), concavePolygon()};

    for (const auto& poly : polygons) {
        const auto reference = mapbox::earcut<uint32_t>(poly);
        ASSERT_FALSE(reference.empty());
        const std::size_t vertexCount = totalVertices(poly);

        for (const int attributeCount : {0, 3, 8}) {
            std::vector<std::vector<float>> buffers(static_cast<std::size_t>(attributeCount));
            for (auto& buffer : buffers) {
                buffer.resize(vertexCount * 3);
                for (std::size_t v = 0; v < vertexCount; ++v)
                    for (int e = 0; e < 3; ++e) buffer[v * 3 + e] = float(v) * 3.0f + float(e);
            }
            VertexData table = makeTable(vertexCount, buffers, 3);
            ASSERT_EQ(table.attributeCount(), static_cast<std::size_t>(attributeCount));

            const EarcutResult<uint32_t> result = mapbox::earcut<uint32_t>(poly, table);

            // Exact, bit-for-bit comparison of the whole index vector.
            ASSERT_EQ(result.indices().size(), reference.size());
            EXPECT_EQ(std::memcmp(result.indices().data(), reference.data(),
                                  reference.size() * sizeof(uint32_t)),
                      0)
                << "indices differ with " << attributeCount << " attributes";
            EXPECT_EQ(result.indices(), reference);
        }
    }
}

// Attributes come back with the index: every emitted index resolves to the exact bytes the
// caller stored for that vertex, across all slots and components (AoS and SoA layouts).
TEST(VertexAttributesTest, AttributesRoundTripWithIndices) {
    const Polygon poly = squareWithHole();
    const std::size_t vertexCount = totalVertices(poly);

    std::vector<float> normals(vertexCount * 3);
    std::vector<double> uvs(vertexCount * 2);                 // planar per-component buffers
    std::vector<double> uvU(vertexCount), uvV(vertexCount);   // true SoA: stride == elementSize
    std::vector<std::uint8_t> colors(vertexCount * 4);
    for (std::size_t v = 0; v < vertexCount; ++v) {
        for (int e = 0; e < 3; ++e) normals[v * 3 + e] = float(v) + 0.1f * float(e);
        for (int e = 0; e < 2; ++e) uvs[v * 2 + e] = double(v) + 0.25 * e;
        uvU[v] = double(v) + 0.25 * 0;
        uvV[v] = double(v) + 0.25 * 1;
        for (int e = 0; e < 4; ++e) colors[v * 4 + e] = static_cast<std::uint8_t>(v * 4 + e);
    }

    VertexData table(vertexCount);
    table.addAttribute(VertexAttribute{normals.data(), 3, sizeof(float), 3 * sizeof(float)});
    table.addAttribute(VertexAttribute{uvs.data(), 2, sizeof(double), 2 * sizeof(double)});
    table.addAttribute(VertexAttribute{uvU.data(), 1, sizeof(double), sizeof(double)}); // SoA
    table.addAttribute(VertexAttribute{uvV.data(), 1, sizeof(double), sizeof(double)}); // SoA
    table.addAttribute(VertexAttribute{colors.data(), 4, 1, 4});

    const auto result = mapbox::earcut<uint32_t>(poly, table);
    ASSERT_EQ(table.attributeCount(), 5u);
    for (std::size_t i = 0; i < result.indices().size(); ++i) {
        const std::size_t v = result.indices()[i];
        for (int e = 0; e < 3; ++e)
            EXPECT_EQ(*result.attributeAt<float>(i, 0, e), normals[v * 3 + e]);
        for (int e = 0; e < 2; ++e)
            EXPECT_EQ(*result.attributeAt<double>(i, 1, e), uvs[v * 2 + e]);
        EXPECT_EQ(*result.attributeAt<double>(i, 2, 0), uvU[v]);
        EXPECT_EQ(*result.attributeAt<double>(i, 3, 0), uvV[v]);
        for (int e = 0; e < 4; ++e)
            EXPECT_EQ(*result.attributeAt<std::uint8_t>(i, 4, e), colors[v * 4 + e]);
    }
}

// The 0-attribute table must behave exactly like the plain call and still be queryable for
// geometry-only metadata.
TEST(VertexAttributesTest, ZeroAttributeTable) {
    const Polygon poly = squareWithHole();
    VertexData table(totalVertices(poly));
    const auto result = mapbox::earcut<uint32_t>(poly, table);
    EXPECT_EQ(result.indices(), mapbox::earcut<uint32_t>(poly));
    EXPECT_EQ(result.vertexCount(), totalVertices(poly));
    EXPECT_EQ(result.vertexData()->attributeCount(), 0u);

    bool threw = false;
    try {
        (void)result.attributeAt<float>(0, 0);
    } catch (const std::logic_error&) {
        threw = true;
    }
    EXPECT_TRUE(threw);
}

// ------------------------------------------------------------------------------------------------
// Failure paths: each lists a deterministic reproduction and the expected error.
// ------------------------------------------------------------------------------------------------
TEST(VertexAttributesFailureTest, RejectsNullAttribute) {
    VertexData table(4);
    EXPECT_THROW(table.addAttribute(VertexAttribute{nullptr, 3, sizeof(float), 12}),
                 std::invalid_argument);
}

TEST(VertexAttributesFailureTest, RejectsZeroCountAndZeroSize) {
    static float backing[12];
    VertexData table(4);
    EXPECT_THROW(table.addAttribute(VertexAttribute{backing, 0, sizeof(float), 0}),
                 std::invalid_argument);
    EXPECT_THROW(table.addAttribute(VertexAttribute{backing, 3, 0, 0}), std::invalid_argument);
}

TEST(VertexAttributesFailureTest, RejectsStrideSmallerThanFootprint) {
    static float backing[12];
    VertexData table(4);
    // 3 floats per vertex need 12 bytes; a stride of 8 cannot address element 2.
    EXPECT_THROW(table.addAttribute(VertexAttribute{backing, 3, sizeof(float), 8}),
                 std::invalid_argument);
}

TEST(VertexAttributesFailureTest, RejectsTableSmallerThanPolygon) {
    const Polygon poly = squareWithHole(); // 8 vertices
    VertexData table(4);                    // only 4 covered
    EXPECT_THROW(mapbox::earcut<uint32_t>(poly, table), std::invalid_argument);
}

TEST(VertexAttributesFailureTest, RejectsOutOfRangeSlotVertexElement) {
    const Polygon poly = squareWithHole();
    const std::size_t vertexCount = totalVertices(poly);
    std::vector<float> normals(vertexCount * 3, 0.0f);
    VertexData table(vertexCount);
    table.addAttribute(VertexAttribute{normals.data(), 3, sizeof(float), 12});

    const auto result = mapbox::earcut<uint32_t>(poly, table);
    EXPECT_THROW(table.get<float>(0, 5), std::out_of_range);       // bad slot
    EXPECT_THROW(table.get<float>(vertexCount, 0), std::out_of_range); // bad vertex
    EXPECT_THROW(table.get<float>(0, 0, 3), std::out_of_range);     // bad element
    EXPECT_THROW(result.attributeAt<float>(result.indices().size(), 0),
                 std::out_of_range); // bad output position
}

// A deliberately misaligned base address (1-byte buffer) must be rejected rather than handed
// out as a float pointer.
TEST(VertexAttributesFailureTest, RejectsMisalignedAccess) {
    const Polygon poly = squareWithHole();
    const std::size_t vertexCount = totalVertices(poly);
    static std::uint8_t bytes[64];
    VertexData table(vertexCount);
    table.addAttribute(VertexAttribute{bytes + 1, 1, sizeof(float), sizeof(float)});
    EXPECT_THROW(table.get<float>(0, 0), std::invalid_argument);

    // The same table is fine when queried through a byte-wide type, which needs alignment 1.
    EXPECT_NO_THROW(table.get<std::uint8_t>(0, 0));
}

// Empty polygon keeps the historical empty-result behavior even with a table attached.
TEST(VertexAttributesTest, EmptyPolygonStaysEmpty) {
    const Polygon empty;
    VertexData table(0);
    const auto result = mapbox::earcut<uint32_t>(empty, table);
    EXPECT_TRUE(result.indices().empty());
    EXPECT_TRUE(mapbox::earcut<uint32_t>(empty).empty());
}
