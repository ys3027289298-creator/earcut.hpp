#include <gtest/gtest.h>

#include <cmath>
#include <mapbox/simplify.hpp>
#include <string>
#include <vector>

#include "fixtures/geometries.hpp"
#include "fixtures/mvt_fixture.hpp"

namespace {

using mapbox::fixtures::DoublePoint;
using mapbox::fixtures::DoublePolygon;

template <typename Vertices, typename Indices>
double meshArea(const Vertices& vertices, const Indices& indices) {
    using namespace mapbox::util;
    using Point = typename Vertices::value_type;
    double area = 0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        const Point& a = vertices[indices[i]];
        const Point& b = vertices[indices[i + 1]];
        const Point& c = vertices[indices[i + 2]];
        // cast before dividing: with integer point types, abs()/2 would truncate
        area += double(std::abs((nth<0, Point>::get(a) - nth<0, Point>::get(c)) *
                                    (nth<1, Point>::get(b) - nth<1, Point>::get(a)) -
                                (nth<0, Point>::get(a) - nth<0, Point>::get(b)) *
                                    (nth<1, Point>::get(c) - nth<1, Point>::get(a)))) /
                2;
    }
    return area;
}

template <typename Polygon>
double polyArea(const Polygon& rings) {
    using namespace mapbox::util;
    if (rings.empty()) return 0;
    double total = 0;
    for (size_t r = 0; r < rings.size(); ++r) {
        using Point = typename Polygon::value_type::value_type;
        double sum = 0;
        const auto& ring = rings[r];
        for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
            sum += (nth<0, Point>::get(ring[i]) - nth<0, Point>::get(ring[j])) *
                   (nth<1, Point>::get(ring[i]) + nth<1, Point>::get(ring[j]));
        }
        const double a = std::abs(sum) / 2;
        total = (r == 0) ? a : total - a;
    }
    return std::max(total, 0.0);
}

template <typename Ring>
double ringPerimeter(const Ring& ring) {
    using namespace mapbox::util;
    using Point = typename Ring::value_type;
    double p = 0;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        p += std::hypot(double(nth<0, Point>::get(ring[i]) - nth<0, Point>::get(ring[j])),
                        double(nth<1, Point>::get(ring[i]) - nth<1, Point>::get(ring[j])));
    }
    return p;
}

// Figure-8: the classic self-intersecting ring.
DoublePolygon figureEight() {
    return {{{0, 0}, {4, 4}, {4, 0}, {0, 4}}};
}

} // namespace

// ---- degenerate input rejection ----------------------------------------------------------

TEST(SimplifyDegenerate, ZeroAreaRingRejected) {
    DoublePolygon collinear{{{0, 0}, {1, 1}, {2, 2}}};
    try {
        mapbox::simplifyPolygon(collinear, 1.0);
        FAIL() << "expected SimplifyError";
    } catch (const mapbox::SimplifyError& e) {
        EXPECT_NE(std::string(e.what()).find("zero area"), std::string::npos) << e.what();
    }
}

TEST(SimplifyDegenerate, BacktrackingRingRejected) {
    // collinear ring that retraces itself: both zero-area and self-touching
    DoublePolygon backtracking{{{0, 0}, {1, 1}, {2, 2}, {3, 3}}};
    EXPECT_THROW(mapbox::simplifyPolygon(backtracking, 1.0), mapbox::SimplifyError);
}

TEST(SimplifyDegenerate, SelfIntersectingRingRejected) {
    try {
        mapbox::simplifyPolygon(figureEight(), 1.0);
        FAIL() << "expected SimplifyError";
    } catch (const mapbox::SimplifyError& e) {
        EXPECT_NE(std::string(e.what()).find("self-intersects"), std::string::npos) << e.what();
    }
}

TEST(SimplifyDegenerate, TooFewPointsRejected) {
    DoublePolygon twoPoints{{{0, 0}, {1, 1}}};
    EXPECT_THROW(mapbox::simplifyPolygon(twoPoints, 1.0), mapbox::SimplifyError);
}

TEST(SimplifyDegenerate, ZeroAreaHoleRejected) {
    DoublePolygon poly{
        {{0, 0}, {10, 0}, {10, 10}, {0, 10}},
        {{2, 2}, {4, 4}, {6, 6}}, // collinear hole
    };
    EXPECT_THROW(mapbox::simplifyPolygon(poly, 1.0), mapbox::SimplifyError);
}

TEST(SimplifyDegenerate, BadToleranceRejected) {
    DoublePolygon square{{{0, 0}, {10, 0}, {10, 10}, {0, 10}}};
    EXPECT_THROW(mapbox::simplifyPolygon(square, -1.0), mapbox::SimplifyError);
    EXPECT_THROW(mapbox::simplifyPolygon(square, std::nan("")), mapbox::SimplifyError);
    EXPECT_THROW((mapbox::earcut<uint32_t>(square, -0.5)), mapbox::SimplifyError);
}

TEST(SimplifyDegenerate, EarcutOverloadRejectsToo) {
    EXPECT_THROW((mapbox::earcut<uint32_t>(figureEight(), 0.0)), mapbox::SimplifyError);
    EXPECT_THROW((mapbox::earcut<uint32_t>(figureEight(), 1.0)), mapbox::SimplifyError);
}

// ---- statistics are queryable -------------------------------------------------------------

TEST(SimplifyStats, ReportsCountsAndDeviation) {
    // a square with dense collinear-ish points on each edge, plus a small bump
    DoublePolygon poly{{{0, 0},   {1, 0.1},  {2, 0},   {3, -0.1}, {4, 0},   {5, 0.1},
                        {6, 0},   {7, -0.1}, {8, 0},   {9, 0.1},  {10, 0},  {10, 1},
                        {10, 2},  {10, 3},   {10, 4},  {10, 5},   {10, 6},  {10, 7},
                        {10, 8},  {10, 9},   {10, 10}, {9, 10},   {8, 10},  {7, 10},
                        {6, 10},  {5, 10},   {4, 10},  {3, 10},   {2, 10},  {1, 10},
                        {0, 10},  {0, 9},    {0, 8},   {0, 7},    {0, 6},   {0, 5},
                        {0, 4},   {0, 3},    {0, 2},   {0, 1}}};
    const double tolerance = 0.5;
    const auto result = mapbox::simplifyPolygon(poly, tolerance);

    EXPECT_EQ(result.stats.inputPoints, 40u);
    EXPECT_LT(result.stats.outputPoints, result.stats.inputPoints);
    EXPECT_GT(result.stats.maxDeviation, 0.0);
    EXPECT_LE(result.stats.maxDeviation, tolerance + 1e-12);
    std::size_t total = 0;
    for (const auto& ring : result.rings) total += ring.size();
    EXPECT_EQ(total, result.stats.outputPoints);
}

TEST(SimplifyStats, KeptPointsStayOnOriginalOutline) {
    DoublePolygon poly{{{0, 0}, {5, 0.2}, {10, 0}, {10, 10}, {0, 10}}};
    const auto result = mapbox::simplifyPolygon(poly, 1.0);
    // every kept point must be one of the original points (simplification never invents points)
    for (const auto& p : result.rings[0]) {
        bool found = false;
        for (const auto& q : poly[0])
            if (p == q) found = true;
        EXPECT_TRUE(found);
    }
}

// ---- structural invariants ----------------------------------------------------------------

TEST(SimplifyStructure, RingsNeverDropBelowThreePoints) {
    // tiny features far below tolerance still keep a valid triangle per ring
    DoublePolygon poly{
        {{0, 0}, {100, 0}, {100, 100}, {0, 100}},
        {{40, 40}, {40.1, 40}, {40, 40.1}}, // hole smaller than tolerance
    };
    const auto result = mapbox::simplifyPolygon(poly, 10.0);
    ASSERT_EQ(result.rings.size(), 2u);
    for (const auto& ring : result.rings) EXPECT_GE(ring.size(), 3u);
}

TEST(SimplifyStructure, TriangleCountMatchesEulerFormula) {
    DoublePolygon poly{
        {{0, 0}, {50, 0.3}, {100, 0}, {100, 100}, {0, 100}},
        {{25, 25}, {75, 25}, {75, 75}, {25, 75}},
    };
    mapbox::SimplifyStats stats;
    const auto indices = mapbox::earcut<uint32_t>(poly, 1.0, &stats);
    // triangles = vertices + 2*holes - 2 for a valid polygon triangulation
    EXPECT_EQ(indices.size() / 3, stats.outputPoints + 2 * 1 - 2);
}

// ---- topology safety net --------------------------------------------------------------------

// A simple ring whose outline folds back near itself every winding: Douglas-Peucker chords can
// cut across adjacent windings and self-intersect once the tolerance reaches the gap between
// them. The pipeline must detect that and fall back to the original polygon.
static DoublePolygon spiralBand() {
    const double pi = 3.14159265358979323846;
    std::vector<DoublePoint> ring;
    const int n = 24;
    const double thMax = 4 * pi; // two turns
    for (int i = 0; i < n; ++i) {
        const double th = thMax * i / (n - 1), r = 60 + 20 * th / (2 * pi) + 4;
        ring.emplace_back(r * std::cos(th), r * std::sin(th));
    }
    for (int i = n - 1; i >= 0; --i) {
        const double th = thMax * i / (n - 1), r = 60 + 20 * th / (2 * pi) - 4;
        ring.emplace_back(r * std::cos(th), r * std::sin(th));
    }
    return {ring};
}

TEST(SimplifyFallback, TopologyBreakFallsBackToOriginal) {
    const DoublePolygon spiral = spiralBand();

    const auto result = mapbox::simplifyPolygon(spiral, 12.0);
    EXPECT_TRUE(result.stats.usedFallback);
    EXPECT_EQ(result.stats.outputPoints, result.stats.inputPoints);
    EXPECT_EQ(result.stats.maxDeviation, 0.0);
    ASSERT_EQ(result.rings.size(), 1u);
    EXPECT_EQ(result.rings[0].size(), spiral[0].size());

    // the triangulation of a fallen-back polygon is the plain earcut output
    mapbox::SimplifyStats stats;
    const auto indices = mapbox::earcut<uint32_t>(spiral, 12.0, &stats);
    EXPECT_TRUE(stats.usedFallback);
    EXPECT_EQ(indices, mapbox::earcut<uint32_t>(spiral));
}

// The same spiral simplifies fine at a smaller tolerance (no fallback needed).
TEST(SimplifyFallback, SmallToleranceSimplifies) {
    const auto result = mapbox::simplifyPolygon(spiralBand(), 10.0);
    EXPECT_FALSE(result.stats.usedFallback);
    EXPECT_LT(result.stats.outputPoints, result.stats.inputPoints);
}

// ---- tolerance 0 is bit-identical to plain earcut ------------------------------------------

TEST(SimplifyZeroTolerance, BitIdenticalOnJsonFixtures) {
    for (auto* fixture : mapbox::fixtures::FixtureTester::collection()) {
        const auto baseline = mapbox::earcut<uint32_t>(fixture->polygon());
        std::vector<uint32_t> pipelined;
        try {
            pipelined = mapbox::earcut<uint32_t>(fixture->polygon(), 0.0);
        } catch (const mapbox::SimplifyError&) {
            // degenerate fixtures are rejected by the new API; the old one stays untouched
            continue;
        }
        EXPECT_EQ(baseline, pipelined) << fixture->name << ": tolerance=0 output differs";
    }
}

TEST(SimplifyZeroTolerance, BitIdenticalOnMvtTiles) {
    const auto& features = mapbox::fixtures::mvtFixture();
    ASSERT_EQ(features.size(), 119680u);

    std::size_t compared = 0, rejected = 0;
    for (const auto& f : features) {
        const auto baseline = mapbox::earcut<uint32_t>(f.polygon);
        try {
            const auto pipelined = mapbox::earcut<uint32_t>(f.polygon, 0.0);
            ASSERT_EQ(baseline, pipelined) << "tolerance=0 output differs";
            ++compared;
        } catch (const mapbox::SimplifyError&) {
            // rejections are only acceptable where the old pipeline had nothing meaningful to
            // preserve: zero-area input
            EXPECT_EQ(polyArea(f.polygon), 0.0) << "non-degenerate polygon was rejected";
            ++rejected;
        }
    }
    EXPECT_GT(compared, 100000u);
    EXPECT_LT(rejected, 100u);
}

// ---- area deviation stays within the tolerance-derived bound --------------------------------

TEST(SimplifyArea, MvtTilesStayWithinBound) {
    const auto& features = mapbox::fixtures::mvtFixture();
    const double tolerances[] = {0.5, 1.0, 2.0};
    const double pi = 3.14159265358979323846;

    for (double tolerance : tolerances) {
        double worstRelative = 0;
        std::size_t evaluated = 0;
        for (const auto& f : features) {
            const double expectedArea = polyArea(f.polygon);
            if (expectedArea == 0) continue;

            std::vector<uint32_t> indices;
            decltype(mapbox::simplifyPolygon(f.polygon, tolerance)) simplified;
            try {
                simplified = mapbox::simplifyPolygon(f.polygon, tolerance);
                indices = mapbox::earcut<uint32_t>(simplified.rings);
            } catch (const mapbox::SimplifyError&) {
                continue;
            }

            std::vector<mapbox::fixtures::MvtPoint> verts;
            double perimeter = 0;
            std::size_t edges = 0;
            for (const auto& ring : simplified.rings) {
                perimeter += ringPerimeter(ring);
                edges += ring.size();
                for (const auto& p : ring) verts.push_back(p);
            }

            const double area = meshArea(verts, indices);
            const double absolute = std::abs(area - expectedArea);
            // theoretical bound: |dA| <= 2*t*L + pi*t^2*E (see simplify.hpp docs)
            const double bound =
                2 * tolerance * perimeter + pi * tolerance * tolerance * double(edges) + 1e-9;
            EXPECT_LE(absolute, bound) << "area drift exceeds tolerance-derived bound";
            worstRelative = std::max(worstRelative, absolute / expectedArea);
            ++evaluated;
        }
        EXPECT_GT(evaluated, 100000u);
        RecordProperty("tolerance", tolerance);
        RecordProperty("worstRelativeDeviation", worstRelative);
    }
}

TEST(SimplifyArea, MaxDeviationNeverExceedsTolerance) {
    const auto& features = mapbox::fixtures::mvtFixture();
    const double tolerances[] = {0.5, 1.0, 2.0};
    for (double tolerance : tolerances) {
        for (const auto& f : features) {
            mapbox::SimplifyStats stats;
            try {
                mapbox::earcut<uint32_t>(f.polygon, tolerance, &stats);
            } catch (const mapbox::SimplifyError&) {
                continue;
            }
            EXPECT_LE(stats.maxDeviation, tolerance + 1e-9);
            EXPECT_LE(stats.outputPoints, stats.inputPoints);
        }
    }
}
