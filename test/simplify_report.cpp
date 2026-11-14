// simplify_report: generates the acceptance data for the contour-simplification preprocessing
// pipeline on one shared input set (the 119,680-polygon MVT fixture + the JSON fixtures):
//
//   1. before/after comparison at 3 tolerance levels (points, triangles, area deviation, time,
//      output size)
//   2. bit-exact consistency check: tolerance=0 output vs. plain earcut()
//   3. failure-path demo: degenerate inputs and the errors they produce
//
// Build: cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target simplify_report
// Run:   ./build/simplify_report

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <mapbox/simplify.hpp>
#include <string>
#include <vector>

#include "fixtures/geometries.hpp"
#include "fixtures/mvt_fixture.hpp"

namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(Clock::time_point start) {
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

template <typename Vertices, typename Indices>
double meshArea(const Vertices& vertices, const Indices& indices) {
    double area = 0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        const auto& a = vertices[indices[i]];
        const auto& b = vertices[indices[i + 1]];
        const auto& c = vertices[indices[i + 2]];
        // cast before dividing: with integer point types, abs()/2 would truncate
        area += double(std::abs((a.first - c.first) * (b.second - a.second) -
                                (a.first - b.first) * (c.second - a.second))) /
                2;
    }
    return area;
}

template <typename Polygon>
double polyArea(const Polygon& rings) {
    double total = 0;
    for (size_t r = 0; r < rings.size(); ++r) {
        double sum = 0;
        const auto& ring = rings[r];
        for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
            sum += double(ring[i].first - ring[j].first) * double(ring[i].second + ring[j].second);
        }
        const double a = std::abs(sum) / 2;
        total = (r == 0) ? a : total - a;
    }
    return std::max(total, 0.0);
}

struct RunStats {
    std::size_t inputPoints = 0;
    std::size_t outputPoints = 0;
    std::size_t triangles = 0;
    std::size_t indexBytes = 0;
    std::size_t rejected = 0;
    std::size_t fallbacks = 0;
    double maxPointDeviation = 0;
    double maxRelativeAreaDeviation = 0;
    double maxAbsoluteAreaDeviation = 0;
    double sumRelativeAreaDeviation = 0;
    std::size_t areaEvaluated = 0;
    double simplifyMillis = 0;
    double earcutMillis = 0;
};

template <typename Polygon>
void accumulatePolygon(RunStats& stats, const Polygon& polygon, double tolerance,
                       std::vector<typename Polygon::value_type::value_type>& verts) {
    std::size_t inPoints = 0;
    for (const auto& ring : polygon) inPoints += ring.size();
    stats.inputPoints += inPoints;

    mapbox::SimplifyStats simplifyStats;
    std::vector<uint32_t> indices;
    verts.clear();
    auto start = Clock::now();
    try {
        if (tolerance == 0.0) {
            indices = mapbox::earcut<uint32_t>(polygon, 0.0, &simplifyStats);
            for (const auto& ring : polygon)
                for (const auto& p : ring) verts.push_back(p);
        } else {
            const auto simplified = mapbox::simplifyPolygon(polygon, tolerance);
            simplifyStats = simplified.stats;
            stats.simplifyMillis += elapsedMs(start);
            start = Clock::now();
            indices = mapbox::earcut<uint32_t>(simplified.rings);
            for (const auto& ring : simplified.rings)
                for (const auto& p : ring) verts.push_back(p);
        }
    } catch (const mapbox::SimplifyError&) {
        ++stats.rejected;
        return;
    }
    stats.earcutMillis += elapsedMs(start);
    stats.outputPoints += simplifyStats.outputPoints;
    stats.triangles += indices.size() / 3;
    stats.indexBytes += indices.size() * sizeof(uint32_t);
    stats.maxPointDeviation = std::max(stats.maxPointDeviation, simplifyStats.maxDeviation);
    if (simplifyStats.usedFallback) ++stats.fallbacks;

    const double expectedArea = polyArea(polygon);
    if (expectedArea == 0) return;
    const double absolute = std::abs(meshArea(verts, indices) - expectedArea);
    stats.maxAbsoluteAreaDeviation = std::max(stats.maxAbsoluteAreaDeviation, absolute);
    stats.maxRelativeAreaDeviation = std::max(stats.maxRelativeAreaDeviation, absolute / expectedArea);
    stats.sumRelativeAreaDeviation += absolute / expectedArea;
    ++stats.areaEvaluated;
}

RunStats runPipeline(double tolerance) {
    const auto& features = mapbox::fixtures::mvtFixture();
    RunStats stats;
    std::vector<mapbox::fixtures::MvtPoint> verts;
    for (const auto& f : features) {
        accumulatePolygon(stats, f.polygon, tolerance, verts);
    }
    return stats;
}

RunStats runBaseline() {
    const auto& features = mapbox::fixtures::mvtFixture();
    RunStats stats;
    std::vector<mapbox::fixtures::MvtPoint> verts;
    for (const auto& f : features) {
        std::size_t inPoints = 0;
        for (const auto& ring : f.polygon) inPoints += ring.size();
        stats.inputPoints += inPoints;
        stats.outputPoints += inPoints;

        const auto start = Clock::now();
        const auto indices = mapbox::earcut<uint32_t>(f.polygon);
        stats.earcutMillis += elapsedMs(start);
        stats.triangles += indices.size() / 3;
        stats.indexBytes += indices.size() * sizeof(uint32_t);

        const double expectedArea = polyArea(f.polygon);
        if (expectedArea == 0) continue;
        verts.clear();
        for (const auto& ring : f.polygon)
            for (const auto& p : ring) verts.push_back(p);
        const double deviation = std::abs(meshArea(verts, indices) - expectedArea) / expectedArea;
        stats.maxRelativeAreaDeviation = std::max(stats.maxRelativeAreaDeviation, deviation);
        stats.sumRelativeAreaDeviation += deviation;
        ++stats.areaEvaluated;
    }
    return stats;
}

void printRow(const std::string& label, const RunStats& s) {
    std::cout << std::left << std::setw(14) << label << std::right << std::setw(10) << s.inputPoints
              << std::setw(10) << s.outputPoints << std::setw(10) << s.triangles << std::setw(10)
              << (s.indexBytes / 1024) << std::setw(12) << s.maxPointDeviation << std::setw(14)
              << s.maxAbsoluteAreaDeviation << std::setw(14) << s.maxRelativeAreaDeviation << std::setw(14)
              << (s.areaEvaluated ? s.sumRelativeAreaDeviation / double(s.areaEvaluated) : 0.0) << std::setw(8)
              << s.rejected << std::setw(8) << s.fallbacks << std::setw(10) << s.simplifyMillis
              << std::setw(10) << s.earcutMillis << "\n";
}

void printHeader() {
    std::cout << std::left << std::setw(14) << "run" << std::right << std::setw(10) << "pts_in"
              << std::setw(10) << "pts_out" << std::setw(10) << "triangles" << std::setw(10) << "KiB_idx"
              << std::setw(12) << "max_pt_dev" << std::setw(14) << "max_abs_area" << std::setw(14)
              << "max_rel_area" << std::setw(14) << "mean_rel_area" << std::setw(8) << "reject"
              << std::setw(8) << "fallback"
              << std::setw(10) << "simpl_ms" << std::setw(10) << "ecut_ms" << "\n";
}

// A dense imported-contour stand-in: 100k-point coastline-like ring (star-shaped, so simple by
// construction) with multi-frequency radial detail plus fine jitter — the "too many vertices"
// regime the simplification preprocessing is for.
std::vector<std::pair<double, double>> denseContour(std::size_t n) {
    const double pi = 3.14159265358979323846;
    std::vector<std::pair<double, double>> ring;
    ring.reserve(n);
    unsigned rng = 12345;
    for (std::size_t i = 0; i < n; ++i) {
        const double th = 2 * pi * double(i) / double(n);
        rng = rng * 1664525u + 1013904223u;
        const double jitter = (double(rng >> 8) / 16777216.0 - 0.5) * 4.0; // +/-2
        const double r = 1000 + 200 * std::sin(3 * th) + 80 * std::sin(17 * th) +
                         25 * std::sin(61 * th + 1.0) + jitter;
        ring.emplace_back(r * std::cos(th), r * std::sin(th));
    }
    return ring;
}

RunStats runDense(const std::vector<std::vector<std::pair<double, double>>>& polygon, double tolerance) {
    RunStats stats;
    std::vector<std::pair<double, double>> verts;
    accumulatePolygon(stats, polygon, tolerance, verts);
    return stats;
}

} // namespace

int main() {
    const auto& features = mapbox::fixtures::mvtFixture();
    std::cout << "== simplify pipeline report ==\n";
    std::cout << "input set: MVT tiles fixture, " << features.size() << " polygons\n\n";

    std::cout << "-- 1. before/after comparison (same input set) --\n";
    printHeader();
    const auto baseline = runBaseline();
    printRow("baseline", baseline);
    const double tolerances[] = {0.5, 1.0, 2.0};
    RunStats runs[3];
    for (int i = 0; i < 3; ++i) {
        runs[i] = runPipeline(tolerances[i]);
        printRow("tol=" + std::to_string(tolerances[i]).substr(0, 3), runs[i]);
    }
    std::cout << "(pts_in/pts_out = contour vertices, KiB_idx = uint32 index output size,\n"
                 " max_pt_dev = max removed-point distance, max_abs/rel_area = max absolute/relative\n"
                 " triangulated-area deviation vs. original polygon (rel. deviation is dominated by\n"
                 " polygons smaller than the tolerance), reject = degenerate polygons rejected,\n"
                 " fallback = polygons where simplification broke topology and input was kept,\n"
                 " simpl_ms = validate+simplify time, ecut_ms = triangulation time)\n\n";

    std::cout << "-- 1b. dense contour (100k-point ring, the target regime) --\n";
    const std::vector<std::vector<std::pair<double, double>>> dense{denseContour(100000)};
    printHeader();
    {
        RunStats denseBase;
        std::vector<std::pair<double, double>> verts;
        std::size_t inPoints = dense[0].size();
        denseBase.inputPoints = denseBase.outputPoints = inPoints;
        const auto start = Clock::now();
        const auto indices = mapbox::earcut<uint32_t>(dense);
        denseBase.earcutMillis = elapsedMs(start);
        denseBase.triangles = indices.size() / 3;
        denseBase.indexBytes = indices.size() * sizeof(uint32_t);
        for (const auto& p : dense[0]) verts.push_back(p);
        denseBase.maxAbsoluteAreaDeviation = std::abs(meshArea(verts, indices) - polyArea(dense));
        denseBase.maxRelativeAreaDeviation = denseBase.maxAbsoluteAreaDeviation / polyArea(dense);
        denseBase.areaEvaluated = 1;
        printRow("baseline", denseBase);
    }
    for (double tolerance : {0.5, 1.0, 2.0}) {
        const auto s = runDense(dense, tolerance);
        printRow("tol=" + std::to_string(tolerance).substr(0, 3), s);
    }
    std::cout << "\n";

    std::cout << "-- 2. bit-exact consistency at tolerance=0 --\n";
    std::size_t compared = 0, mismatched = 0, rejected = 0;
    for (const auto& f : features) {
        const auto expected = mapbox::earcut<uint32_t>(f.polygon);
        try {
            const auto actual = mapbox::earcut<uint32_t>(f.polygon, 0.0);
            if (actual != expected) ++mismatched;
            ++compared;
        } catch (const mapbox::SimplifyError&) {
            ++rejected;
        }
    }
    std::cout << "MVT tiles: " << compared << " polygons compared, " << mismatched
              << " mismatched, " << rejected << " rejected as degenerate\n";
    std::size_t fixtureCompared = 0, fixtureMismatched = 0, fixtureRejected = 0;
    for (const auto* fixture : mapbox::fixtures::FixtureTester::collection()) {
        const auto expected = mapbox::earcut<uint32_t>(fixture->polygon());
        try {
            const auto actual = mapbox::earcut<uint32_t>(fixture->polygon(), 0.0);
            if (actual != expected) ++fixtureMismatched;
            ++fixtureCompared;
        } catch (const mapbox::SimplifyError&) {
            ++fixtureRejected;
        }
    }
    std::cout << "JSON fixtures: " << fixtureCompared << " polygons compared, " << fixtureMismatched
              << " mismatched, " << fixtureRejected << " rejected as degenerate\n";
    std::cout << (mismatched == 0 && fixtureMismatched == 0 ? "BIT-EXACT: PASS\n\n"
                                                            : "BIT-EXACT: FAIL\n\n");

    std::cout << "-- 3. failure paths (degenerate input) --\n";
    using mapbox::fixtures::DoublePolygon;
    const std::pair<std::string, DoublePolygon> cases[] = {
        {"zero-area (collinear) ring", {{{0, 0}, {1, 1}, {2, 2}}}},
        {"backtracking collinear ring", {{{0, 0}, {1, 1}, {2, 2}, {3, 3}}}},
        {"self-intersecting figure-8", {{{0, 0}, {4, 4}, {4, 0}, {0, 4}}}},
        {"ring with 2 points", {{{0, 0}, {1, 1}}}},
        {"zero-area hole", {{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, {{2, 2}, {4, 4}, {6, 6}}}},
    };
    for (const auto& c : cases) {
        try {
            mapbox::earcut<uint32_t>(c.second, 1.0);
            std::cout << "  [" << c.first << "] NO ERROR (unexpected)\n";
        } catch (const mapbox::SimplifyError& e) {
            std::cout << "  [" << c.first << "] SimplifyError: " << e.what() << "\n";
        }
    }
    try {
        DoublePolygon square{{{0, 0}, {10, 0}, {10, 10}, {0, 10}}};
        mapbox::earcut<uint32_t>(square, -1.0);
        std::cout << "  [negative tolerance] NO ERROR (unexpected)\n";
    } catch (const mapbox::SimplifyError& e) {
        std::cout << "  [negative tolerance] SimplifyError: " << e.what() << "\n";
    }
    std::cout << "\n";

    std::cout << "-- 4. resource bounds --\n";
    std::cout << "validation: min-x sweep, near O(E) typical / O(E^2) worst per ring, O(E) memory\n"
                 "simplify:   O(E log E) average / O(E^2) worst time, <= ~33 bytes/point extra\n"
                 "            (2x8B coords + 1B keep-mask + 8B index + <=16B/kept-point stack)\n"
                 "output:     simplified polygon <= input size; earcut allocates O(kept) nodes\n";
    return 0;
}
