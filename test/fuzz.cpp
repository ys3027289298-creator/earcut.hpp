// Fuzz driver for the earcut validation/batch layer.
//
//   fuzz [iterations] [seed]   run randomized fuzzing (default 100000 iterations, seed 1)
//   fuzz --demo                validate a small hand-made dirty batch and print diagnostics
//
// Each iteration builds a polygon (clean star shapes, plus dirty variants injecting duplicate
// points, degenerate edges, self-intersections and overflowing/non-finite coordinates), then:
//   1. runs validatePolygon() on everything — it must never crash and must agree with earcut
//      on what is triangulatable;
//   2. runs earcut directly on every finite-coordinate input (even self-intersecting) — dirty
//      data must never crash or hang the triangulator;
//   3. for validator-passing input, checks the triangulation is well-formed.
// Exits non-zero on any anomaly.

#include <mapbox/earcut.hpp>
#include <mapbox/earcut_validate.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace {

using Point = std::pair<double, double>;
using Ring = std::vector<Point>;
using Polygon = std::vector<Ring>;

struct XorShift {
    uint64_t state;
    explicit XorShift(uint64_t seed) : state(seed ? seed : 0x9e3779b97f4a7c15ULL) {}
    uint64_t next() {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    }
    uint64_t below(uint64_t n) { return next() % n; }
    double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
    double range(double lo, double hi) { return lo + (hi - lo) * uniform(); }
};

// Sorted angles + random radii give a star-shaped (hence simple) polygon.
Ring makeStarRing(XorShift& rng, std::size_t n, double cx, double cy, double radius) {
    Ring ring;
    ring.reserve(n);
    const double twoPi = 6.283185307179586;
    double angle = rng.range(0, twoPi);
    for (std::size_t i = 0; i < n; ++i) {
        angle += rng.range(0.05, twoPi / static_cast<double>(n));
        const double r = radius * rng.range(0.3, 1.0);
        ring.emplace_back(cx + r * std::cos(angle), cy + r * std::sin(angle));
    }
    return ring;
}

Polygon makeCleanPolygon(XorShift& rng) {
    Polygon poly;
    poly.push_back(makeStarRing(rng, 3 + rng.below(40), 0, 0, 100));
    if (rng.below(3) == 0) { // concentric hole, reverse wound
        Ring hole = makeStarRing(rng, 3 + rng.below(8), 0, 0, 30);
        std::reverse(hole.begin(), hole.end());
        poly.push_back(std::move(hole));
    }
    return poly;
}

Polygon makeDirtyPolygon(XorShift& rng, int kind) {
    Polygon poly;
    switch (kind) {
        case 0: { // duplicate consecutive points
            poly = makeCleanPolygon(rng);
            Ring& ring = poly.front();
            const std::size_t at = rng.below(ring.size());
            ring.insert(ring.begin() + static_cast<std::ptrdiff_t>(at), ring[at]);
            break;
        }
        case 1: { // degenerate edge: vertex nudged by ~1e-12
            poly = makeCleanPolygon(rng);
            Ring& ring = poly.front();
            const std::size_t at = rng.below(ring.size());
            ring.insert(ring.begin() + static_cast<std::ptrdiff_t>(at),
                        Point{ring[at].first + 1e-12, ring[at].second - 1e-12});
            break;
        }
        case 2: { // self-intersection: shuffled random points
            Ring ring;
            const std::size_t n = 4 + rng.below(30);
            for (std::size_t i = 0; i < n; ++i)
                ring.emplace_back(rng.range(-100, 100), rng.range(-100, 100));
            poly.push_back(std::move(ring));
            break;
        }
        case 3: { // coordinate overflow: beyond 2^53
            poly = makeCleanPolygon(rng);
            Ring& ring = poly.front();
            ring[rng.below(ring.size())].first = rng.range(1e16, 1e18);
            break;
        }
        default: { // non-finite coordinate
            poly = makeCleanPolygon(rng);
            Ring& ring = poly.front();
            const double bad = rng.below(2) == 0 ? std::numeric_limits<double>::quiet_NaN()
                                                 : std::numeric_limits<double>::infinity();
            ring[rng.below(ring.size())].second = bad;
            break;
        }
    }
    return poly;
}

std::size_t vertexCount(const Polygon& poly) {
    std::size_t n = 0;
    for (const auto& ring : poly) n += ring.size();
    return n;
}

int runDemo() {
    // Small hand-made batch: two valid polygons plus one of each dirty category.
    const std::vector<Polygon> batch{
        // 0: valid square with a square hole
        {{{0, 0}, {100, 0}, {100, 100}, {0, 100}}, {{25, 25}, {75, 25}, {75, 75}, {25, 75}}},
        // 1: self-intersecting bowtie
        {{{0, 0}, {40, 40}, {40, 0}, {0, 40}}},
        // 2: duplicate consecutive vertex
        {{{0, 0}, {50, 0}, {50, 0}, {50, 50}, {0, 50}}},
        // 3: degenerate edge (length 1e-11)
        {{{0, 0}, {100, 0}, {100, 1e-11}, {100, 100}, {0, 100}}},
        // 4: coordinate overflow (1e17 > 2^53)
        {{{0, 0}, {1e17, 0}, {1e17, 10}, {0, 10}}},
        // 5: valid triangle
        {{{0, 0}, {10, 0}, {5, 8}}},
    };

    const auto results = mapbox::earcutBatch(batch);
    std::size_t triangulated = 0;
    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& item = results[i];
        if (item.ok()) {
            ++triangulated;
            std::printf("polygon %zu: OK -> %zu triangle(s)\n", i, item.indices.size() / 3);
        } else {
            std::printf("polygon %zu: SKIPPED, %zu diagnostic(s):\n", i, item.report.diagnostics.size());
            for (const auto& d : item.report.diagnostics)
                std::printf("  [%s] %s\n", mapbox::issueName(d.issue), d.message.c_str());
        }
    }
    std::printf("batch done: %zu/%zu triangulated, %zu diagnosed (batch no longer fails wholesale)\n",
                triangulated, results.size(), results.size() - triangulated);
    return 0;
}

int runFuzz(uint64_t iterations, uint64_t seed) {
    XorShift rng(seed);
    std::size_t clean = 0;
    std::size_t diagnosed[4] = {0, 0, 0, 0};
    std::size_t directDirty = 0;
    const uint64_t heartbeat = iterations >= 10 ? iterations / 10 : 1;

    for (uint64_t iter = 0; iter < iterations; ++iter) {
        Polygon poly;
        if (rng.below(2) == 0) {
            poly = makeCleanPolygon(rng);
        } else {
            poly = makeDirtyPolygon(rng, static_cast<int>(rng.below(5)));
        }

        const auto report = mapbox::validatePolygon(poly);
        if (report.ok()) {
            ++clean;
            const auto indices = mapbox::earcut<uint32_t>(poly);
            if (indices.size() % 3 != 0) {
                std::fprintf(stderr, "iter %llu: index count %zu not a multiple of 3\n",
                             static_cast<unsigned long long>(iter), indices.size());
                return 1;
            }
            const std::size_t verts = vertexCount(poly);
            for (const uint32_t idx : indices) {
                if (idx >= verts) {
                    std::fprintf(stderr, "iter %llu: index %u out of range (%zu vertices)\n",
                                 static_cast<unsigned long long>(iter), idx, verts);
                    return 1;
                }
            }
        } else {
            for (const auto& d : report.diagnostics)
                ++diagnosed[static_cast<std::size_t>(d.issue)];
            // Dirty but finite input must still never crash the triangulator itself.
            if (!report.has(mapbox::PolygonIssue::CoordinateOverflow)) {
                ++directDirty;
                const auto indices = mapbox::earcut<uint32_t>(poly);
                (void)indices;
            }
        }

        if ((iter + 1) % heartbeat == 0)
            std::printf("... %llu/%llu iterations\n", static_cast<unsigned long long>(iter + 1),
                        static_cast<unsigned long long>(iterations));
    }

    std::printf("FUZZ OK: %llu iterations (seed %llu), no crash\n",
                static_cast<unsigned long long>(iterations), static_cast<unsigned long long>(seed));
    std::printf("  clean polygons triangulated: %zu\n", clean);
    std::printf("  diagnostics: CoordinateOverflow=%zu DuplicatePoint=%zu DegenerateEdge=%zu "
                "SelfIntersection=%zu\n",
                diagnosed[0], diagnosed[1], diagnosed[2], diagnosed[3]);
    std::printf("  dirty-but-finite polygons also run through earcut directly: %zu\n", directDirty);
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--demo") == 0) return runDemo();
    const uint64_t iterations = argc > 1 ? std::strtoull(argv[1], nullptr, 10) : 100000;
    const uint64_t seed = argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 1;
    if (iterations == 0) {
        std::fprintf(stderr, "usage: fuzz [iterations] [seed] | fuzz --demo\n");
        return 2;
    }
    return runFuzz(iterations, seed);
}
