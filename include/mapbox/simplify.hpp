#pragma once

#include "earcut.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace mapbox {

// Statistics of a simplifyPolygon() / earcut(poly, tolerance) call, so callers can inspect what
// the simplification did: how many points went in, how many survived, and the largest distance
// any removed point had from the simplified outline.
struct SimplifyStats {
    std::size_t inputPoints = 0;   // total vertices across all rings, before simplification
    std::size_t outputPoints = 0;  // total vertices kept after simplification
    double maxDeviation = 0.0;     // max distance of a removed point to the simplified outline
    bool usedFallback = false;     // true when simplification broke topology and the original
                                   // polygon was used instead (outputPoints == inputPoints then)
};

// Thrown by simplifyPolygon() and earcut(poly, tolerance) on degenerate input: a ring with fewer
// than 3 points, a zero-area ring, or a self-intersecting ring. Derives from std::invalid_argument
// so existing catch clauses for invalid arguments keep working.
class SimplifyError : public std::invalid_argument {
public:
    explicit SimplifyError(const std::string& message) : std::invalid_argument(message) {}
};

namespace detail {

template <typename Point>
inline double coordX(const Point& p) {
    return static_cast<double>(util::nth<0, Point>::get(p));
}
template <typename Point>
inline double coordY(const Point& p) {
    return static_cast<double>(util::nth<1, Point>::get(p));
}

// Squared distance from p to the segment a-b (a == b is handled as a point).
inline double sqSegmentDistance(double px, double py, double ax, double ay, double bx, double by) {
    double dx = bx - ax;
    double dy = by - ay;
    if (dx != 0.0 || dy != 0.0) {
        const double t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy);
        if (t > 1.0) {
            ax = bx;
            ay = by;
        } else if (t > 0.0) {
            ax += dx * t;
            ay += dy * t;
        }
    }
    dx = px - ax;
    dy = py - ay;
    return dx * dx + dy * dy;
}

inline double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// Signed doubled area of a ring (shoelace formula).
template <typename Ring>
double signedRingArea(const Ring& ring) {
    const std::size_t m = ring.size();
    double sum = 0.0;
    for (std::size_t i = 0, j = m - 1; i < m; j = i++) {
        sum += (coordX(ring[j]) - coordX(ring[i])) * (coordY(ring[j]) + coordY(ring[i]));
    }
    return sum / 2.0;
}

// True when segments (a1,a2) and (b1,b2) touch or cross, endpoint contact included.
inline bool segmentsIntersect(double a1x, double a1y, double a2x, double a2y, double b1x, double b1y,
                              double b2x, double b2y) {
    if (std::max(a1x, a2x) < std::min(b1x, b2x) || std::max(b1x, b2x) < std::min(a1x, a2x) ||
        std::max(a1y, a2y) < std::min(b1y, b2y) || std::max(b1y, b2y) < std::min(a1y, a2y)) {
        return false;
    }
    const double d1 = orient2d(b1x, b1y, b2x, b2y, a1x, a1y);
    const double d2 = orient2d(b1x, b1y, b2x, b2y, a2x, a2y);
    const double d3 = orient2d(a1x, a1y, a2x, a2y, b1x, b1y);
    const double d4 = orient2d(a1x, a1y, a2x, a2y, b2x, b2y);
    return ((d1 > 0) != (d2 > 0) || d1 == 0.0 || d2 == 0.0) && ((d3 > 0) != (d4 > 0) || d3 == 0.0 || d4 == 0.0);
}

// True when any two non-adjacent edges of the ring touch or cross. Small rings use a brute-force
// O(m^2) scan; larger rings sweep edges sorted by min-x so only x-overlapping pairs are tested —
// near O(m) on typical outlines, O(m^2) worst case (pathological x-overlap). Validation is
// opt-in via the tolerance-taking entry points; plain earcut() never pays for it.
template <typename Ring>
bool ringSelfIntersects(const Ring& ring) {
    const std::size_t m = ring.size();
    if (m < 4) return false;

    std::vector<double> xs(m), ys(m);
    for (std::size_t i = 0; i < m; ++i) {
        xs[i] = coordX(ring[i]);
        ys[i] = coordY(ring[i]);
    }
    const auto intersects = [&](std::size_t i, std::size_t j) {
        const std::size_t i2 = (i + 1) % m, j2 = (j + 1) % m;
        if (j == i2 || i == j2) return false; // adjacent edges share a vertex by construction
        return segmentsIntersect(xs[i], ys[i], xs[i2], ys[i2], xs[j], ys[j], xs[j2], ys[j2]);
    };

    if (m <= 64) {
        for (std::size_t i = 0; i < m; ++i)
            for (std::size_t j = i + 1; j < m; ++j)
                if (intersects(i, j)) return true;
        return false;
    }

    struct EdgeSpan {
        double minX, maxX, minY, maxY;
        std::size_t i;
    };
    std::vector<EdgeSpan> edges(m);
    for (std::size_t i = 0; i < m; ++i) {
        const std::size_t i2 = (i + 1) % m;
        edges[i] = {std::min(xs[i], xs[i2]), std::max(xs[i], xs[i2]),
                    std::min(ys[i], ys[i2]), std::max(ys[i], ys[i2]), i};
    }
    std::sort(edges.begin(), edges.end(),
              [](const EdgeSpan& a, const EdgeSpan& b) { return a.minX < b.minX; });

    std::vector<std::size_t> active; // positions in `edges` whose x-span still overlaps
    for (std::size_t e = 0; e < m; ++e) {
        const EdgeSpan& cur = edges[e];
        active.erase(std::remove_if(active.begin(), active.end(),
                                    [&](std::size_t a) { return edges[a].maxX < cur.minX; }),
                     active.end());
        for (const std::size_t a : active) {
            const EdgeSpan& other = edges[a];
            if (other.minY > cur.maxY || cur.minY > other.maxY) continue;
            if (intersects(other.i, cur.i)) return true;
        }
        active.push_back(e);
    }
    return false;
}

// True when any edge of ring a touches or crosses any edge of ring b. O(Ea*Eb) with whole-ring
// bounding-box rejection.
template <typename RingA, typename RingB>
bool ringsCross(const RingA& a, const RingB& b) {
    const std::size_t ma = a.size(), mb = b.size();
    if (ma == 0 || mb == 0) return false;
    double aMinX = coordX(a[0]), aMaxX = aMinX, aMinY = coordY(a[0]), aMaxY = aMinY;
    for (std::size_t i = 1; i < ma; ++i) {
        aMinX = std::min(aMinX, coordX(a[i]));
        aMaxX = std::max(aMaxX, coordX(a[i]));
        aMinY = std::min(aMinY, coordY(a[i]));
        aMaxY = std::max(aMaxY, coordY(a[i]));
    }
    double bMinX = coordX(b[0]), bMaxX = bMinX, bMinY = coordY(b[0]), bMaxY = bMinY;
    for (std::size_t i = 1; i < mb; ++i) {
        bMinX = std::min(bMinX, coordX(b[i]));
        bMaxX = std::max(bMaxX, coordX(b[i]));
        bMinY = std::min(bMinY, coordY(b[i]));
        bMaxY = std::max(bMaxY, coordY(b[i]));
    }
    if (aMaxX < bMinX || bMaxX < aMinX || aMaxY < bMinY || bMaxY < aMinY) return false;
    for (std::size_t i = 0; i < ma; ++i) {
        const std::size_t i2 = (i + 1) % ma;
        for (std::size_t j = 0; j < mb; ++j) {
            const std::size_t j2 = (j + 1) % mb;
            if (segmentsIntersect(coordX(a[i]), coordY(a[i]), coordX(a[i2]), coordY(a[i2]), coordX(b[j]),
                                  coordY(b[j]), coordX(b[j2]), coordY(b[j2]))) {
                return true;
            }
        }
    }
    return false;
}

// Rejects degenerate input with SimplifyError: rings with fewer than 3 points, self-intersecting
// rings, and zero-area rings.
template <typename Polygon>
void validatePolygon(const Polygon& poly) {
    const std::size_t ringCount = poly.size();
    for (std::size_t r = 0; r < ringCount; ++r) {
        const auto& ring = poly[r];
        if (ring.size() < 3) {
            throw SimplifyError("mapbox: ring " + std::to_string(r) + " has fewer than 3 points (" +
                                std::to_string(ring.size()) + ")");
        }
        if (ringSelfIntersects(ring)) {
            throw SimplifyError("mapbox: ring " + std::to_string(r) + " self-intersects");
        }
        if (signedRingArea(ring) == 0.0) {
            throw SimplifyError("mapbox: ring " + std::to_string(r) + " has zero area");
        }
    }
}

// Even-odd point-in-ring test. Only used on rings known not to touch the point's ring, so
// boundary cases cannot occur.
template <typename Ring>
bool pointInRing(const Ring& ring, double px, double py) {
    bool inside = false;
    const std::size_t m = ring.size();
    for (std::size_t i = 0, j = m - 1; i < m; j = i++) {
        const double xi = coordX(ring[i]), yi = coordY(ring[i]);
        const double xj = coordX(ring[j]), yj = coordY(ring[j]);
        if ((yi > py) != (yj > py) && px < xi + (py - yi) * (xj - xi) / (yj - yi)) {
            inside = !inside;
        }
    }
    return inside;
}

// Douglas-Peucker keeps a subset of the vertices in order, which guarantees the per-ring area
// bound — but it does NOT guarantee the simplified rings stay simple or keep their nesting
// (a chord can cut across a distant part of the outline, a hole can drift onto the outer ring).
// This re-validates the simplified polygon: every ring simple and non-zero-area, holes strictly
// inside the outer ring, holes not crossing or containing each other.
template <typename Rings>
bool simplifiedPolygonValid(const Rings& rings) {
    const std::size_t n = rings.size();
    for (std::size_t r = 0; r < n; ++r) {
        if (signedRingArea(rings[r]) == 0.0 || ringSelfIntersects(rings[r])) return false;
    }
    for (std::size_t h = 1; h < n; ++h) {
        if (rings[h].empty()) return false;
        if (ringsCross(rings[0], rings[h]) ||
            !pointInRing(rings[0], coordX(rings[h][0]), coordY(rings[h][0]))) {
            return false;
        }
        for (std::size_t h2 = h + 1; h2 < n; ++h2) {
            if (ringsCross(rings[h], rings[h2]) ||
                pointInRing(rings[h], coordX(rings[h2][0]), coordY(rings[h2][0])) ||
                pointInRing(rings[h2], coordX(rings[h][0]), coordY(rings[h][0]))) {
                return false;
            }
        }
    }
    return true;
}

// Douglas-Peucker simplification of one closed ring. Fills `keep` (size m) with 1 for surviving
// vertices and returns the largest distance of a removed point to the simplified outline.
// The ring must be non-degenerate (validated): it always keeps at least 3 non-collinear points.
//
// Memory: xs/ys 2*8 bytes/point, keep 1 byte/point, seq 8 bytes/point, DP stack at most one
// 16-byte frame per kept point. Peak extra usage is therefore ~33 bytes per input point.
template <typename Ring>
double simplifyRing(const Ring& ring, double sqTolerance, std::vector<char>& keep) {
    const std::size_t m = ring.size();
    keep.assign(m, 1);
    if (m <= 3 || sqTolerance == 0.0) return 0.0; // never drop below a triangle

    std::vector<double> xs(m), ys(m);
    for (std::size_t i = 0; i < m; ++i) {
        xs[i] = coordX(ring[i]);
        ys[i] = coordY(ring[i]);
    }

    // Closed-ring Douglas-Peucker: anchor at vertex 0 and at the vertex farthest from it, then
    // simplify the two polylines between the anchors. `seq` linearizes the ring as
    // 0,1,...,m-1,0 so both halves are plain index ranges into it.
    std::vector<std::size_t> seq(m + 1);
    for (std::size_t i = 0; i <= m; ++i) seq[i] = i % m;

    std::size_t anchor = 0;
    double maxDist = 0.0;
    for (std::size_t i = 1; i < m; ++i) {
        const double dx = xs[i] - xs[0], dy = ys[i] - ys[0];
        const double d = dx * dx + dy * dy;
        if (d > maxDist) {
            maxDist = d;
            anchor = i;
        }
    }

    keep.assign(m, 0);
    keep[0] = 1;
    keep[anchor] = 1;

    double maxRemovedSq = 0.0;
    std::vector<std::pair<std::size_t, std::size_t>> stack;
    stack.emplace_back(0, anchor);
    stack.emplace_back(anchor, m);
    while (!stack.empty()) {
        const std::size_t first = stack.back().first;
        const std::size_t last = stack.back().second;
        stack.pop_back();

        double maxSq = 0.0;
        std::size_t index = first;
        for (std::size_t i = first + 1; i < last; ++i) {
            const double d = sqSegmentDistance(xs[seq[i]], ys[seq[i]], xs[seq[first]], ys[seq[first]],
                                               xs[seq[last]], ys[seq[last]]);
            if (d > maxSq) {
                maxSq = d;
                index = i;
            }
        }
        if (maxSq > sqTolerance) {
            keep[seq[index]] = 1;
            stack.emplace_back(first, index);
            stack.emplace_back(index, last);
        } else if (maxSq > maxRemovedSq) {
            maxRemovedSq = maxSq; // interior points of a discarded range are removed for good
        }
    }

    // Guarantee at least 3 kept points: promote the removed point farthest from the anchor
    // segment. A validated ring has non-zero area, so such a point always exists.
    std::size_t kept = 0;
    for (std::size_t i = 0; i < m; ++i) kept += keep[i];
    if (kept < 3) {
        double bestSq = -1.0;
        std::size_t best = 0;
        for (std::size_t i = 0; i < m; ++i) {
            if (keep[i]) continue;
            const double d = sqSegmentDistance(xs[i], ys[i], xs[0], ys[0], xs[anchor], ys[anchor]);
            if (d > bestSq) {
                bestSq = d;
                best = i;
            }
        }
        keep[best] = 1;
        if (bestSq > maxRemovedSq) maxRemovedSq = bestSq;
    }

    return std::sqrt(maxRemovedSq);
}

} // namespace detail

// Result of simplifyPolygon(): the simplified rings plus the queryable statistics.
template <typename Polygon>
struct SimplifiedPolygon {
    using Ring = typename std::decay<decltype(std::declval<const Polygon&>()[0])>::type;
    using Point = typename std::decay<decltype(std::declval<const Ring&>()[0])>::type;

    std::vector<std::vector<Point>> rings;
    SimplifyStats stats;
};

// Validates the polygon and simplifies every ring with Douglas-Peucker so that no removed point
// is farther than `tolerance` (in coordinate units) from the simplified outline. Rings keep their
// order and stay above 3 points each. Throws SimplifyError on degenerate input (see
// validatePolygon) and on a negative/NaN tolerance.
//
// Simplification is not topology-preserving by itself, so the simplified polygon is re-validated;
// if it came out self-intersecting or with broken hole nesting, the original polygon is returned
// instead and stats.usedFallback is set. This keeps the triangulated-area deviation within the
// tolerance-derived bound below for every accepted input.
//
// Area impact: every removed point lies within `tolerance` of the simplified outline, so the
// absolute area change of a ring is bounded by 2*tolerance*L + pi*tolerance^2*E, where L is the
// simplified ring's perimeter and E its edge count — far below that in practice.
template <typename Polygon>
SimplifiedPolygon<Polygon> simplifyPolygon(const Polygon& poly, double tolerance) {
    if (!(tolerance >= 0.0)) { // also catches NaN
        throw SimplifyError("mapbox: tolerance must be a non-negative number");
    }
    detail::validatePolygon(poly);

    SimplifiedPolygon<Polygon> result;
    result.rings.reserve(poly.size());
    std::vector<char> keep;
    for (std::size_t r = 0; r < poly.size(); ++r) {
        const auto& ring = poly[r];
        result.stats.inputPoints += ring.size();
        const double deviation = detail::simplifyRing(ring, tolerance * tolerance, keep);
        if (deviation > result.stats.maxDeviation) result.stats.maxDeviation = deviation;

        std::vector<typename SimplifiedPolygon<Polygon>::Point> out;
        out.reserve(ring.size());
        for (std::size_t i = 0; i < ring.size(); ++i) {
            if (keep[i]) out.push_back(ring[i]);
        }
        result.stats.outputPoints += out.size();
        result.rings.push_back(std::move(out));
    }

    if (!detail::simplifiedPolygonValid(result.rings)) {
        // fall back to the unsimplified polygon: correctness beats compression
        result.rings.clear();
        for (std::size_t r = 0; r < poly.size(); ++r) {
            std::vector<typename SimplifiedPolygon<Polygon>::Point> ring;
            ring.reserve(poly[r].size());
            for (std::size_t i = 0; i < poly[r].size(); ++i) ring.push_back(poly[r][i]);
            result.rings.push_back(std::move(ring));
        }
        result.stats.outputPoints = result.stats.inputPoints;
        result.stats.maxDeviation = 0.0;
        result.stats.usedFallback = true;
    }
    return result;
}

// Simplify-then-triangulate pipeline. With tolerance == 0 the simplification pass is skipped
// entirely and the call forwards to plain earcut(), producing bit-identical indices. With
// tolerance > 0 the polygon is validated and simplified first; `stats` (optional) receives the
// simplification statistics. Throws SimplifyError on degenerate input at any tolerance.
template <typename N = uint32_t, typename Polygon>
std::vector<N> earcut(const Polygon& poly, double tolerance, SimplifyStats* stats = nullptr) {
    if (tolerance == 0.0) {
        detail::validatePolygon(poly);
        if (stats) {
            for (std::size_t r = 0; r < poly.size(); ++r) stats->inputPoints += poly[r].size();
            stats->outputPoints = stats->inputPoints;
            stats->maxDeviation = 0.0;
        }
        return mapbox::earcut<N>(poly);
    }
    const auto simplified = simplifyPolygon(poly, tolerance);
    if (stats) *stats = simplified.stats;
    return mapbox::earcut<N>(simplified.rings);
}

} // namespace mapbox
