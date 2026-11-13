#pragma once

#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <mapbox/earcut.hpp>

// Structural validation and fault-tolerant batch triangulation for earcut input.
//
// earcut itself is left untouched: validatePolygon() is an opt-in pre-flight check that
// classifies dirty rings into diagnosable categories, and earcutBatch() uses it to skip
// bad polygons instead of letting one dirty ring fail a whole batch silently.

namespace mapbox {

enum class PolygonIssue {
    // non-finite coordinate, or magnitude beyond 2^53 (exact-integer limit)
    CoordinateOverflow,
    // consecutive (or closing) identical vertices
    DuplicatePoint,
    // non-zero edge shorter than relEpsilon * ring bbox diagonal
    DegenerateEdge,
    // non-adjacent edges of the same ring cross or touch
    SelfIntersection,
};

inline const char* issueName(PolygonIssue issue) {
    switch (issue) {
        case PolygonIssue::CoordinateOverflow: return "CoordinateOverflow";
        case PolygonIssue::DuplicatePoint: return "DuplicatePoint";
        case PolygonIssue::DegenerateEdge: return "DegenerateEdge";
        case PolygonIssue::SelfIntersection: return "SelfIntersection";
    }
    return "Unknown";
}

struct PolygonDiagnostic {
    PolygonIssue issue;
    std::size_t ring = 0;   // ring index within the polygon
    std::size_t vertex = 0; // offending vertex, or start of the offending edge
    std::size_t other = 0;  // secondary index: previous vertex / edge end / crossing edge start
    std::string message;    // human-readable, ready to log
};

struct PolygonReport {
    std::vector<PolygonDiagnostic> diagnostics;

    bool ok() const { return diagnostics.empty(); }
    bool has(PolygonIssue issue) const {
        for (const auto& d : diagnostics)
            if (d.issue == issue) return true;
        return false;
    }
};

namespace detail {

// Coordinates beyond 2^53 can no longer represent adjacent integers, so quantization silently
// merges distinct vertices; non-finite coordinates poison every downstream predicate.
const double kMaxExactCoordinate = 9007199254740992.0; // 2^53

inline double orient2d(double ax, double ay, double bx, double by, double cx, double cy) {
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

inline bool onSegment(double ax, double ay, double bx, double by, double px, double py) {
    return px >= std::fmin(ax, bx) && px <= std::fmax(ax, bx) && py >= std::fmin(ay, by) &&
           py <= std::fmax(ay, by);
}

// True when segments ab and cd share at least one point (proper crossing or endpoint touch).
inline bool segmentsIntersect(double ax, double ay, double bx, double by, double cx, double cy, double dx,
                              double dy) {
    const double o1 = orient2d(ax, ay, bx, by, cx, cy);
    const double o2 = orient2d(ax, ay, bx, by, dx, dy);
    const double o3 = orient2d(cx, cy, dx, dy, ax, ay);
    const double o4 = orient2d(cx, cy, dx, dy, bx, by);
    if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)))
        return true;
    if (o1 == 0 && onSegment(ax, ay, bx, by, cx, cy)) return true;
    if (o2 == 0 && onSegment(ax, ay, bx, by, dx, dy)) return true;
    if (o3 == 0 && onSegment(cx, cy, dx, dy, ax, ay)) return true;
    if (o4 == 0 && onSegment(cx, cy, dx, dy, bx, by)) return true;
    return false;
}

inline std::string coordString(double v) {
    std::ostringstream ss;
    ss << v;
    return ss.str();
}

} // namespace detail

// Validates each ring of `polygon` (a container of rings of points, same shape as earcut input).
// relEpsilon sets the DegenerateEdge threshold relative to the ring's bbox diagonal.
template <typename Polygon>
PolygonReport validatePolygon(const Polygon& polygon, double relEpsilon = 1e-9) {
    using Ring = typename Polygon::value_type;
    using Point = typename Ring::value_type;
    using mapbox::util::nth;

    PolygonReport report;
    const std::size_t ringCount = polygon.size();

    for (std::size_t r = 0; r < ringCount; ++r) {
        const Ring& ring = polygon[r];
        const std::size_t n = ring.size();
        if (n == 0) continue;

        // Pass 1: coordinate range / finiteness, accumulating the ring bbox.
        bool ringFinite = true;
        double minX = 0, minY = 0, maxX = 0, maxY = 0;
        bool bboxInit = false;
        for (std::size_t i = 0; i < n; ++i) {
            const double x = static_cast<double>(nth<0, Point>::get(ring[i]));
            const double y = static_cast<double>(nth<1, Point>::get(ring[i]));
            if (!std::isfinite(x) || !std::isfinite(y)) {
                ringFinite = false;
                report.diagnostics.push_back(
                    {PolygonIssue::CoordinateOverflow, r, i, i,
                     "ring " + std::to_string(r) + " vertex " + std::to_string(i) +
                         ": non-finite coordinate (" + detail::coordString(x) + ", " +
                         detail::coordString(y) + ")"});
                continue;
            }
            if (std::fabs(x) > detail::kMaxExactCoordinate || std::fabs(y) > detail::kMaxExactCoordinate) {
                report.diagnostics.push_back(
                    {PolygonIssue::CoordinateOverflow, r, i, i,
                     "ring " + std::to_string(r) + " vertex " + std::to_string(i) +
                         ": coordinate (" + detail::coordString(x) + ", " + detail::coordString(y) +
                         ") exceeds 2^53 exact-integer limit"});
                continue;
            }
            if (!bboxInit) {
                minX = maxX = x;
                minY = maxY = y;
                bboxInit = true;
            } else {
                minX = std::fmin(minX, x);
                minY = std::fmin(minY, y);
                maxX = std::fmax(maxX, x);
                maxY = std::fmax(maxY, y);
            }
        }

        // Pass 2: consecutive duplicates, including the closing pair (last vs first).
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t j = (i + 1) % n;
            const double xi = static_cast<double>(nth<0, Point>::get(ring[i]));
            const double yi = static_cast<double>(nth<1, Point>::get(ring[i]));
            const double xj = static_cast<double>(nth<0, Point>::get(ring[j]));
            const double yj = static_cast<double>(nth<1, Point>::get(ring[j]));
            if (xi == xj && yi == yj) {
                report.diagnostics.push_back(
                    {PolygonIssue::DuplicatePoint, r, j, i,
                     "ring " + std::to_string(r) + " vertex " + std::to_string(j) +
                         " duplicates vertex " + std::to_string(i) + " at (" + detail::coordString(xi) +
                         ", " + detail::coordString(yi) + ")"});
            }
        }

        // Geometric checks are meaningless on non-finite rings; they are already diagnosed.
        if (!ringFinite || n < 2) continue;

        // Pass 3: degenerate (near-zero length) edges.
        const double diag = std::hypot(maxX - minX, maxY - minY);
        const double eps = relEpsilon * diag;
        if (eps > 0) {
            const double eps2 = eps * eps;
            for (std::size_t i = 0; i < n; ++i) {
                const std::size_t j = (i + 1) % n;
                const double dx = static_cast<double>(nth<0, Point>::get(ring[j])) -
                                  static_cast<double>(nth<0, Point>::get(ring[i]));
                const double dy = static_cast<double>(nth<1, Point>::get(ring[j])) -
                                  static_cast<double>(nth<1, Point>::get(ring[i]));
                const double len2 = dx * dx + dy * dy;
                if (len2 > 0 && len2 < eps2) {
                    report.diagnostics.push_back(
                        {PolygonIssue::DegenerateEdge, r, i, j,
                         "ring " + std::to_string(r) + " edge " + std::to_string(i) + "->" +
                             std::to_string(j) + " length " + detail::coordString(std::sqrt(len2)) +
                             " below threshold " + detail::coordString(eps)});
                }
            }
        }

        // Pass 4: self-intersection between non-adjacent edges (first hit per ring is enough
        // to locate the problem; keeps diagnosis bounded on heavily tangled input).
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t i1 = (i + 1) % n;
            const double ax = static_cast<double>(nth<0, Point>::get(ring[i]));
            const double ay = static_cast<double>(nth<1, Point>::get(ring[i]));
            const double bx = static_cast<double>(nth<0, Point>::get(ring[i1]));
            const double by = static_cast<double>(nth<1, Point>::get(ring[i1]));
            bool found = false;
            for (std::size_t j = i + 1; j < n; ++j) {
                const std::size_t j1 = (j + 1) % n;
                if (j == i || j1 == i || i1 == j) continue; // adjacent edges share a vertex
                const double cx = static_cast<double>(nth<0, Point>::get(ring[j]));
                const double cy = static_cast<double>(nth<1, Point>::get(ring[j]));
                const double dx = static_cast<double>(nth<0, Point>::get(ring[j1]));
                const double dy = static_cast<double>(nth<1, Point>::get(ring[j1]));
                if (detail::segmentsIntersect(ax, ay, bx, by, cx, cy, dx, dy)) {
                    report.diagnostics.push_back(
                        {PolygonIssue::SelfIntersection, r, i, j,
                         "ring " + std::to_string(r) + " self-intersection: edge " + std::to_string(i) +
                             "->" + std::to_string(i1) + " crosses edge " + std::to_string(j) + "->" +
                             std::to_string(j1)});
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }
    return report;
}

// One batch entry: either a clean triangulation, or diagnostics explaining why it was skipped.
template <typename N = uint32_t>
struct BatchItem {
    PolygonReport report;
    std::vector<N> indices; // empty unless report.ok()

    bool ok() const { return report.ok(); }
};

// Fault-tolerant batch triangulation: every polygon is validated first; dirty ones come back
// with diagnostics and empty indices while the rest of the batch still triangulates. Valid
// polygons go through the unmodified mapbox::earcut, so their output is vertex-for-vertex
// identical to calling earcut directly.
template <typename N = uint32_t, typename Batch>
std::vector<BatchItem<N>> earcutBatch(const Batch& polygons, double relEpsilon = 1e-9) {
    std::vector<BatchItem<N>> results;
    results.reserve(polygons.size());
    for (const auto& polygon : polygons) {
        BatchItem<N> item;
        item.report = validatePolygon(polygon, relEpsilon);
        if (item.report.ok()) item.indices = mapbox::earcut<N>(polygon);
        results.push_back(std::move(item));
    }
    return results;
}

} // namespace mapbox
