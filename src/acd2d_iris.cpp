#include "acd2d_iris.h"
#include <iostream>

namespace acd2d {

IrisWrapper::IrisWrapper() {}

drake::geometry::optimization::HPolyhedron IrisWrapper::InflateRegion(
    const std::vector<drake::geometry::optimization::HPolyhedron>& obstacles,
    const Eigen::Vector2d& sample_point,
    const drake::geometry::optimization::HPolyhedron& domain
) {
    drake::geometry::optimization::IrisOptions options;
    options.require_sample_point_is_contained = true;

    // Convert vector of HPolyhedron into ConvexSets
    drake::geometry::optimization::ConvexSets obstacle_sets;
    for (const auto& obs : obstacles) {
        obstacle_sets.push_back(drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(obs.Clone()));
    }

    // Run Drake's IRIS algorithm
    drake::geometry::optimization::HPolyhedron region = drake::geometry::optimization::Iris(
        obstacle_sets,
        sample_point,
        domain,
        options
    );

    return region;
}

std::vector<Eigen::Vector2d> IrisWrapper::GetHPolyhedronVertices(
    const drake::geometry::optimization::HPolyhedron& hpoly
) {
    std::vector<Eigen::Vector2d> vertices;
    const Eigen::MatrixXd& A = hpoly.A();
    const Eigen::VectorXd& b = hpoly.b();
    int m = A.rows();

    for (int i = 0; i < m; ++i) {
        for (int j = i + 1; j < m; ++j) {
            double det = A(i, 0) * A(j, 1) - A(i, 1) * A(j, 0);
            if (std::abs(det) < 1e-8) continue; // Parallel lines

            double x = (b(i) * A(j, 1) - b(j) * A(i, 1)) / det;
            double y = (A(i, 0) * b(j) - A(j, 0) * b(i)) / det;
            Eigen::Vector2d pt(x, y);

            // Check if pt satisfies all inequalities A * pt <= b
            bool inside = true;
            for (int k = 0; k < m; ++k) {
                if (A(k, 0) * x + A(k, 1) * y > b(k) + 1e-4) {
                    inside = false;
                    break;
                }
            }

            if (inside) {
                bool dup = false;
                for (const auto& v : vertices) {
                    if ((v - pt).squaredNorm() < 1e-8) {
                        dup = true;
                        break;
                    }
                }
                if (!dup) {
                    vertices.push_back(pt);
                }
            }
        }
    }

    if (vertices.size() < 3) return vertices;

    // Compute centroid of vertices to sort radially
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (const auto& v : vertices) center += v;
    center /= static_cast<double>(vertices.size());

    std::sort(vertices.begin(), vertices.end(), [&center](const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
        return std::atan2(p1(1) - center(1), p1(0) - center(0)) < std::atan2(p2(1) - center(1), p2(0) - center(0));
    });

    return vertices;
}

static drake::geometry::optimization::HPolyhedron MakeSegmentObstacle(
    const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    double eps = 1e-3
) {
    Eigen::Vector2d edge = p2 - p1;
    double L = edge.norm();
    Eigen::Vector2d u = edge / L;
    Eigen::Vector2d v(-u(1), u(0));

    double v_dot_p1 = v.dot(p1);
    double u_dot_p1 = u.dot(p1);

    Eigen::Matrix<double, 4, 2> A_obs;
    A_obs <<  v(0),  v(1),
             -v(0), -v(1),
              u(0),  u(1),
             -u(0), -u(1);

    Eigen::Vector4d b_obs(
         v_dot_p1 + eps,
        -v_dot_p1 + eps,
         u_dot_p1 + L,
        -u_dot_p1
    );

    return drake::geometry::optimization::HPolyhedron(A_obs, b_obs);
}

std::vector<Eigen::Vector2d> IrisWrapper::ComputeIrisPolygon(
    const std::vector<Eigen::Vector2d>& poly_vertices,
    const Eigen::Vector2d& seed,
    const double bbox[4]
) {
    if (poly_vertices.size() < 3) return {};

    // Create domain HPolyhedron box from bbox
    Eigen::Matrix<double, 4, 2> A_dom;
    A_dom <<  1,  0,
             -1,  0,
              0,  1,
              0, -1;
    Eigen::Vector4d b_dom;
    double margin = (bbox[1] - bbox[0] + bbox[3] - bbox[2]) * 0.1;
    if (margin < 1.0) margin = 1.0;
    b_dom << bbox[1] + margin, -bbox[0] + margin, bbox[3] + margin, -bbox[2] + margin;
    drake::geometry::optimization::HPolyhedron domain(A_dom, b_dom);

    // Build obstacle sets from polygon edges using exact segment-aligned boxes
    std::vector<drake::geometry::optimization::HPolyhedron> obstacles;
    size_t n = poly_vertices.size();
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector2d& p1 = poly_vertices[i];
        const Eigen::Vector2d& p2 = poly_vertices[(i + 1) % n];

        if ((p2 - p1).squaredNorm() < 1e-8) continue;
        obstacles.push_back(MakeSegmentObstacle(p1, p2));
    }

    // Call Iris
    drake::geometry::optimization::HPolyhedron inflated = InflateRegion(obstacles, seed, domain);

    // Extract sorted 2D vertices of the inflated polyhedron
    return GetHPolyhedronVertices(inflated);
}

IrisWrapper::IrisDecompositionResult IrisWrapper::ComputeIrisDecomposition(
    const std::vector<std::vector<Eigen::Vector2d>>& all_rings,
    const std::vector<Eigen::Vector2d>& seeds,
    const double bbox[4]
) {
    IrisDecompositionResult result;
    if (all_rings.empty() || seeds.empty()) return result;

    // Create domain HPolyhedron box from bbox
    Eigen::Matrix<double, 4, 2> A_dom;
    A_dom <<  1,  0,
             -1,  0,
              0,  1,
              0, -1;
    Eigen::Vector4d b_dom;
    double margin = (bbox[1] - bbox[0] + bbox[3] - bbox[2]) * 0.1;
    if (margin < 1.0) margin = 1.0;
    b_dom << bbox[1] + margin, -bbox[0] + margin, bbox[3] + margin, -bbox[2] + margin;
    drake::geometry::optimization::HPolyhedron domain(A_dom, b_dom);

    // Build obstacle sets from ALL ring edges (outer boundary + holes)
    std::vector<drake::geometry::optimization::HPolyhedron> obstacles;
    for (const auto& ring : all_rings) {
        size_t n = ring.size();
        for (size_t i = 0; i < n; ++i) {
            const Eigen::Vector2d& p1 = ring[i];
            const Eigen::Vector2d& p2 = ring[(i + 1) % n];

            if ((p2 - p1).squaredNorm() < 1e-8) continue;
            obstacles.push_back(MakeSegmentObstacle(p1, p2));
        }
    }

    std::vector<drake::geometry::optimization::HPolyhedron> inflated_regions;

    for (const auto& seed : seeds) {
        // Skip seed if already covered by an existing inflated region
        bool covered = false;
        for (const auto& region : inflated_regions) {
            if (region.PointInSet(seed, 1e-2)) {
                covered = true;
                break;
            }
        }
        if (covered) continue;

        try {
            drake::geometry::optimization::HPolyhedron region = InflateRegion(obstacles, seed, domain);
            std::vector<Eigen::Vector2d> verts = GetHPolyhedronVertices(region);
            if (verts.size() >= 3) {
                inflated_regions.push_back(region);
                result.regions.push_back(verts);
                result.seeds.push_back(seed);
            }
        } catch (...) {
            // Ignore optimization failures
        }
    }

    return result;
}

} // namespace acd2d
