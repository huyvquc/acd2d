#ifndef _ACD2D_IRIS_H_
#define _ACD2D_IRIS_H_

#include <vector>
#include <Eigen/Dense>
#include <drake/geometry/optimization/iris.h>
#include <drake/geometry/optimization/hpolyhedron.h>

namespace acd2d {

/**
 * @brief Simple wrapper around Drake's IRIS algorithm for 2D convex region inflation.
 */
class IrisWrapper {
public:
    IrisWrapper();

    /**
     * @brief Computes a convex polytope (HPolyhedron) in 2D given obstacle hyperplanes and a sample point inside the obstacle-free space.
     * 
     * @param obstacles Vector of Drake HPolyhedron representing obstacle sets.
     * @param sample_point Seed point in 2D space inside the free region.
     * @param domain Outer bounding box (HPolyhedron) for the region.
     * @return drake::geometry::optimization::HPolyhedron The inflated convex region.
     */
    static drake::geometry::optimization::HPolyhedron InflateRegion(
        const std::vector<drake::geometry::optimization::HPolyhedron>& obstacles,
        const Eigen::Vector2d& sample_point,
        const drake::geometry::optimization::HPolyhedron& domain
    );

    /**
     * @brief Extracts sorted 2D vertices of an HPolyhedron Ax <= b.
     */
    static std::vector<Eigen::Vector2d> GetHPolyhedronVertices(
        const drake::geometry::optimization::HPolyhedron& hpoly
    );

    /**
     * @brief Computes 2D IRIS inflated region for a single seed.
     */
    static std::vector<Eigen::Vector2d> ComputeIrisPolygon(
        const std::vector<Eigen::Vector2d>& poly_vertices,
        const Eigen::Vector2d& seed,
        const double bbox[4]
    );

    /**
     * @brief Computes 2D IRIS decomposition for an entire polygon (with optional holes) using multiple interior seed points.
     */
    static std::vector<std::vector<Eigen::Vector2d>> ComputeIrisDecomposition(
        const std::vector<std::vector<Eigen::Vector2d>>& all_rings,
        const std::vector<Eigen::Vector2d>& seeds,
        const double bbox[4]
    );
};

} // namespace acd2d

#endif // _ACD2D_IRIS_H_
