//------------------------------------------------------------------------------
// Vertex Clique Cover (VCC) Convex Decomposition Wrapper for acd2d
//------------------------------------------------------------------------------

// CGAL & ExtensionCC headers FIRST to avoid macro clashes
#include "cgal.h"
#include "dualgraph.h"
#include "visibility.h"
#include "intersection_predicates.h"
#include "chgraph.h"
#include "partition_constructor.h"
#include "clique.h"
#include "geometry_utils.h"

#include "acd2d_vcc.h"
#include <iostream>
#include <chrono>
#include <vector>

namespace acd2d {

VccDecompositionResult VccWrapper::ComputeVccDecomposition(const cd_polygon& poly, bool use_extension) {
    VccDecompositionResult result;
    result.is_extension = use_extension;
    auto start_time = std::chrono::high_resolution_clock::now();

    if (poly.empty()) return result;

    ::Polygon outer_polygon;
    std::vector<::Polygon> hole_polygons;

    for (cd_polygon::const_iterator ip = poly.begin(); ip != poly.end(); ++ip) {
        ::Polygon current_cgal_poly;
        cd_vertex* ptr = ip->getHead();
        if (ptr != NULL) {
            do {
                const Point2d& pt = ptr->getPos();
                current_cgal_poly.push_back(::Point(pt[0], pt[1]));
                ptr = ptr->getNext();
            } while (ptr != ip->getHead());
        }

        if (current_cgal_poly.size() < 3) continue;

        if (ip->getType() == cd_poly::POUT || outer_polygon.size() == 0) {
            outer_polygon = current_cgal_poly;
            if (!outer_polygon.is_counterclockwise_oriented()) {
                outer_polygon.reverse_orientation();
            }
        } else {
            if (!current_cgal_poly.is_clockwise_oriented()) {
                current_cgal_poly.reverse_orientation();
            }
            hole_polygons.push_back(current_cgal_poly);
        }
    }

    if (outer_polygon.size() < 3) {
        std::cerr << "! ERROR [VCC]: Outer polygon is invalid or has < 3 vertices!" << std::endl;
        return result;
    }

    ::Polygon_with_holes input_polygon(outer_polygon, hole_polygons.begin(), hole_polygons.end());

    try {
        PartitionConstructor partition(input_polygon);
        if (use_extension) {
            partition.add_extension_segments();
        }

        auto triangulation = partition.get_constrained_delaunay_triangulation();

        std::string cache_id = use_extension ? "extension_triangulation_gui" : "delaunay_gui";
        CHGraph chgraph(input_polygon, triangulation, cache_id);
        chgraph.add_all_edges();

        CliqueCover cliquecover(chgraph);
        std::vector<::Polygon> vcc_polygons;

        try {
            vcc_polygons = cliquecover.get_cliques_reduvcc();
            if (vcc_polygons.empty()) {
                std::cerr << "! WARNING [VCC Solver]: ReduVCC returned no cliques, falling back to naive greedy clique cover." << std::endl;
                vcc_polygons = cliquecover.get_cliques_smalladj_naive();
            }
        } catch (const std::exception& e) {
            std::cerr << "! WARNING [VCC Solver]: Fallback to naive greedy clique cover: " << e.what() << std::endl;
            vcc_polygons = cliquecover.get_cliques_smalladj_naive();
        }

        chgraph.delete_datastructures();

        // Convert output CGAL polygons to Point2d regions for acd2d rendering
        for (auto& cgal_poly : vcc_polygons) {
            std::vector<Point2d> region;
            for (auto& pt : cgal_poly) {
                double x = CGAL::to_double(pt.x());
                double y = CGAL::to_double(pt.y());
                region.push_back(Point2d(x, y));
            }
            if (region.size() >= 3) {
                result.regions.push_back(region);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "! ERROR [VCC Exception]: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "! ERROR [VCC Exception]: Unknown exception during VCC decomposition!" << std::endl;
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    result.computation_time_sec = std::chrono::duration<double>(end_time - start_time).count();

    std::cout << "- VCC Convex Decomposition (ReduVCC - " << (use_extension ? "Extension Triangulation" : "Constrained Delaunay")
              << ") completed: " << result.regions.size() << " convex regions in "
              << result.computation_time_sec << " seconds." << std::endl;

    return result;
}

} // namespace acd2d
