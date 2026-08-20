//------------------------------------------------------------------------------
// Convex Decomposition Directed Weighted Graph for ACD2D
// Supports ACD, IRIS, and VCC convex decompositions
//------------------------------------------------------------------------------

#ifndef _ACD2D_GRAPH_H_
#define _ACD2D_GRAPH_H_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <queue>
#include "acd2d_Point.h"
#include "acd2d_Vector.h"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <GL/gliFont.h>

namespace acd2d {

/**
 * @brief Directed edge in the convex decomposition graph.
 */
struct GraphEdge {
    int target;                 // Index of adjacent convex node
    double weight;              // Edge weight (currently 0.0)
    double shared_length;       // Shared boundary contact length or overlap measure
    Point2d interface_pt;       // Contact interface midpoint
};

/**
 * @brief Node in the convex decomposition graph representing one convex piece.
 */
struct GraphNode {
    int id;                     // Unique node index (0, 1, 2, ...)
    std::string label;          // Label string, e.g., "C0", "C1", ...
    Point2d centroid;           // Centroid / center of mass (Cx, Cy)
    double area;                // Geometric area of the convex polygon
    std::vector<Point2d> vertices; // Boundary vertices of the convex polygon
    std::vector<GraphEdge> adj; // Outgoing directed edges
};

/**
 * @brief Directed Weighted Graph constructed from 2D convex decompositions.
 */
class ConvexGraph {
public:
    std::string decomposition_type; // "ACD", "IRIS", "VCC-Delaunay", "VCC-Extension"
    std::vector<GraphNode> nodes;

    ConvexGraph() : decomposition_type("None") {}

    bool empty() const {
        return nodes.empty();
    }

    size_t size() const {
        return nodes.size();
    }

    void clear() {
        nodes.clear();
        decomposition_type = "None";
    }

    /**
     * @brief Computes centroid and unsigned area of a 2D polygon.
     */
    static void computePolygonCentroidAndArea(
        const std::vector<Point2d>& pts,
        Point2d& centroid,
        double& area
    ) {
        int n = static_cast<int>(pts.size());
        if (n < 3) {
            centroid = (n > 0) ? pts[0] : Point2d(0, 0);
            area = 0.0;
            return;
        }

        double signed_area = 0.0;
        double cx = 0.0;
        double cy = 0.0;

        for (int i = 0; i < n; ++i) {
            const Point2d& p1 = pts[i];
            const Point2d& p2 = pts[(i + 1) % n];
            double cross = (p1[0] * p2[1] - p2[0] * p1[1]);
            signed_area += cross;
            cx += (p1[0] + p2[0]) * cross;
            cy += (p1[1] + p2[1]) * cross;
        }

        signed_area *= 0.5;
        area = std::abs(signed_area);

        if (area > 1e-7) {
            cx /= (6.0 * signed_area);
            cy /= (6.0 * signed_area);
            centroid = Point2d(cx, cy);
        } else {
            // Fallback to arithmetic mean of vertices
            double sum_x = 0.0, sum_y = 0.0;
            for (const auto& p : pts) {
                sum_x += p[0];
                sum_y += p[1];
            }
            centroid = Point2d(sum_x / n, sum_y / n);
        }
    }

    /**
     * @brief Checks if two line segments (a1-b1) and (a2-b2) have collinear overlap.
     */
    static bool checkEdgeCollinearOverlap(
        const Point2d& a1, const Point2d& b1,
        const Point2d& a2, const Point2d& b2,
        double& shared_len,
        Point2d& mid_pt,
        double eps = 1e-3
    ) {
        Vector2d e1(b1[0] - a1[0], b1[1] - a1[1]);
        double L1 = e1.norm();
        if (L1 < 1e-6) return false;

        Vector2d u1(e1[0] / L1, e1[1] / L1);
        Vector2d n1(-u1[1], u1[0]); // Normal to e1

        // Check if a2 and b2 lie on the line passing through e1
        Vector2d v_a2(a2[0] - a1[0], a2[1] - a1[1]);
        Vector2d v_b2(b2[0] - a1[0], b2[1] - a1[1]);

        double d_a2 = v_a2 * n1;
        double d_b2 = v_b2 * n1;

        if (std::abs(d_a2) > eps || std::abs(d_b2) > eps) {
            return false;
        }

        // Project onto line 1
        double t_a2 = v_a2 * u1;
        double t_b2 = v_b2 * u1;

        double t2_min = std::min(t_a2, t_b2);
        double t2_max = std::max(t_a2, t_b2);

        double t_overlap_min = std::max(0.0, t2_min);
        double t_overlap_max = std::min(L1, t2_max);

        if (t_overlap_max - t_overlap_min > 1e-4) {
            shared_len = t_overlap_max - t_overlap_min;
            double t_mid = 0.5 * (t_overlap_min + t_overlap_max);
            mid_pt = Point2d(a1[0] + u1[0] * t_mid, a1[1] + u1[1] * t_mid);
            return true;
        }

        return false;
    }

    /**
     * @brief Computes polygon intersection area using Sutherland-Hodgman polygon clipping.
     */
    static void ensureCCW(std::vector<Point2d>& pts) {
        int n = static_cast<int>(pts.size());
        if (n < 3) return;
        double signed_area = 0.0;
        for (int i = 0; i < n; ++i) {
            signed_area += (pts[i][0] * pts[(i + 1) % n][1] - pts[(i + 1) % n][0] * pts[i][1]);
        }
        if (signed_area < 0) {
            std::reverse(pts.begin(), pts.end());
        }
    }

    static std::vector<Point2d> clipConvexPolygon(
        const std::vector<Point2d>& subjectPoly,
        const std::vector<Point2d>& clipPoly
    ) {
        std::vector<Point2d> outputList = subjectPoly;
        int clipSize = static_cast<int>(clipPoly.size());
        if (clipSize < 3) return {};

        for (int i = 0; i < clipSize; ++i) {
            if (outputList.empty()) break;
            std::vector<Point2d> inputList = outputList;
            outputList.clear();

            Point2d c1 = clipPoly[i];
            Point2d c2 = clipPoly[(i + 1) % clipSize];

            auto isInside = [&](const Point2d& p) -> bool {
                return (c2[0] - c1[0]) * (p[1] - c1[1]) - (c2[1] - c1[1]) * (p[0] - c1[0]) >= -1e-7;
            };

            auto intersection = [&](const Point2d& p1, const Point2d& p2) -> Point2d {
                double A1 = p2[1] - p1[1];
                double B1 = p1[0] - p2[0];
                double C1 = A1 * p1[0] + B1 * p1[1];

                double A2 = c2[1] - c1[1];
                double B2 = c1[0] - c2[0];
                double C2 = A2 * c1[0] + B2 * c1[1];

                double det = A1 * B2 - A2 * B1;
                if (std::abs(det) < 1e-10) return p1;
                return Point2d((B2 * C1 - B1 * C2) / det, (A1 * C2 - A2 * C1) / det);
            };

            int inSize = static_cast<int>(inputList.size());
            for (int j = 0; j < inSize; ++j) {
                Point2d currentPt = inputList[j];
                Point2d prevPt = inputList[(j + inSize - 1) % inSize];

                bool curIn = isInside(currentPt);
                bool prevIn = isInside(prevPt);

                if (curIn) {
                    if (!prevIn) {
                        outputList.push_back(intersection(prevPt, currentPt));
                    }
                    outputList.push_back(currentPt);
                } else if (prevIn) {
                    outputList.push_back(intersection(prevPt, currentPt));
                }
            }
        }
        return outputList;
    }

    /**
     * @brief Determines if two convex pieces are adjacent/neighbors.
     * Two convex pieces are adjacent iff they share a boundary line segment (L > 0)
     * or have a positive 2D area overlap (for overlapping covers like IRIS).
     * Single isolated vertex contact (sharing a single corner point) is NOT adjacency.
     */
    static bool arePolygonsAdjacent(
        const GraphNode& n1,
        const GraphNode& n2,
        double& shared_len,
        Point2d& interface_pt
    ) {
        const auto& p1 = n1.vertices;
        const auto& p2 = n2.vertices;
        int m1 = static_cast<int>(p1.size());
        int m2 = static_cast<int>(p2.size());

        if (m1 < 3 || m2 < 3) return false;

        // 1. Check for collinear shared boundary segments (Shared edge contact L > 0)
        double total_shared = 0.0;
        double sum_mid_x = 0.0, sum_mid_y = 0.0;
        int match_count = 0;

        for (int i = 0; i < m1; ++i) {
            const Point2d& a1 = p1[i];
            const Point2d& b1 = p1[(i + 1) % m1];

            for (int j = 0; j < m2; ++j) {
                const Point2d& a2 = p2[j];
                const Point2d& b2 = p2[(j + 1) % m2];

                double seg_len = 0.0;
                Point2d seg_mid;
                if (checkEdgeCollinearOverlap(a1, b1, a2, b2, seg_len, seg_mid)) {
                    total_shared += seg_len;
                    sum_mid_x += seg_mid[0] * seg_len;
                    sum_mid_y += seg_mid[1] * seg_len;
                    match_count++;
                }
            }
        }

        if (match_count > 0 && total_shared > 1e-4) {
            shared_len = total_shared;
            interface_pt = Point2d(sum_mid_x / total_shared, sum_mid_y / total_shared);
            return true;
        }

        // 2. Check for true 2D area overlap (for overlapping covers like IRIS)
        std::vector<Point2d> ccw1 = p1;
        std::vector<Point2d> ccw2 = p2;
        ensureCCW(ccw1);
        ensureCCW(ccw2);

        std::vector<Point2d> intersectionPoly = clipConvexPolygon(ccw1, ccw2);
        if (intersectionPoly.size() >= 3) {
            Point2d inter_centroid;
            double inter_area = 0.0;
            computePolygonCentroidAndArea(intersectionPoly, inter_centroid, inter_area);

            double min_area = std::min(n1.area, n2.area);
            if (inter_area > 1e-4 && (min_area <= 1e-7 || inter_area > 0.005 * min_area)) {
                shared_len = inter_area;
                interface_pt = inter_centroid;
                return true;
            }
        }

        // Single isolated vertex contact (0D point) is NOT adjacency
        return false;
    }

    /**
     * @brief Constructs the directed weighted graph from a collection of convex polygons.
     */
    void buildFromPolygons(
        const std::vector<std::vector<Point2d>>& pieces,
        const std::string& type_name
    ) {
        clear();
        decomposition_type = type_name;

        int num_pieces = static_cast<int>(pieces.size());
        if (num_pieces == 0) return;

        nodes.resize(num_pieces);

        // Step 1: Initialize all vertices / nodes
        for (int i = 0; i < num_pieces; ++i) {
            nodes[i].id = i;
            std::stringstream ss;
            ss << "C" << i;
            nodes[i].label = ss.str();
            nodes[i].vertices = pieces[i];
            computePolygonCentroidAndArea(pieces[i], nodes[i].centroid, nodes[i].area);
        }

        // Step 2: Determine adjacency and construct directed edges (weight = 0.0)
        for (int i = 0; i < num_pieces; ++i) {
            for (int j = i + 1; j < num_pieces; ++j) {
                double shared_len = 0.0;
                Point2d if_pt;
                if (arePolygonsAdjacent(nodes[i], nodes[j], shared_len, if_pt)) {
                    // Directed edge i -> j
                    GraphEdge e_ij;
                    e_ij.target = j;
                    e_ij.weight = 0.0; // Fixed to 0.0 as requested
                    e_ij.shared_length = shared_len;
                    e_ij.interface_pt = if_pt;
                    nodes[i].adj.push_back(e_ij);

                    // Directed edge j -> i
                    GraphEdge e_ji;
                    e_ji.target = i;
                    e_ji.weight = 0.0; // Fixed to 0.0 as requested
                    e_ji.shared_length = shared_len;
                    e_ji.interface_pt = if_pt;
                    nodes[j].adj.push_back(e_ji);
                }
            }
        }
    }

    /**
     * @brief Total number of directed edges in the graph.
     */
    int getTotalDirectedEdges() const {
        int count = 0;
        for (const auto& n : nodes) {
            count += static_cast<int>(n.adj.size());
        }
        return count;
    }

    /**
     * @brief Prints a human-readable summary of the directed weighted graph.
     */
    void printSummary(std::ostream& os = std::cout) const {
        os << "\n=========================================================\n";
        os << " Convex Decomposition Directed Weighted Graph [" << decomposition_type << "]\n";
        os << " Vertices: " << nodes.size() 
           << " | Directed Edges: " << getTotalDirectedEdges() << "\n";
        os << "---------------------------------------------------------\n";

        for (const auto& n : nodes) {
            os << " Vertex " << std::left << std::setw(4) << n.label 
               << " (id=" << n.id << "): Centroid=(" 
               << std::fixed << std::setprecision(2) << n.centroid[0] << ", " 
               << n.centroid[1] << "), Area=" << n.area 
               << ", Vertices=" << n.vertices.size() << "\n";

            if (n.adj.empty()) {
                os << "   -> (No adjacent neighbors)\n";
            } else {
                for (const auto& e : n.adj) {
                    os << "   -> " << std::left << std::setw(4) << nodes[e.target].label
                       << " [weight=" << std::fixed << std::setprecision(2) << e.weight
                       << ", shared_len=" << e.shared_length
                       << ", contact_pt=(" << e.interface_pt[0] << ", " << e.interface_pt[1] << ")]\n";
                }
            }
        }
        os << "=========================================================\n" << std::flush;
    }

    /**
     * @brief Exports the graph to a JSON file.
     */
    bool exportJSON(const std::string& filepath) const {
        std::ofstream fout(filepath.c_str());
        if (!fout.good()) {
            std::cerr << "! ERROR [Graph]: Cannot open JSON output file: " << filepath << std::endl;
            return false;
        }

        fout << "{\n";
        fout << "  \"decomposition_type\": \"" << decomposition_type << "\",\n";
        fout << "  \"num_vertices\": " << nodes.size() << ",\n";
        fout << "  \"num_directed_edges\": " << getTotalDirectedEdges() << ",\n";

        // Vertices
        fout << "  \"vertices\": [\n";
        for (size_t i = 0; i < nodes.size(); ++i) {
            const auto& n = nodes[i];
            fout << "    {\n";
            fout << "      \"id\": " << n.id << ",\n";
            fout << "      \"label\": \"" << n.label << "\",\n";
            fout << "      \"centroid\": [" << std::fixed << std::setprecision(4) 
                 << n.centroid[0] << ", " << n.centroid[1] << "],\n";
            fout << "      \"area\": " << n.area << ",\n";
            fout << "      \"polygon\": [";
            for (size_t v = 0; v < n.vertices.size(); ++v) {
                fout << "[" << n.vertices[v][0] << ", " << n.vertices[v][1] << "]";
                if (v + 1 < n.vertices.size()) fout << ", ";
            }
            fout << "]\n";
            fout << "    }" << (i + 1 < nodes.size() ? "," : "") << "\n";
        }
        fout << "  ],\n";

        // Directed Edges
        fout << "  \"edges\": [\n";
        bool first_edge = true;
        for (const auto& n : nodes) {
            for (const auto& e : n.adj) {
                if (!first_edge) fout << ",\n";
                first_edge = false;
                fout << "    {\n";
                fout << "      \"from\": " << n.id << ",\n";
                fout << "      \"to\": " << e.target << ",\n";
                fout << "      \"from_label\": \"" << n.label << "\",\n";
                fout << "      \"to_label\": \"" << nodes[e.target].label << "\",\n";
                fout << "      \"weight\": " << std::fixed << std::setprecision(4) << e.weight << ",\n";
                fout << "      \"shared_length\": " << e.shared_length << ",\n";
                fout << "      \"interface_pt\": [" << e.interface_pt[0] << ", " << e.interface_pt[1] << "]\n";
                fout << "    }";
            }
        }
        fout << "\n  ]\n";
        fout << "}\n";

        fout.close();
        std::cout << "- Graph exported to JSON: " << filepath << std::endl;
        return true;
    }

    /**
     * @brief Exports the graph to a Graphviz DOT file.
     */
    bool exportDOT(const std::string& filepath) const {
        std::ofstream fout(filepath.c_str());
        if (!fout.good()) {
            std::cerr << "! ERROR [Graph]: Cannot open DOT output file: " << filepath << std::endl;
            return false;
        }

        fout << "digraph ConvexGraph {\n";
        fout << "  graph [label=\"" << decomposition_type << " Convex Decomposition Graph\", fontsize=14];\n";
        fout << "  node [shape=circle, style=filled, fillcolor=\"#e0f0ff\", color=\"#0066cc\", fontname=\"Helvetica\"];\n";
        fout << "  edge [fontname=\"Helvetica\", fontsize=10, color=\"#444444\"];\n\n";

        for (const auto& n : nodes) {
            fout << "  " << n.id << " [label=\"" << n.label << "\\n(" 
                 << std::fixed << std::setprecision(1) << n.centroid[0] << "," << n.centroid[1] << ")\"];\n";
        }
        fout << "\n";

        for (const auto& n : nodes) {
            for (const auto& e : n.adj) {
                fout << "  " << n.id << " -> " << e.target 
                     << " [label=\"w=" << std::fixed << std::setprecision(1) << e.weight 
                     << "\", weight=" << e.weight << "];\n";
            }
        }

        fout << "}\n";
        fout.close();
        std::cout << "- Graph exported to DOT: " << filepath << std::endl;
        return true;
    }

    /**
     * @brief Pixel-accurate screen-space label rendering in Main Window at exact convex center.
     */
    void drawScreenLabelsGL() const {
        if (nodes.empty()) return;

        GLdouble modelview[16];
        GLdouble projection[16];
        GLint viewport[4];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        glGetIntegerv(GL_VIEWPORT, viewport);

        struct ProjectedNode {
            double winX, winY;
            std::string label;
        };
        std::vector<ProjectedNode> p_nodes;
        p_nodes.reserve(nodes.size());

        for (const auto& n : nodes) {
            GLdouble winX, winY, winZ;
            gluProject(n.centroid[0], n.centroid[1], 0.0, modelview, projection, viewport, &winX, &winY, &winZ);
            if (winX >= -100 && winX <= viewport[2] + 100 && winY >= -100 && winY <= viewport[3] + 100) {
                p_nodes.push_back({winX, winY, n.label});
            }
        }

        if (p_nodes.empty()) return;

        glPushAttrib(GL_ALL_ATTRIB_BITS);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_STENCIL_TEST);
        glDisable(GL_CULL_FACE);
        glDisable(GL_TEXTURE_2D);
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

        // Switch to Screen-space 2D Pixel Ortho projection
        glMatrixMode(GL_PROJECTION);
        glPushMatrix();
        glLoadIdentity();
        gluOrtho2D(0, viewport[2], 0, viewport[3]);

        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glLoadIdentity();

        double badge_r = 15.0; // 15px radius

        // Pass 1: Draw all circular dark badge discs and white border rings
        for (const auto& pn : p_nodes) {
            glColor4f(0.08f, 0.09f, 0.14f, 0.92f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2d(pn.winX, pn.winY);
            for (int a = 0; a <= 28; ++a) {
                double theta = a * 2.0 * 3.1415926535 / 28.0;
                glVertex2d(pn.winX + cos(theta) * badge_r, pn.winY + sin(theta) * badge_r);
            }
            glEnd();

            // Border ring
            glLineWidth(1.8f);
            glColor4f(1.0f, 1.0f, 1.0f, 0.95f);
            glBegin(GL_LINE_LOOP);
            for (int a = 0; a < 28; ++a) {
                double theta = a * 2.0 * 3.1415926535 / 28.0;
                glVertex2d(pn.winX + cos(theta) * badge_r, pn.winY + sin(theta) * badge_r);
            }
            glEnd();
        }

        // Pass 2: Draw all crisp centered text labels on top of all badges
        glColor3f(1.0f, 0.95f, 0.15f); // Bright yellow bold text
        for (const auto& pn : p_nodes) {
            double char_w = 7.0;
            double text_w = pn.label.length() * char_w;
            glRasterPos2d(pn.winX - text_w * 0.5, pn.winY - 4.0);
            for (char c : pn.label) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
            }
        }

        glFlush();

        glMatrixMode(GL_PROJECTION);
        glPopMatrix();
        glMatrixMode(GL_MODELVIEW);
        glPopMatrix();

        glPopAttrib();
    }

    /**
     * @brief Dedicated Tree-like Graph Window Renderer.
     * Arranges the convex adjacency graph into an intuitive, hierarchical tree view.
     */
    void drawTreeGraphGL(int win_width, int win_height) const {
        glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LINE_BIT | GL_POINT_BIT);
        glDisable(GL_LIGHTING);
        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        int n_nodes = static_cast<int>(nodes.size());

        if (n_nodes == 0) {
            // Display empty / instructions state
            glColor3f(0.35f, 0.75f, 0.95f);
            setfont("helvetica", 18);
            const char* msg1 = "ACD2d Convex Decomposition Graph View";
            glRasterPos2f(win_width * 0.5 - 160.0, win_height * 0.65);
            for (const char* p = msg1; *p; ++p) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *p);

            glColor3f(0.70f, 0.75f, 0.85f);
            setfont("helvetica", 12);
            const char* msg2 = "No active decomposition. Run decomposition in the main window:";
            const char* msg3 = "  Press 'd' / 'D'  : Approximate Convex Decomposition (ACD)";
            const char* msg4 = "  Press 'i' / '`'  : Drake IRIS Convex Region Inflation";
            const char* msg5 = "  Press 'v' / 'V'  : Vertex Clique Cover (VCC Delaunay / Extension)";
            glRasterPos2f(win_width * 0.5 - 180.0, win_height * 0.52);
            for (const char* p = msg2; *p; ++p) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *p);
            glRasterPos2f(win_width * 0.5 - 180.0, win_height * 0.44);
            for (const char* p = msg3; *p; ++p) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *p);
            glRasterPos2f(win_width * 0.5 - 180.0, win_height * 0.38);
            for (const char* p = msg4; *p; ++p) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *p);
            glRasterPos2f(win_width * 0.5 - 180.0, win_height * 0.32);
            for (const char* p = msg5; *p; ++p) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *p);

            glPopAttrib();
            return;
        }

        // 1. Build BFS Hierarchical Layers (Tree Layout)
        std::vector<int> depth(n_nodes, -1);
        std::vector<std::vector<int>> layers;
        std::vector<bool> visited(n_nodes, false);

        for (int start = 0; start < n_nodes; ++start) {
            if (visited[start]) continue;
            std::queue<int> q;
            q.push(start);
            visited[start] = true;
            depth[start] = 0;

            while (!q.empty()) {
                int u = q.front();
                q.pop();
                int d = depth[u];
                if (d >= static_cast<int>(layers.size())) {
                    layers.resize(d + 1);
                }
                layers[d].push_back(u);

                for (const auto& e : nodes[u].adj) {
                    int v = e.target;
                    if (!visited[v]) {
                        visited[v] = true;
                        depth[v] = d + 1;
                        q.push(v);
                    }
                }
            }
        }

        struct ScreenPos { double x, y; };
        std::vector<ScreenPos> node_pos(n_nodes);

        int num_layers = static_cast<int>(layers.size());
        double margin_top = 70.0;
        double margin_bottom = 50.0;
        double margin_left = 60.0;
        double margin_right = 60.0;

        double avail_w = win_width - margin_left - margin_right;
        double avail_h = win_height - margin_top - margin_bottom;
        double layer_step = (num_layers > 1) ? (avail_h / (num_layers - 1)) : 0.0;

        for (int d = 0; d < num_layers; ++d) {
            int count_in_layer = static_cast<int>(layers[d].size());
            double y = (num_layers > 1) 
                ? (win_height - margin_top - d * layer_step)
                : (win_height * 0.5);

            for (int i = 0; i < count_in_layer; ++i) {
                int u = layers[d][i];
                double x = margin_left + (i + 1) * (avail_w / (count_in_layer + 1));
                node_pos[u] = {x, y};
            }
        }

        // 2. Draw Top Info Header
        glColor3f(0.30f, 0.85f, 1.0f);
        setfont("helvetica", 18);
        std::string header_title = "Convex Adjacency Tree Graph [" + decomposition_type + "]";
        glRasterPos2f(25.0, win_height - 30.0);
        for (char c : header_title) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);

        glColor3f(0.70f, 0.75f, 0.82f);
        setfont("helvetica", 12);
        std::stringstream ss_info;
        ss_info << "Vertices: " << n_nodes 
                << " | Directed Edges: " << getTotalDirectedEdges() 
                << " (weight = 0.0) | Hierarchy Levels: " << num_layers;
        std::string info_str = ss_info.str();
        glRasterPos2f(25.0, win_height - 50.0);
        for (char c : info_str) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);

        // Palette for node colors (matches polygon coloring)
        static float palette[][3] = {
            {0.92f, 0.28f, 0.28f}, // 0: Crimson Red
            {0.20f, 0.76f, 0.38f}, // 1: Emerald Green
            {0.25f, 0.52f, 0.92f}, // 2: Royal Blue
            {0.68f, 0.28f, 0.88f}, // 3: Rich Purple / Violet
            {0.96f, 0.55f, 0.15f}, // 4: Vivid Orange
            {0.15f, 0.80f, 0.85f}, // 5: Cyan Turquoise
            {0.95f, 0.35f, 0.65f}, // 6: Hot Pink
            {0.55f, 0.80f, 0.20f}, // 7: Lime
            {0.45f, 0.35f, 0.85f}, // 8: Deep Indigo
            {0.95f, 0.72f, 0.15f}, // 9: Amber Gold
            {0.20f, 0.70f, 0.60f}, // 10: Teal
            {0.85f, 0.35f, 0.55f}  // 11: Magenta
        };
        int num_palette = sizeof(palette) / sizeof(palette[0]);

        double node_radius = 20.0;

        // 3. Draw Directed Edges with Arrows
        glLineWidth(2.0f);
        for (int u = 0; u < n_nodes; ++u) {
            double x1 = node_pos[u].x;
            double y1 = node_pos[u].y;

            for (const auto& e : nodes[u].adj) {
                int v = e.target;
                double x2 = node_pos[v].x;
                double y2 = node_pos[v].y;

                double dx = x2 - x1;
                double dy = y2 - y1;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist < 1.0) continue;

                double ux = dx / dist;
                double uy = dy / dist;
                double nx = -uy;
                double ny = ux;

                bool same_level = (depth[u] == depth[v]);

                if (same_level) {
                    // Draw curved arc for horizontal neighbors
                    int idx_u = 0, idx_v = 0;
                    const auto& layer = layers[depth[u]];
                    for (int k = 0; k < static_cast<int>(layer.size()); ++k) {
                        if (layer[k] == u) idx_u = k;
                        if (layer[k] == v) idx_v = k;
                    }
                    int span = std::abs(idx_u - idx_v);
                    double arc_height = (u < v ? 1.0 : -1.0) * (span * 28.0 + 10.0);
                    double mid_x = 0.5 * (x1 + x2);
                    double mid_y = 0.5 * (y1 + y2) + arc_height;

                    glColor4f(0.95f, 0.60f, 0.20f, 0.75f);
                    glBegin(GL_LINE_STRIP);
                    for (int s = 0; s <= 16; ++s) {
                        double t = s / 16.0;
                        double px = (1 - t) * (1 - t) * x1 + 2 * (1 - t) * t * mid_x + t * t * x2;
                        double py = (1 - t) * (1 - t) * y1 + 2 * (1 - t) * t * mid_y + t * t * y2;
                        glVertex2d(px, py);
                    }
                    glEnd();

                    // Arrowhead on arc near target
                    double t_arr = 0.85;
                    double ax = (1 - t_arr) * (1 - t_arr) * x1 + 2 * (1 - t_arr) * t_arr * mid_x + t_arr * t_arr * x2;
                    double ay = (1 - t_arr) * (1 - t_arr) * y1 + 2 * (1 - t_arr) * t_arr * mid_y + t_arr * t_arr * y2;
                    double tangent_x = 2 * (1 - t_arr) * (mid_x - x1) + 2 * t_arr * (x2 - mid_x);
                    double tangent_y = 2 * (1 - t_arr) * (mid_y - y1) + 2 * t_arr * (y2 - mid_y);
                    double t_len = std::sqrt(tangent_x * tangent_x + tangent_y * tangent_y);
                    if (t_len > 1e-4) {
                        tangent_x /= t_len;
                        tangent_y /= t_len;
                        double norm_x = -tangent_y;
                        double norm_y = tangent_x;

                        glColor4f(0.95f, 0.60f, 0.20f, 0.95f);
                        glBegin(GL_TRIANGLES);
                        glVertex2d(ax, ay);
                        glVertex2d(ax - tangent_x * 8.0 + norm_x * 4.5, ay - tangent_y * 8.0 + norm_y * 4.5);
                        glVertex2d(ax - tangent_x * 8.0 - norm_x * 4.5, ay - tangent_y * 8.0 - norm_y * 4.5);
                        glEnd();
                    }
                } else {
                    // Draw tree hierarchy straight edge
                    double sx = x1 + ux * node_radius;
                    double sy = y1 + uy * node_radius;
                    double ex = x2 - ux * (node_radius + 4.0);
                    double ey = y2 - uy * (node_radius + 4.0);

                    glColor4f(0.30f, 0.70f, 0.95f, 0.75f);
                    glBegin(GL_LINES);
                    glVertex2d(sx, sy);
                    glVertex2d(ex, ey);
                    glEnd();

                    // Arrowhead pointing towards child/adjacent node
                    double tip_x = x2 - ux * (node_radius + 1.0);
                    double tip_y = y2 - uy * (node_radius + 1.0);
                    double base_lx = tip_x - ux * 8.0 + nx * 4.5;
                    double base_ly = tip_y - uy * 8.0 + ny * 4.5;
                    double base_rx = tip_x - ux * 8.0 - nx * 4.5;
                    double base_ry = tip_y - uy * 8.0 - ny * 4.5;

                    glColor4f(0.20f, 0.85f, 0.95f, 0.95f);
                    glBegin(GL_TRIANGLES);
                    glVertex2d(tip_x, tip_y);
                    glVertex2d(base_lx, base_ly);
                    glVertex2d(base_rx, base_ry);
                    glEnd();
                }
            }
        }

        // 4. Draw Tree Node Cards / Badges
        for (int u = 0; u < n_nodes; ++u) {
            double x = node_pos[u].x;
            double y = node_pos[u].y;
            float* col = palette[u % num_palette];

            // Outer drop-shadow
            glColor4f(0.04f, 0.05f, 0.08f, 0.60f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2d(x + 2.0, y - 2.0);
            for (int a = 0; a <= 28; ++a) {
                double theta = a * 2.0 * 3.1415926535 / 28.0;
                glVertex2d(x + 2.0 + cos(theta) * (node_radius + 2.0), y - 2.0 + sin(theta) * (node_radius + 2.0));
            }
            glEnd();

            // Colored Node Body (matches polygon color in main view)
            glColor4f(col[0], col[1], col[2], 0.95f);
            glBegin(GL_TRIANGLE_FAN);
            glVertex2d(x, y);
            for (int a = 0; a <= 28; ++a) {
                double theta = a * 2.0 * 3.1415926535 / 28.0;
                glVertex2d(x + cos(theta) * node_radius, y + sin(theta) * node_radius);
            }
            glEnd();

            // Crisp White Outline
            glLineWidth(2.0f);
            glColor4f(1.0f, 1.0f, 1.0f, 0.95f);
            glBegin(GL_LINE_LOOP);
            for (int a = 0; a < 28; ++a) {
                double theta = a * 2.0 * 3.1415926535 / 28.0;
                glVertex2d(x + cos(theta) * node_radius, y + sin(theta) * node_radius);
            }
            glEnd();

            // Bold Centered Label Text
            setfont("helvetica", 12);
            glColor3f(1.0f, 1.0f, 1.0f);
            double str_w = nodes[u].label.length() * 7.0;
            glRasterPos2f(x - str_w * 0.5, y - 4.5);
            for (char c : nodes[u].label) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, c);
            }

            // Subtitle label below node (Centroid coordinates)
            char sub_buf[32];
            std::sprintf(sub_buf, "(%.1f, %.1f)", nodes[u].centroid[0], nodes[u].centroid[1]);
            double sub_w = std::string(sub_buf).length() * 5.5;
            glColor3f(0.80f, 0.85f, 0.95f);
            glRasterPos2f(x - sub_w * 0.5, y - node_radius - 14.0);
            for (char* p = sub_buf; *p; ++p) {
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *p);
            }
        }

        glPopAttrib();
    }
};

} // namespace acd2d

#endif // _ACD2D_GRAPH_H_
