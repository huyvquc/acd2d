//------------------------------------------------------------------------------
//  Copyright 2007-2008 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _ACD2d_GUI_DRAW_H_
#define _ACD2d_GUI_DRAW_H_

/*
   acd2d_draw.h
   a list of functions that draw path, polygon and vertices using
   opengGL

   copyright (c) 2005, Parasol Lab, Texas A&M University
   
   includes function prototypes for main.cpp
*/

///////////////////////////////////////////////////////////////////////////////
// acd2d headers
#include <random>
#include "acd2d.h"
#include "acd2d_stat.h"
#include "acd2d_bridge.h"
#include "acd2d_iris.h"
#include "acd2d_vcc.h"
#include "acd2d_graph.h"

///////////////////////////////////////////////////////////////////////////////
// openggl headers
#include <GL/gli.h>
#include <GL/gliFont.h>

//extern cd_state state;
extern double box[4]; //bbox, defined in acd2d_main_gui.h
extern Point2d O;
extern bool g_showIRIS;
extern bool g_showVCC;
extern bool g_vccUseExtension;
extern bool g_showLabels;
extern bool g_showGraph;
extern bool g_showWeights;
extern acd2d::ConvexGraph g_activeGraph;
int colorid=-1;

inline void updateAcdGraph(cd_2d& cd2d, double decomp_time_sec = -1.0) {
    clock_t g_start = clock();
    std::vector<std::vector<Point2d>> pieces;
    for (const auto& polys : cd2d.getDoneList()) {
        for (const auto& poly : polys) {
            std::vector<Point2d> piece;
            cd_vertex* ptr = poly.getHead();
            if (ptr != NULL) {
                do {
                    piece.push_back(ptr->getPos());
                    ptr = ptr->getNext();
                } while (ptr != poly.getHead());
            }
            if (piece.size() >= 3) pieces.push_back(piece);
        }
    }
    if (!pieces.empty()) {
        g_activeGraph.buildFromPolygons(pieces, "ACD");
    } else {
        g_activeGraph.clear();
    }
    double graph_time = (double)(clock() - g_start) / CLOCKS_PER_SEC;

    if (decomp_time_sec >= 0.0) {
        double total_time = decomp_time_sec + graph_time;
        std::cout << "=========================================================\n"
                  << " [ACD & Graph Timing Summary]\n"
                  << "   - ACD Decomposition Time : " << std::fixed << std::setprecision(4) << decomp_time_sec << " s (" << pieces.size() << " pieces)\n"
                  << "   - Graph Creation Time    : " << std::fixed << std::setprecision(3) << (graph_time * 1000.0) << " ms (" << std::fixed << std::setprecision(5) << graph_time << " s)\n"
                  << "   - Total Time             : " << std::fixed << std::setprecision(4) << total_time << " s\n"
                  << "=========================================================\n" << std::flush;
    }
}

inline void drawPoly(const cd_poly& poly) 
{
    //draw vertices
    cd_vertex* ptr=poly.getHead();
    if(ptr==NULL) return;

    glBegin(GL_LINE_LOOP);
    do{
        const Point2d& pt=ptr->getPos();
        glVertex2d(pt[0],pt[1]);
        ptr=ptr->getNext();
    }while( ptr!=poly.getHead() );
    glEnd();
}

void drawPolyNormal(const cd_poly& poly)
{
    //draw normal
    cd_vertex* ptr=poly.getHead();
    if(ptr==NULL) return;

	float length=min(box[1]-box[0],box[3]-box[2])/25;
    glColor3d(0.7,0.5,0);
    glBegin(GL_LINES);
    do{
        const Point2d& pt1=ptr->getPos();
        const Point2d& pt2=ptr->getNext()->getPos();
        double x=(pt1[0]+pt2[0])/2;
        double y=(pt1[1]+pt2[1])/2;
        glVertex2d(x,y);
        const Vector2d& n=ptr->getNormal()*length;
        x=x+n[0];
        y=y+n[1];
        glVertex2d(x,y);
        ptr=ptr->getNext();
    }while( ptr!=poly.getHead() );
    glEnd();
}


inline void drawpolylist(const list<cd_polygon>& pl)
{
    glDisable(GL_LIGHTING);
    list<cd_polygon>::const_iterator ips=pl.begin();
    for( ;ips!=pl.end();ips++ ){
        glPushMatrix();
        const cd_polygon& polys=*ips;
        for( PLYCIT ip=polys.begin();ip!=polys.end();ip++ ){ //for each poly
            drawPoly(*ip);
            glTranslated(0,0,0.01);
        }
        glPopMatrix();
    }   
}

extern cd_polygon g_orig_poly;
extern bool g_showColor;

void drawFill(const cd_polygon& pl);

inline void drawColoredDoneList(const list<cd_polygon>& done_list)
{
    if (done_list.empty()) return;

    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 1. Draw colored interior fills strictly clipped to valid polygon interior via Stencil Test
    if (colorid >= 0) {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
    }

    int idx = 0;
    for (const auto& polys : done_list) {
        float col[3];
        acd2d::getDistinctColor(idx, col);
        glColor4f(col[0], col[1], col[2], 0.50f);
        drawFill(polys);
        idx++;
    }

    // 2. Draw partition boundary lines between pieces (clipped to polygon interior so they do not cross into holes)
    idx = 0;
    glLineWidth(1.2f);
    for (const auto& polys : done_list) {
        float col[3];
        acd2d::getDistinctColor(idx, col);
        glColor3f(col[0] * 0.45f, col[1] * 0.45f, col[2] * 0.45f);
        for (const auto& poly : polys) {
            drawPoly(poly);
        }
        idx++;
    }

    if (colorid >= 0) {
        glDisable(GL_STENCIL_TEST);
    }

    glPopAttrib();
}

inline void draw(cd_2d& cd2d)
{
    glDisable(GL_LIGHTING);

    if (!g_showColor) {
        // Classic uncolored wireframe rendering
        glPushAttrib(GL_CURRENT_BIT);
        glColor3d(0.85, 0.85, 0.95);
        glCallList(colorid);
        glPopAttrib();

        // draw todo list
        if (!cd2d.getTodoList().empty()) {
            glTranslated(0, 0, 10);
            glColor3f(0.1f, 0.1f, 0.1f);
            glLineWidth(1.5f);
            drawpolylist(cd2d.getTodoList());
        }

        // draw done list
        if (!cd2d.getDoneList().empty()) {
            glTranslated(0, 0, 10);
            glPushAttrib(GL_CURRENT_BIT);
            glColor3f(0.2f, 0.2f, 0.2f);
            glLineWidth(1.5f);
            drawpolylist(cd2d.getDoneList());
            glPopAttrib();
        }

        // Always draw original polygon boundary and hole outlines on top
        if (!g_orig_poly.empty()) {
            glTranslated(0, 0, 15);
            glLineWidth(2.2f);
            glColor3f(0.08f, 0.08f, 0.08f);
            for (const auto& poly : g_orig_poly) {
                drawPoly(poly);
            }
        }
        return;
    }

    // If done_list is empty, draw default neutral background fill
    if (cd2d.getDoneList().empty()) {
        glPushAttrib(GL_CURRENT_BIT);
        glColor3d(0.85, 0.85, 0.95);
        glCallList(colorid);
        glPopAttrib();
    }

    // Draw decomposed pieces with distinct colors (stencil-clipped)
    if (!cd2d.getDoneList().empty()) {
        glTranslated(0, 0, 5);
        drawColoredDoneList(cd2d.getDoneList());
    }

    // Draw remaining todo list pieces
    if (!cd2d.getTodoList().empty()) {
        if (!cd2d.getDoneList().empty()) {
            glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT);
            if (colorid >= 0) {
                glEnable(GL_STENCIL_TEST);
                glStencilFunc(GL_EQUAL, 1, 0xFF);
                glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            }
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glColor4f(0.85f, 0.85f, 0.95f, 0.75f);
            for (const auto& p : cd2d.getTodoList()) {
                drawFill(p);
            }
            if (colorid >= 0) {
                glDisable(GL_STENCIL_TEST);
            }
            glPopAttrib();
        }

        glTranslated(0, 0, 10);
        glColor3f(0.1f, 0.1f, 0.1f);
        glLineWidth(1.5f);
        drawpolylist(cd2d.getTodoList());
    }

    // Always draw original polygon boundary and ALL hole outlines on top with bold black lines
    if (!g_orig_poly.empty()) {
        glTranslated(0, 0, 15);
        glLineWidth(2.2f);
        glColor3f(0.08f, 0.08f, 0.08f);
        for (const auto& poly : g_orig_poly) {
            drawPoly(poly);
        }
    }
}

inline void drawPolyListNormal(const list<cd_polygon>& pl)
{
    list<cd_polygon>::const_iterator ips=pl.begin();
    for( ;ips!=pl.end();ips++ ){
        glPushMatrix();
        const cd_polygon& polys=*ips;
        for( PLYCIT ip=polys.begin();ip!=polys.end();ip++ ){ //for each poly
            drawPolyNormal(*ip);
            glTranslated(0,0,0.01);
        }
        glPopMatrix();
    }
}

inline void drawNormal(cd_2d& cd2d)
{
    //if( state.show_normal==false ) return;
    drawPolyListNormal(cd2d.getTodoList());
    drawPolyListNormal(cd2d.getDoneList());
}

inline void drawPolyText(cd_2d& cd2d)
{
    char value[128];
    
    //////////////////////////////////////////////
    glTranslated(0,-0.5,0);
    glColor3f(0.2f,0.2f,0.5f);
    sprintf(value,"%d",countVertices(cd2d));
    drawstr(0.2f,0,0,"Number of Vertices: ");
    glColor3f(1,0,0);
    drawstr(5,0,0,value);

    //////////////////////////////////////////////
    glTranslated(0,-0.5,0);
    glColor3f(0.2f,0.2f,0.5f);
    sprintf(value,"%d",countNotches(cd2d));
    drawstr(0.2f,0,0,"Number of Notches: ");
    glColor3f(1,0,0);
    drawstr(5,0,0,value);

    //////////////////////////////////////////////
    glTranslated(0,-0.5,0);
    glColor3f(0.2f,0.2f,0.5f);
    sprintf(value,"%d",(int)cd2d.getDoneList().size());
    drawstr(0.2f,0,0,"Convex Components: ");
    glColor3f(1,0,0);
    drawstr(5,0,0,value);
}

inline void drawTextInfo(cd_2d& cd2d)
{
    glPushAttrib(GL_CURRENT_BIT);

    //draw reference axis
    glMatrixMode(GL_PROJECTION); //change to Ortho view
    glPushMatrix(); 
    glLoadIdentity();
    gluOrtho2D(0,20,0,20);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    
    //Draw text
    glTranslated(0,20,0.1);
    drawPolyText(cd2d);
    
    glPopMatrix();

    //pop GL_PROJECTION
    glMatrixMode(GL_PROJECTION); //change to Pers view
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopAttrib();
}

inline void drawBridge(const cd_poly & poly)
{
    list<cd_bridge*> bridges;

    //get bridges
    poly.getBridges(bridges);
    if( bridges.empty() ) return;

    //draw bridges
    typedef list<cd_bridge*>::const_iterator BIT;
    glPushAttrib(GL_CURRENT_BIT);
    {
        glBegin(GL_LINES);
        for(BIT ib=bridges.begin();ib!=bridges.end();ib++ ){
            const Point2d& p1=(*ib)->v1->getPos();
            const Point2d& p2=(*ib)->v2->getPos();
            glVertex2d(p1[0],p1[1]);
            glVertex2d(p2[0],p2[1]);
        }
        glEnd();
    }

    {
        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3d(0,1,0);
        for(BIT ib=bridges.begin();ib!=bridges.end();ib++ ){
            const Point2d& p1=(*ib)->v1->getPos();
            const Point2d& p2=(*ib)->v2->getPos();
            glVertex2d(p1[0],p1[1]);
            glVertex2d(p2[0],p2[1]);
        }
        glEnd();
    }

    { //mark the max concavity notch in the pocket
        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3d(1,0,0);
        for(BIT ib=bridges.begin();ib!=bridges.end();ib++ ){
            cd_bridge * b=*ib;
            if( b->max_r==NULL ) continue;
            const Point2d& p1=b->max_r->getPos();
            glVertex2d(p1[0],p1[1]);
        }
        glEnd();
    }

    glPopAttrib();
}

inline void drawBridge(cd_2d& cd2d)
{
    glTranslated(0,0,1);
    const list<cd_polygon>& todo=cd2d.getTodoList();    
    typedef list<cd_polygon>::const_iterator PLIT;
    typedef cd_polygon::const_iterator PIT;
    glColor3f(0.9f,0.5f,0.1f);
    {for( PLIT ipl=todo.begin();ipl!=todo.end();ipl++ ){
        const cd_polygon& pg=*ipl;
        for(PIT ip=pg.begin();ip!=pg.end();ip++){
            if( ip->getType()==cd_poly::POUT ) 
                drawBridge(*ip);
        }
    }}
}

///////////////////////////////////////////////////////////////////////////
#ifdef WIN32
extern "C"{
#include "triangulate.h"
}
#else 
#include "triangulate.h"
#endif

void drawFill(const cd_polygon& pl)
{
    typedef cd_polygon::const_iterator   PIT;
    int ringN = pl.size();           // number of rings
    if (ringN == 0) return;

    int* ringVN = new int[ringN];     // number of vertices for each ring
    int vN = 0;                        // total number of vertices
    {
        int r = 0;
        for (PIT ip = pl.begin(); ip != pl.end(); ip++, r++) {
            int count = 0;
            cd_vertex* ptr = ip->getHead();
            if (ptr != NULL) {
                do {
                    count++;
                    ptr = ptr->getNext();
                } while (ptr != ip->getHead());
            }
            ringVN[r] = count;
            vN += count;
        }
    }
    
    if (vN < 3) {
        delete[] ringVN;
        return;
    }

    int tN = vN + 2 * ringN;          // (n-2)+2*(#holes)
    double* V = new double[vN * 2];   // vertex positions
    int* T = new int[3 * tN];         // resulting triangles
    
    // copy vertices
    {
        int i = 0;
        for (PIT ip = pl.begin(); ip != pl.end(); ip++) {
            cd_vertex* ptr = ip->getHead();
            if (ptr != NULL) {
                do {
                    Point2d pt = ptr->getPos();
                    V[i * 2]     = pt[0];
                    V[i * 2 + 1] = pt[1];
                    ptr = ptr->getNext();
                    i++;
                } while (ptr != ip->getHead());
            }
        }
    }
    
    FIST_PolygonalArray(ringN, ringVN, (double (*)[2])V, &tN, (int (*)[3])T);
    {
        glBegin(GL_TRIANGLES);
        for (int i = 0; i < tN; i++) {
            for (int j = 0; j < 3; j++) {
                int tid = T[i * 3 + j];
                if (tid >= 0 && tid < vN) {
                    glVertex2d(V[tid * 2], V[tid * 2 + 1]);
                }
            }
        }
        glEnd();
    }

    delete[] ringVN;
    delete[] V;
    delete[] T;
}

void Fill(const list<cd_polygon>& pl)
{
    typedef list<cd_polygon>::const_iterator PIT;
    
    if(colorid>=0) glDeleteLists(colorid,1);
    colorid=glGenLists(1);
    glNewList(colorid,GL_COMPILE);
    for( PIT ip=pl.begin();ip!=pl.end();ip++){
        drawFill(*ip);
    }
    glEndList();


}

static std::vector<std::vector<Eigen::Vector2d>> g_irisComputedRegions;
static std::vector<Eigen::Vector2d> g_irisComputedSeeds;
static std::vector<drake::geometry::optimization::HPolyhedron> g_irisHPolyhedrons;
static std::vector<std::vector<Point2d>> g_vccComputedRegions;

inline void updateIrisGraph() {
    std::vector<std::vector<Point2d>> pieces;
    std::vector<Point2d> seeds;
    for (size_t i = 0; i < g_irisComputedRegions.size(); ++i) {
        const auto& r = g_irisComputedRegions[i];
        std::vector<Point2d> piece;
        for (const auto& v : r) {
            piece.push_back(Point2d(v(0), v(1)));
        }
        if (piece.size() >= 3) {
            pieces.push_back(piece);
            if (i < g_irisComputedSeeds.size()) {
                seeds.push_back(Point2d(g_irisComputedSeeds[i](0), g_irisComputedSeeds[i](1)));
            }
        }
    }
    if (!pieces.empty()) {
        g_activeGraph.buildFromPolygons(pieces, "IRIS", seeds);
    } else {
        g_activeGraph.clear();
    }
}

inline void updateVccGraph() {
    if (!g_vccComputedRegions.empty()) {
        g_activeGraph.buildFromPolygons(g_vccComputedRegions, g_vccUseExtension ? "VCC-Extension" : "VCC-Delaunay");
    } else {
        g_activeGraph.clear();
    }
}

inline void resetIRIS() {
    g_irisComputedRegions.clear();
    g_irisComputedSeeds.clear();
    g_irisHPolyhedrons.clear();
    if (g_activeGraph.decomposition_type == "IRIS") {
        g_activeGraph.clear();
    }
}

inline bool IsPointInPolygon(const Eigen::Vector2d& pt, const std::vector<Eigen::Vector2d>& poly) {
    bool inside = false;
    size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double xi = poly[i](0), yi = poly[i](1);
        double xj = poly[j](0), yj = poly[j](1);
        bool intersect = ((yi > pt(1)) != (yj > pt(1))) &&
                         (pt(0) < (xj - xi) * (pt(1) - yi) / (yj - yi + 1e-12) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

inline double computeIrisFreeSpaceCoverage(
    const std::vector<Eigen::Vector2d>& outer_vertices,
    const std::vector<std::vector<Eigen::Vector2d>>& holes_vertices,
    const std::vector<drake::geometry::optimization::HPolyhedron>& regions,
    const double bbox[4],
    int num_samples = 2000,
    Eigen::Vector2d* out_uncovered_seed = nullptr
) {
    if (outer_vertices.empty()) return 0.0;

    auto IsValidFreeSpace = [&](const Eigen::Vector2d& pt) -> bool {
        if (!IsPointInPolygon(pt, outer_vertices)) return false;
        for (const auto& hole : holes_vertices) {
            if (IsPointInPolygon(pt, hole)) return false;
        }
        return true;
    };

    static std::random_device rd;
    static std::mt19937 rng(rd());
    std::uniform_real_distribution<double> dist_x(bbox[0], bbox[1]);
    std::uniform_real_distribution<double> dist_y(bbox[2], bbox[3]);

    int valid_samples = 0;
    int covered_samples = 0;
    std::vector<Eigen::Vector2d> uncovered_candidates;

    int max_trials = num_samples * 50;
    int trial = 0;
    while (valid_samples < num_samples && trial < max_trials) {
        trial++;
        Eigen::Vector2d pt(dist_x(rng), dist_y(rng));
        if (!IsValidFreeSpace(pt)) continue;

        valid_samples++;
        bool is_covered = false;
        for (const auto& r : regions) {
            if (r.PointInSet(pt, 1e-2)) {
                is_covered = true;
                break;
            }
        }

        if (is_covered) {
            covered_samples++;
        } else {
            uncovered_candidates.push_back(pt);
        }
    }

    if (out_uncovered_seed && !uncovered_candidates.empty()) {
        std::uniform_int_distribution<size_t> idx_dist(0, uncovered_candidates.size() - 1);
        *out_uncovered_seed = uncovered_candidates[idx_dist(rng)];
    }

    if (valid_samples == 0) return 1.0;
    return static_cast<double>(covered_samples) / static_cast<double>(valid_samples);
}

inline void stepIRIS(cd_2d& cd2d) {
    const list<cd_polygon>& todo = cd2d.getTodoList();
    if (todo.empty() || todo.begin()->empty()) return;

    const cd_polygon& poly = *todo.begin();

    std::vector<std::vector<Eigen::Vector2d>> all_rings;
    std::vector<Eigen::Vector2d> outer_vertices;
    std::vector<std::vector<Eigen::Vector2d>> holes_vertices;

    for (cd_polygon::const_iterator ip = poly.begin(); ip != poly.end(); ++ip) {
        std::vector<Eigen::Vector2d> ring_verts;
        cd_vertex* ptr = ip->getHead();
        if (ptr != NULL) {
            do {
                const Point2d& pt = ptr->getPos();
                ring_verts.push_back(Eigen::Vector2d(pt[0], pt[1]));
                ptr = ptr->getNext();
            } while (ptr != ip->getHead());
        }

        if (!ring_verts.empty()) {
            all_rings.push_back(ring_verts);
            if (ip->getType() == cd_poly::POUT || outer_vertices.empty()) {
                outer_vertices = ring_verts;
            } else {
                holes_vertices.push_back(ring_verts);
            }
        }
    }

    // Check current space coverage (target: >= 98%)
    Eigen::Vector2d cand;
    double current_coverage = computeIrisFreeSpaceCoverage(
        outer_vertices, holes_vertices, g_irisHPolyhedrons, box, 2000, &cand
    );

    if (current_coverage >= 0.98) {
        std::cout << "- Step IRIS: Space coverage is already at " 
                  << std::fixed << std::setprecision(1) << (current_coverage * 100.0) 
                  << "% (>= 98% target reached with " << g_irisComputedRegions.size() 
                  << " regions)" << std::endl;
        return;
    }

    // Domain & Obstacles
    Eigen::Matrix<double, 4, 2> A_dom;
    A_dom <<  1,  0, -1,  0,  0,  1,  0, -1;
    Eigen::Vector4d b_dom;
    double margin = (box[1] - box[0] + box[3] - box[2]) * 0.1;
    if (margin < 1.0) margin = 1.0;
    b_dom << box[1] + margin, -box[0] + margin, box[3] + margin, -box[2] + margin;
    drake::geometry::optimization::HPolyhedron domain(A_dom, b_dom);

    std::vector<drake::geometry::optimization::HPolyhedron> obstacles;
    for (const auto& ring : all_rings) {
        size_t n = ring.size();
        for (size_t i = 0; i < n; ++i) {
            const Eigen::Vector2d& p1 = ring[i];
            const Eigen::Vector2d& p2 = ring[(i + 1) % n];
            if ((p2 - p1).squaredNorm() < 1e-8) continue;
            
            Eigen::Vector2d edge = p2 - p1;
            double L = edge.norm();
            Eigen::Vector2d u = edge / L;
            Eigen::Vector2d v(-u(1), u(0));
            double v_dot_p1 = v.dot(p1);
            double u_dot_p1 = u.dot(p1);
            double eps = 1e-6;

            Eigen::Matrix<double, 4, 2> A_obs;
            A_obs <<  v(0),  v(1), -v(0), -v(1), u(0), u(1), -u(0), -u(1);
            Eigen::Vector4d b_obs(v_dot_p1 + eps, -v_dot_p1 + eps, u_dot_p1 + L, -u_dot_p1);
            obstacles.emplace_back(A_obs, b_obs);
        }
    }

    // Attempt inflation from uncovered seed candidates
    for (int retry = 0; retry < 50; ++retry) {
        try {
            drake::geometry::optimization::HPolyhedron region = acd2d::IrisWrapper::InflateRegion(obstacles, cand, domain);
            std::vector<Eigen::Vector2d> verts = acd2d::IrisWrapper::GetHPolyhedronVertices(region);
            if (verts.size() >= 3) {
                g_irisHPolyhedrons.push_back(region);
                g_irisComputedRegions.push_back(verts);
                g_irisComputedSeeds.push_back(cand);
                g_showIRIS = true;
                updateIrisGraph();

                double new_cov = computeIrisFreeSpaceCoverage(
                    outer_vertices, holes_vertices, g_irisHPolyhedrons, box, 2000, nullptr
                );
                std::cout << "- Step IRIS: Inflated region #" << g_irisComputedRegions.size()
                          << " at seed (" << cand(0) << ", " << cand(1) 
                          << "). Current coverage: " << std::fixed << std::setprecision(1) 
                          << (new_cov * 100.0) << "% / 98.0%" << std::endl;
                return;
            }
        } catch (...) {}

        computeIrisFreeSpaceCoverage(outer_vertices, holes_vertices, g_irisHPolyhedrons, box, 2000, &cand);
    }

    std::cout << "- Step IRIS: Could not find further valid seeds. Coverage: " 
              << std::fixed << std::setprecision(1) << (current_coverage * 100.0) << "%." << std::endl;
}

inline void runIRIS(cd_2d& cd2d) {
    std::cout << "- Running IRIS until >= 98% space coverage is reached..." << std::endl;
    int max_regions = 100;
    while (g_irisComputedRegions.size() < (size_t)max_regions) {
        size_t prev_count = g_irisComputedRegions.size();
        stepIRIS(cd2d);
        if (g_irisComputedRegions.size() == prev_count) {
            break;
        }
    }
    updateIrisGraph();
}

inline void drawIRIS(cd_2d& cd2d)
{
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // If no step-by-step regions computed yet, compute full decomposition once
    if (g_irisComputedRegions.empty()) {
        const list<cd_polygon>& todo = cd2d.getTodoList();
        if (!todo.empty() && !todo.begin()->empty()) {
            const cd_polygon& poly = *todo.begin();
            std::vector<std::vector<Eigen::Vector2d>> all_rings;
            std::vector<Eigen::Vector2d> outer_vertices;
            std::vector<std::vector<Eigen::Vector2d>> holes_vertices;

            for (cd_polygon::const_iterator ip = poly.begin(); ip != poly.end(); ++ip) {
                std::vector<Eigen::Vector2d> ring_verts;
                cd_vertex* ptr = ip->getHead();
                if (ptr != NULL) {
                    do {
                        const Point2d& pt = ptr->getPos();
                        ring_verts.push_back(Eigen::Vector2d(pt[0], pt[1]));
                        ptr = ptr->getNext();
                    } while (ptr != ip->getHead());
                }

                if (!ring_verts.empty()) {
                    all_rings.push_back(ring_verts);
                    if (ip->getType() == cd_poly::POUT || outer_vertices.empty()) {
                        outer_vertices = ring_verts;
                    } else {
                        holes_vertices.push_back(ring_verts);
                    }
                }
            }

            auto IsValidFreeSpaceSeed = [&](const Eigen::Vector2d& cand) -> bool {
                if (!IsPointInPolygon(cand, outer_vertices)) return false;
                for (const auto& hole : holes_vertices) {
                    if (IsPointInPolygon(cand, hole)) return false;
                }
                return true;
            };

            std::vector<Eigen::Vector2d> seeds;
            Eigen::Vector2d center_seed(O[0], O[1]);
            if (IsValidFreeSpaceSeed(center_seed)) seeds.push_back(center_seed);

            int ringN = all_rings.size();
            if (ringN > 0) {
                int* ringVN = new int[ringN];
                int vN = 0;
                for (int r = 0; r < ringN; r++) {
                    ringVN[r] = all_rings[r].size();
                    vN += all_rings[r].size();
                }

                if (vN >= 3) {
                    int tN = vN + 2 * ringN;
                    double* V = new double[vN * 2];
                    int* T = new int[3 * tN];

                    int idx = 0;
                    for (int r = 0; r < ringN; r++) {
                        for (size_t i = 0; i < all_rings[r].size(); i++) {
                            V[idx * 2]     = all_rings[r][i](0);
                            V[idx * 2 + 1] = all_rings[r][i](1);
                            idx++;
                        }
                    }

                    FIST_PolygonalArray(ringN, ringVN, (double (*)[2])V, &tN, (int (*)[3])T);
                    for (int i = 0; i < tN; i++) {
                        int id0 = T[i * 3 + 0];
                        int id1 = T[i * 3 + 1];
                        int id2 = T[i * 3 + 2];
                        if (id0 < 0 || id0 >= vN || id1 < 0 || id1 >= vN || id2 < 0 || id2 >= vN) continue;
                        double cx = (V[id0 * 2] + V[id1 * 2] + V[id2 * 2]) / 3.0;
                        double cy = (V[id0 * 2 + 1] + V[id1 * 2 + 1] + V[id2 * 2 + 1]) / 3.0;
                        Eigen::Vector2d tri_seed(cx, cy);
                        if (IsValidFreeSpaceSeed(tri_seed)) seeds.push_back(tri_seed);
                    }
                    delete[] V;
                    delete[] T;
                }
                delete[] ringVN;
            }

            acd2d::IrisWrapper::IrisDecompositionResult decomp = 
                acd2d::IrisWrapper::ComputeIrisDecomposition(all_rings, seeds, box);
            g_irisComputedRegions = decomp.regions;
            g_irisComputedSeeds = decomp.seeds;
            updateIrisGraph();
        }
    }

    for (size_t r = 0; r < g_irisComputedRegions.size(); ++r) {
        const auto& iris_verts = g_irisComputedRegions[r];
        float col[3];
        acd2d::getDistinctColor(static_cast<int>(r), col);

        glColor4f(col[0], col[1], col[2], 0.40f);
        glBegin(GL_TRIANGLE_FAN);
        for (const auto& v : iris_verts) {
            glVertex2d(v(0), v(1));
        }
        glEnd();

        glLineWidth(1.2f);
        glColor3f(col[0], col[1], col[2]);
        glBegin(GL_LINE_LOOP);
        for (const auto& v : iris_verts) {
            glVertex2d(v(0), v(1));
        }
        glEnd();
    }

    glPointSize(8.0f);
    glColor3f(1.0f, 0.2f, 0.2f);
    glBegin(GL_POINTS);
    for (const auto& s : g_irisComputedSeeds) {
        glVertex2d(s(0), s(1));
    }
    glEnd();

    glPopAttrib();
}

inline void resetVCC() {
    g_vccComputedRegions.clear();
    if (g_activeGraph.decomposition_type.rfind("VCC", 0) == 0) {
        g_activeGraph.clear();
    }
}

inline void computeVCC(cd_2d& cd2d, bool use_extension = false) {
    const list<cd_polygon>& todo = cd2d.getTodoList();
    if (todo.empty() || todo.begin()->empty()) return;

    g_vccUseExtension = use_extension;
    acd2d::VccDecompositionResult res = 
        acd2d::VccWrapper::ComputeVccDecomposition(*todo.begin(), use_extension);
    g_vccComputedRegions = res.regions;
    g_showVCC = true;
    updateVccGraph();
}

inline void drawVCC(cd_2d& cd2d)
{
    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LINE_BIT);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (g_vccComputedRegions.empty()) {
        computeVCC(cd2d, g_vccUseExtension);
    }

    for (size_t r = 0; r < g_vccComputedRegions.size(); ++r) {
        const auto& verts = g_vccComputedRegions[r];
        float col[3];
        acd2d::getDistinctColor(static_cast<int>(r), col);

        glColor4f(col[0], col[1], col[2], 0.55f);
        glBegin(GL_TRIANGLE_FAN);
        for (const auto& pt : verts) {
            glVertex2d(pt[0], pt[1]);
        }
        glEnd();

        glLineWidth(1.8f);
        glColor3f(col[0] * 0.75f, col[1] * 0.75f, col[2] * 0.75f);
        glBegin(GL_LINE_LOOP);
        for (const auto& pt : verts) {
            glVertex2d(pt[0], pt[1]);
        }
        glEnd();
    }

    glPopAttrib();
}

#endif //_ACD2d_DRAW_H_

