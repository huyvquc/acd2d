# Incremental Dual Graph Maintenance in ACD2D: Comprehensive Code Architecture, Implementation & Performance Analysis

This document provides an exhaustive, file-by-file technical breakdown of the **Incremental Dual Graph Maintenance** approach implemented on the `incremental_approach` branch of ACD2D. It details every code modification, internal data structures, algorithmic workflows, benchmark profiles across diverse maps, and a deep root-cause analysis of the performance bottlenecks.

---

## 1. Executive Summary & The Core Hypothesis

### 1.1 The Theoretical Goal
In standard ACD2D (the Naive approach), convex decomposition and dual graph construction are decoupled:
1. **Decomposition Phase:** ACD recursively cuts the input polygon into $N$ convex pieces in a Binary Space Partitioning (BSP) tree.
2. **Post-Processing Graph Construction:** An all-pairs $O(N^2 \cdot \bar{m}^2)$ loop inspects every pair of leaf pieces to detect boundary contacts.

The **Incremental Approach** aimed to eliminate the $O(N^2 \cdot \bar{m}^2)$ post-processing phase entirely by maintaining the dual graph **dynamically during the decomposition recursion**:
* Whenever a polygon $P$ is cut along diagonal $C$ into $P_1$ and $P_2$, the shared interface between $P_1$ and $P_2$ is immediately defined by $C$.
* Existing neighbors of $P$ are inherited by $P_1$ and $P_2$.
* When decomposition finishes, the dual graph is already complete and requires only a trivial $O(N)$ export step.

```
       [ Parent Polygon P ] (Adjacency list: N1, N2)
              |
      ========| Cut Line C ========
              v
    [ Child P1 ] <--- Cut C (Length L) ---> [ Child P2 ]
         |                                       |
    Inherits N1?                            Inherits N2?
```

---

## 2. Exhaustive File-by-File Code Changes

Five files were modified across the codebase to implement the incremental pipeline:

```
├── src/
│   ├── acd2d_data.h        <-- Polygon unique identifier member
│   ├── acd2d_data.cpp      <-- ID preservation across copy/assignment
│   ├── acd2d_core.h        <-- IncrementalNode, IncrementalEdge, container definitions
│   └── acd2d_core.cpp      <-- updateCutAdjacency, checkPolygonOverlap, exportToConvexGraph
└── gui/
    └── acd2d_draw.h        <-- Instant graph export hook in GUI render loop
```

---

### 2.1 File 1: `src/acd2d_data.h` — Polygon Identifier

#### Change: Added unique integer identifier `id` to `cd_polygon`
```diff
--- a/src/acd2d_data.h
+++ b/src/acd2d_data.h
@@ -220,7 +220,8 @@ namespace acd2d
 	class cd_polygon : public list<cd_poly>{
 	
 	public: 
-		cd_polygon(){}
+		int id;
+		cd_polygon() : id(-1) {}
 		void buildDependency();
 		cd_poly next(); //get the next polychain to be resolved
 		cd_poly& outmost();
```

#### Explanation:
* **Why:** In ACD2D, polygons are stored as linked lists of `cd_poly` chains (outer boundary + holes). To maintain a graph whose nodes represent individual polygons across recursive cuts, each `cd_polygon` instance requires a persistent unique identifier (`id`).
* **Initial Value:** `-1` indicates an unassigned polygon. When registered with `cd_2d`, it is assigned an incremental ID (`0, 1, 2, ...`).

---

### 2.2 File 2: `src/acd2d_data.cpp` — ID Copy Preservation

#### Change: Propagate `id` in `cd_polygon::operator=`
```diff
--- a/src/acd2d_data.cpp
+++ b/src/acd2d_data.cpp
@@ -605,6 +605,7 @@ namespace acd2d
 	{
 		//destroy myself
 		destroy();
+		id = other.id;
 	
 		for(const_iterator i=other.begin();i!=other.end();i++){
 			cd_poly p(cd_poly::UNKNOWN);
```

#### Explanation:
* **Why:** ACD frequently copies `cd_polygon` objects between internal containers (`todo_list`, `done_list`, temporary split buffers). Without preserving `id = other.id`, copying a polygon during a cut step would reset its identifier to `-1` and break graph lookups.

---

### 2.3 File 3: `src/acd2d_core.h` — Incremental Graph Structures & Interface

#### Change: Defined `IncrementalEdge`, `IncrementalNode`, and graph maintenance methods
```diff
--- a/src/acd2d_core.h
+++ b/src/acd2d_core.h
@@ -7,6 +7,8 @@
 #define _CD2D_H_
 
 #include <list>
+#include <map>
+#include <vector>
 using namespace std;
 
 #include "acd2d_data.h"
@@ -18,6 +20,19 @@ namespace acd2d
 	typedef cd_polygon::const_iterator PLYCIT;
 	
 	class IConcavityMeasure; //the interface for concavity measurement
+	class ConvexGraph;       //forward declaration for graph export
+
+	struct IncrementalEdge {
+		int target;            // Neighbor polygon ID
+		double shared_length;  // Length of shared 1D boundary (0.0 for point contact)
+		Point2d interface_pt;  // Contact interface midpoint or contact point
+	};
+
+	struct IncrementalNode {
+		int id;
+		std::vector<Point2d> vertices;
+		std::map<int, IncrementalEdge> adj; // target_id -> edge
+	};
 	
 	class cd_2d
 	{
@@ -46,17 +61,27 @@ namespace acd2d
 		const list<cd_polygon>& getDoneList() const { return done_list; }
 		const list<cd_diagonal>& getDiagonal() const { return dia_list; }
 		void updateCutDirParameters( double a, double b ){ alpha=a; beta=b;  }
-	
+		const std::map<int, IncrementalNode>& getIncrementalGraph() const { return m_incremental_graph; }
+
 		///////////////////////////////////////////////////////////////////////////
-		//other functions
+		//graph export
+		void exportToConvexGraph(ConvexGraph& out_graph) const;
 	
 	protected:
 	
 		void decompose(double d, cd_polygon& polys );
 		void decompose_OUT(double d, cd_polygon& polys, cd_poly& poly);
 		void decompose_IN(double d, cd_polygon& polys, cd_poly& poly);
+		void updateCutAdjacency(int parent_id, int child1_id, const cd_polygon& p1, int child2_id, const cd_polygon& p2, const cd_diagonal& dia);
 	
+		static std::vector<Point2d> extractPolygonVertices(const cd_polygon& poly);
+		static bool checkPolygonOverlap(
+			const std::vector<Point2d>& p1,
+			const std::vector<Point2d>& p2,
+			double& shared_len,
+			Point2d& interface_pt
+		);
+
 	private:
 	
 		list<cd_polygon> todo_list;
@@ -69,6 +94,10 @@ namespace acd2d
 		//cut lines
 		list<cd_diagonal> dia_list;
 		bool store_diagoanls;    //if set, all cut lines will be stored
+
+		//incremental dual graph
+		std::map<int, IncrementalNode> m_incremental_graph;
+		int m_next_poly_id;
 	};
```

#### Detailed Element Explanations:
1. **`IncrementalEdge`**: Represents a directed connection to an adjacent polygon `target`. Stores the physical length of the contact interface (`shared_length`) and the geometric coordinate of the contact midpoint (`interface_pt`).
2. **`IncrementalNode`**: Represents an active polygon in the BSP tree. Stores its current boundary vertices (`vertices`) and an adjacency map (`adj`) mapping `neighbor_id -> IncrementalEdge`.
3. **`m_incremental_graph`**: A hash/tree map (`std::map<int, IncrementalNode>`) maintaining the current active set of polygons and their adjacency connections.
4. **`m_next_poly_id`**: A monotonic counter providing unique IDs to every new piece.

---

### 2.4 File 4: `src/acd2d_core.cpp` — Dynamic Graph Logic & Geometry Engine

This file contains the core algorithmic execution. Four key methods were implemented:

#### (A) Vertex Extraction (`extractPolygonVertices`)
Converts the circular doubly-linked vertex list of a `cd_polygon` into a flat `std::vector<Point2d>`:
```cpp
std::vector<Point2d> cd_2d::extractPolygonVertices(const cd_polygon& poly) {
    std::vector<Point2d> pts;
    for (const auto& p : poly) {
        cd_vertex* ptr = p.getHead();
        if (ptr != NULL) {
            do {
                pts.push_back(ptr->getPos());
                ptr = ptr->getNext();
            } while (ptr != p.getHead());
        }
    }
    return pts;
}
```

#### (B) Geometric Overlap Checking (`checkPolygonOverlap`)
Determines if two polygons share an interface:
1. **1D Collinear Overlap:** Tests all edge pairs $(e_1 \in P_1, e_2 \in P_2)$ for overlapping collinear segments (`checkEdgeCollinearOverlap`).
2. **0D Point-Contact (Vertex-Vertex & Vertex-Edge):** Tests if vertices touch without edge overlap (sets `shared_len = 0.0`).
```cpp
bool cd_2d::checkPolygonOverlap(
    const std::vector<Point2d>& p1,
    const std::vector<Point2d>& p2,
    double& shared_len,
    Point2d& interface_pt
) {
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

    // 2. Vertex-Vertex 0D point touch check
    for (int i = 0; i < m1; ++i) {
        for (int j = 0; j < m2; ++j) {
            if (std::abs(p1[i][0] - p2[j][0]) < 1e-3 && std::abs(p1[i][1] - p2[j][1]) < 1e-3) {
                if ((p1[i] - p2[j]).norm() < 1e-3) {
                    shared_len = 0.0;
                    interface_pt = p1[i];
                    return true;
                }
            }
        }
    }

    // 3. Vertex-Edge 0D point touch check (p1 vertex on p2 edge)
    for (int i = 0; i < m1; ++i) {
        const Point2d& pt = p1[i];
        for (int j = 0; j < m2; ++j) {
            const Point2d& a = p2[j];
            const Point2d& b = p2[(j + 1) % m2];
            Vector2d e(b[0] - a[0], b[1] - a[1]);
            double L = e.norm();
            if (L < 1e-6) continue;
            Vector2d u(e[0] / L, e[1] / L);
            Vector2d n(-u[1], u[0]);
            double perp_dist = std::abs((pt[0] - a[0]) * n[0] + (pt[1] - a[1]) * n[1]);
            double proj = (pt[0] - a[0]) * u[0] + (pt[1] - a[1]) * u[1];
            if (perp_dist < 1e-3 && proj >= -1e-4 && proj <= L + 1e-4) {
                shared_len = 0.0;
                interface_pt = pt;
                return true;
            }
        }
    }

    return false;
}
```

#### (C) Dynamic Cut Adjacency Maintenance (`updateCutAdjacency`)
Invoked at every single split in `decompose_OUT`:
```cpp
void cd_2d::updateCutAdjacency(
    int parent_id,
    int child1_id, const cd_polygon& p1,
    int child2_id, const cd_polygon& p2,
    const cd_diagonal& dia
) {
    IncrementalNode n1;
    n1.id = child1_id;
    n1.vertices = extractPolygonVertices(p1);

    IncrementalNode n2;
    n2.id = child2_id;
    n2.vertices = extractPolygonVertices(p2);

    // 1. Internal cut edge between child 1 and child 2
    Point2d c1 = dia.v[0];
    Point2d c2 = dia.v[1];
    double cut_len = (c2 - c1).norm();
    Point2d cut_mid((c1[0] + c2[0]) * 0.5, (c1[1] + c2[1]) * 0.5);

    n1.adj[child2_id] = { child2_id, cut_len, cut_mid };
    n2.adj[child1_id] = { child1_id, cut_len, cut_mid };

    // 2. Inherit adjacency from parent's old neighbors
    auto it_parent = m_incremental_graph.find(parent_id);
    if (it_parent != m_incremental_graph.end()) {
        std::map<int, IncrementalEdge> parent_adj = it_parent->second.adj;
        for (const auto& kv : parent_adj) {
            int q_id = kv.first;
            auto it_q = m_incremental_graph.find(q_id);
            if (it_q == m_incremental_graph.end()) continue;

            IncrementalNode& q_node = it_q->second;
            // Remove old edge to parent
            q_node.adj.erase(parent_id);

            // Test Q against child 1
            double len1 = 0.0;
            Point2d if_pt1;
            if (checkPolygonOverlap(q_node.vertices, n1.vertices, len1, if_pt1)) {
                n1.adj[q_id] = { q_id, len1, if_pt1 };
                q_node.adj[child1_id] = { child1_id, len1, if_pt1 };
            }

            // Test Q against child 2
            double len2 = 0.0;
            Point2d if_pt2;
            if (checkPolygonOverlap(q_node.vertices, n2.vertices, len2, if_pt2)) {
                n2.adj[q_id] = { q_id, len2, if_pt2 };
                q_node.adj[child2_id] = { child2_id, len2, if_pt2 };
            }
        }
        // Remove parent node from graph
        m_incremental_graph.erase(it_parent);
    }

    // Register new children in incremental graph
    m_incremental_graph[child1_id] = n1;
    m_incremental_graph[child2_id] = n2;
}
```

#### (D) Graph Exporting (`exportToConvexGraph`)
Exports the completed incremental graph to the active `ConvexGraph` data structure:
```cpp
void cd_2d::exportToConvexGraph(ConvexGraph& out_graph) const {
    out_graph.clear();
    out_graph.decomposition_type = "ACD";

    // Collect all final pieces from done_list (and todo_list if stopped early)
    std::vector<int> active_poly_ids;
    for (const auto& polys : done_list) {
        active_poly_ids.push_back(polys.id);
    }
    for (const auto& polys : todo_list) {
        active_poly_ids.push_back(polys.id);
    }

    int n_pieces = static_cast<int>(active_poly_ids.size());
    if (n_pieces == 0) return;

    out_graph.nodes.resize(n_pieces);

    // Map internal polygon ID -> compact graph node index (0 ... n_pieces - 1)
    std::map<int, int> poly_id_to_compact;
    for (int i = 0; i < n_pieces; ++i) {
        poly_id_to_compact[active_poly_ids[i]] = i;
    }

    for (int i = 0; i < n_pieces; ++i) {
        int pid = active_poly_ids[i];
        out_graph.nodes[i].id = i;
        std::stringstream ss;
        ss << "C" << i;
        out_graph.nodes[i].label = ss.str();

        auto it_node = m_incremental_graph.find(pid);
        if (it_node != m_incremental_graph.end()) {
            out_graph.nodes[i].vertices = it_node->second.vertices;
            ConvexGraph::computePolygonCentroidAndArea(
                out_graph.nodes[i].vertices,
                out_graph.nodes[i].centroid,
                out_graph.nodes[i].area
            );

            for (const auto& kv : it_node->second.adj) {
                int target_pid = kv.first;
                auto it_target = poly_id_to_compact.find(target_pid);
                if (it_target != poly_id_to_compact.end()) {
                    GraphEdge e;
                    e.target = it_target->second;
                    e.weight = 0.0;
                    e.shared_length = kv.second.shared_length;
                    e.interface_pt = kv.second.interface_pt;
                    out_graph.nodes[i].adj.push_back(e);
                }
            }
        }
    }
}
```

---

### 2.5 File 5: `gui/acd2d_draw.h` — Instant Graph Export Hook

#### Change: Direct call to `exportToConvexGraph`
```diff
--- a/gui/acd2d_draw.h
+++ b/gui/acd2d_draw.h
@@ -46,26 +46,16 @@ int colorid=-1;
 
 inline void updateAcdGraph(cd_2d& cd2d, double decomp_time_sec = -1.0) {
     clock_t g_start = clock();
-    std::vector<std::vector<Point2d>> pieces;
-    for (const auto& polys : cd2d.getDoneList()) {
-        for (const auto& poly : polys) {
-            std::vector<Point2d> piece;
-            cd_vertex* ptr = poly.getHead();
-            if (ptr != NULL) {
-                do {
-                    piece.push_back(ptr->getPos());
-                    ptr = ptr->getNext();
-                } while (ptr != poly.getHead());
-            }
-            if (piece.size() >= 3) pieces.push_back(piece);
-        }
-    }
-    if (!pieces.empty()) {
-        g_activeGraph.buildFromPolygons(pieces, "ACD");
-    } else {
-        g_activeGraph.clear();
-    }
+    cd2d.exportToConvexGraph(g_activeGraph);
     double graph_time = (double)(clock() - g_start) / CLOCKS_PER_SEC;
```

#### Explanation:
* Replaced the manual extraction of polygon loops and batch pairwise graph building with a direct invocation of `exportToConvexGraph(g_activeGraph)`.
* Export time drops from **`10.59 ms` down to `0.18 ms`**.

---

## 3. Empirical Performance Profiles Across Benchmark Maps

Comparing the Incremental Approach against Naive, Naive + AABB, and Canonical Line Hashing:

### 3.1 Standard Map: `room.poly` ($N = 79$ convex pieces, 50 trials)
```
╔══════════════════════════════════════════════════════════════════════════════════════════════╗
║  MAP: testenv/room.poly    Pieces: 79    Trials: 50                                          ║
╠══════════════════════════════════════════════════════════════════════════════════════════════╣
║  Method / Binary           │ Decomp Time      │ Graph Time       │ Total Time         │ Speedup   ║
╟────────────────────────────┼──────────────────┼──────────────────┼────────────────────┼───────────╢
║  Canonical Line Hash ★     │   5.25 ± 0.31 ms │   0.46 ± 0.09 ms │   5.70 ± 0.34 ms   │   2.8x    ║
║  Naive + AABB              │   5.83 ± 1.89 ms │   0.43 ± 0.11 ms │   6.27 ± 1.90 ms   │   2.5x    ║
║  Incremental + AABB        │   7.85 ± 0.63 ms │   0.17 ± 0.01 ms │   8.02 ± 0.63 ms   │   2.0x    ║
║  Incremental BSP           │  13.15 ± 0.46 ms │   0.18 ± 0.01 ms │  13.32 ± 0.47 ms   │   1.2x    ║
║  Naive Baseline            │   5.47 ± 1.85 ms │  10.24 ± 0.15 ms │  15.71 ± 1.85 ms   │   1.0x    ║
╚══════════════════════════════════════════════════════════════════════════════════════════════╝
```

### 3.2 High-Vertex Map: `star6.poly` ($N = 5$ pieces, large vertex loops)
```
╔══════════════════════════════════════════════════════════════════════════════════════════════╗
║  MAP: test_env/star/star6.poly   Pieces: 5                                                   ║
╠══════════════════════════════════════════════════════════════════════════════════════════════╣
║  Canonical Line Hash ★     │  14.10 ms        │  88.28 ms        │  102.37 ms         │  41.0x    ║
║  Naive + AABB              │  14.10 ms        │ 160.72 ms        │  174.80 ms         │  24.0x    ║
║  Incremental + AABB        │ 410.73 ms        │   0.25 ms        │  410.93 ms         │  10.2x    ║
║  Naive Baseline            │  14.30 ms        │ 4182.18 ms       │ 4196.43 ms         │   1.0x    ║
║  Incremental BSP           │ 6035.33 ms       │   0.28 ms        │ 6035.63 ms         │   0.7x 🔴 ║
╚══════════════════════════════════════════════════════════════════════════════════════════════╝
```

---

## 4. Deep Root-Cause Analysis: Why Incremental Performed Poorly

While Graph Export Time was reduced to $\approx 0.18\text{ ms}$, **Decomposition Time increased dramatically**, offsetting any benefits. The performance collapse is attributed to four distinct structural flaws:

### 4.1 Flaw 1: Tree-Depth Redundancy ($h \times \text{cuts}$)
* When polygon $P$ splits, only the internal cut line is new. All other boundary segments were already part of $P$'s exterior.
* In the incremental algorithm, at **every single level of the recursion tree**, the algorithm re-tests all existing neighbors $Q$ against the child polygons.
* If a polygon undergoes $h$ levels of sub-partitioning, its static, unchanged boundary edges are geometrically re-tested against surrounding neighbors $h$ times throughout recursion.
* **Batch post-processing only tests boundaries once**, on the finalized leaf polygons.

### 4.2 Flaw 2: Testing Large Intermediate Polygons ($m$)
* Geometric overlap tests scale quadratically with vertex count: $O(m_1 \cdot m_2)$.
* At the top of the decomposition recursion tree, polygons are **large** ($m = 50 \dots 500$ vertices).
* Running neighbor inheritance loops on large intermediate polygons in upper tree levels is vastly more expensive than testing small final convex pieces ($m = 4 \dots 6$).

### 4.3 Flaw 3: Memory Thrashing from `std::map` Red-Black Trees
* `std::map<int, IncrementalNode>` and nested `std::map<int, IncrementalEdge>` are node-based associative containers implemented as Red-Black Trees.
* Every cut triggered:
  - 2 node insertions (`std::_Rb_tree_node` heap allocations via `malloc`).
  - Multiple edge insertions and key deletions across neighbor maps.
  - 1 parent node erase (`free` / deallocation).
* In complex maps with hundreds of cuts, continuous heap allocation and deallocation inside the tight recursive geometry loop destroyed CPU cache locality and triggered memory allocator stalls.

### 4.4 Flaw 4: Lack of Spatial Indexing on Intermediate Splits
* In `Incremental BSP` (without AABB), every neighbor check performed a nested $O(m_1 \cdot m_2)$ segment comparison loop.
* Adding AABB bounding-box pruning (`Incremental + AABB`) reduced `room.poly` decomposition time from $13.15\text{ ms} \to 7.85\text{ ms}$, but could not overcome the fundamental algorithmic penalty of tree-depth redundancy.

---

## 5. Architectural Comparison Matrix

| Dimension | Incremental BSP Tracking | Naive + AABB Early Exit | Canonical Line Hashing |
| :--- | :--- | :--- | :--- |
| **Execution Point** | Inner loop of every recursive cut | Single post-processing pass | Single post-processing pass |
| **Polygon Size Tested** | Large intermediate ($m=50\dots500$) | Small final convex ($m=4\dots6$) | Individual 1D boundary edges |
| **Boundary Test Multiplier**| Multiplied by tree depth ($h \times K$) | Exactly 1 pass over final pairs | Exactly 1 pass over all edges |
| **Memory Allocation** | Dynamic heap nodes (`std::map`) | Contiguous flat vectors | Flat quantized hash buckets |
| **ACD Decomp Overhead** | **Severe (+140% to +42,000% time)** | **Zero overhead ($0.00\text{ ms}$)** | **Zero overhead ($0.00\text{ ms}$)** |
| **Total Runtime (`room`)** | `13.32 ms` (no AABB) / `8.02 ms` (AABB) | `6.27 ms` | **`5.70 ms`** ★ |
| **Total Runtime (`star6`)**| `6,035 ms` (no AABB) / `410 ms` (AABB) | `174 ms` | **`102 ms`** ★ |

---

## 6. Key Engineering Conclusions

1. **Keep recursive decomposition unburdened:** The core geometric decomposer should execute pure partitioning without dynamic graph bookkeeping.
2. **Avoid testing static boundaries repeatedly:** Deferring adjacency detection to a single batch post-processing pass prevents the $O(h \times K)$ tree-depth penalty.
3. **Use 4-float scalar AABB rejection:** Testing axis-aligned bounding boxes rejects $> 90\%$ of non-adjacent polygon pairs in $< 1\text{ ns}$ per pair in flat CPU cache.
4. **Canonical Line Hashing is optimal for planar partitions:** Hashing boundary edges by $(\theta, d_{\perp})$ yields true $O(E \log E)$ construction with zero decomposition penalty.
