#!/usr/bin/env python3
"""
Dijkstra Shortest Path Finder & Visualizer for Convex Decomposition Graphs (ACD2D)
=================================================================================

Reads an ACD2D / IRIS / VCC convex decomposition JSON graph file, takes two polygon labels
(start and goal), executes Dijkstra's algorithm to compute the minimum-cost path, and generates
a high-contrast visualization of the original polygon mesh and the path taken.

Usage:
  python3 dijkstra_path.py <graph_json> <start_label> <goal_label> [options]

Examples:
  python3 dijkstra_path.py testenv/narrow.poly-ACD.graph.json C0 C1
  python3 dijkstra_path.py testenv/room.poly-ACD.graph.json C0 C20 -o room_path.png
  python3 dijkstra_path.py testenv/narrow.poly-ACD.graph.json 0 15 --metric cost --theme dark
  python3 dijkstra_path.py testenv/narrow.poly-ACD.graph.json --list-labels
"""

import argparse
import colorsys
import heapq
import json
import math
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
# If running in non-GUI / headless environment, default to Agg when needed
try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly, FancyArrowPatch, Circle
    from matplotlib.collections import PatchCollection
except ImportError as e:
    sys.exit(f"Error: matplotlib is required. Please install it via `pip install matplotlib`.\n{e}")


# Default linear cost weights: a*d_trav + b*d_shape + c*d_corridor + d*d_distance
def load_weights_config() -> Dict[str, float]:
    """
    Exclusively loads cost weights from 'weights.json'.
    """
    weights_path = "weights.json"
    if not os.path.exists(weights_path):
        script_weights = os.path.join(os.path.dirname(os.path.abspath(__file__)), "weights.json")
        if os.path.exists(script_weights):
            weights_path = script_weights

    if not os.path.exists(weights_path):
        raise FileNotFoundError(
            f"Weights configuration file '{weights_path}' not found! "
            f"Please create 'weights.json' with 'w_trav', 'w_shape', 'w_corr', and 'w_dist'."
        )

    with open(weights_path, "r", encoding="utf-8") as fp:
        data = json.load(fp)

    required = ["w_trav", "w_shape", "w_corr", "w_dist"]
    missing = [k for k in required if k not in data]
    if missing:
        raise KeyError(f"weights.json missing required keys: {missing}")

    return {
        "w_trav": float(data["w_trav"]),
        "w_shape": float(data["w_shape"]),
        "w_corr": float(data["w_corr"]),
        "w_dist": float(data["w_dist"]),
    }


# ==============================================================================
# Color Palettes & Helpers (Matches ACD2D C++ Color Generator)
# ==============================================================================

PALETTE_48 = [
    (0.92, 0.18, 0.22),  # 0: Crimson Red
    (0.12, 0.72, 0.38),  # 1: Emerald Green
    (0.18, 0.45, 0.95),  # 2: Royal Blue
    (0.98, 0.52, 0.08),  # 3: Vivid Orange
    (0.68, 0.22, 0.88),  # 4: Rich Purple
    (0.05, 0.82, 0.86),  # 5: Bright Cyan Turquoise
    (0.96, 0.22, 0.60),  # 6: Hot Pink
    (0.52, 0.82, 0.14),  # 7: Lime Green
    (0.96, 0.78, 0.08),  # 8: Amber Gold
    (0.34, 0.22, 0.88),  # 9: Deep Indigo
    (0.08, 0.68, 0.62),  # 10: Teal
    (0.95, 0.40, 0.35),  # 11: Coral Salmon
    (0.22, 0.86, 0.60),  # 12: Mint Green
    (0.70, 0.12, 0.32),  # 13: Wine Burgundy
    (0.10, 0.62, 0.96),  # 14: Sky Blue
    (0.78, 0.86, 0.10),  # 15: Electric Chartreuse
    (0.86, 0.14, 0.78),  # 16: Vivid Magenta
    (0.82, 0.58, 0.14),  # 17: Mustard Ochre
    (0.48, 0.55, 0.90),  # 18: Periwinkle
    (0.42, 0.66, 0.18),  # 19: Olive Drab
    (0.88, 0.34, 0.14),  # 20: Rust Orange
    (0.74, 0.48, 0.92),  # 21: Lavender
    (0.14, 0.54, 0.30),  # 22: Pine Forest Green
    (0.18, 0.92, 0.78),  # 23: Aquamarine
    (0.56, 0.16, 0.16),  # 24: Deep Maroon
    (0.24, 0.52, 0.72),  # 25: Steel Blue
    (1.00, 0.38, 0.05),  # 26: Tangerine
    (0.00, 0.64, 0.44),  # 27: Jade Green
    (0.52, 0.14, 0.58),  # 28: Dark Plum
    (0.88, 0.68, 0.18),  # 29: Goldenrod
    (0.00, 0.52, 0.82),  # 30: Cerulean Blue
    (0.82, 0.10, 0.42),  # 31: Ruby Red
    (0.20, 0.72, 0.10),  # 32: Kelly Green
    (0.62, 0.36, 0.14),  # 33: Bronze Chocolate
    (0.75, 0.20, 0.52),  # 34: Violet Red
    (0.05, 0.46, 0.62),  # 35: Peacock Blue
    (0.92, 0.62, 0.55),  # 36: Dusty Rose
    (0.40, 0.78, 0.65),  # 37: Seafoam
    (0.60, 0.45, 0.78),  # 38: Amethyst
    (0.85, 0.85, 0.25),  # 39: Lemon Yellow
    (0.30, 0.30, 0.75),  # 40: Cobalt Blue
    (0.85, 0.45, 0.65),  # 41: Bubblegum Pink
    (0.25, 0.65, 0.25),  # 42: Clover Green
    (0.90, 0.55, 0.30),  # 43: Apricot
    (0.50, 0.25, 0.65),  # 44: Byzantium
    (0.15, 0.75, 0.85),  # 45: Ice Blue
    (0.75, 0.30, 0.25),  # 46: Terracotta
    (0.60, 0.75, 0.30),  # 47: Willow Green
]


def get_piece_color(index: int) -> Tuple[float, float, float]:
    """Returns a distinct RGB tuple for polygon index (consistent with ACD2D)."""
    if 0 <= index < len(PALETTE_48):
        return PALETTE_48[index]
    # Golden ratio HSV distribution for n >= 48
    h = (index * 0.618033988749895) % 1.0
    s = 0.65 + 0.30 * ((index * 0.38196601125) % 1.0)
    v = 0.80 + 0.18 * ((index * 0.27182818284) % 1.0)
    return colorsys.hsv_to_rgb(h, s, v)


# ==============================================================================
# Graph Loading & Preprocessing
# ==============================================================================

class DecompositionGraph:
    """Represents the polygon decomposition dual graph."""

    def __init__(self, data: Dict[str, Any], filepath: str = ""):
        self.filepath = filepath
        self.decomposition_type = data.get("decomposition_type", "ACD")
        self.vertices_data: List[Dict[str, Any]] = data.get("vertices", [])
        self.edges_data: List[Dict[str, Any]] = data.get("edges", [])

        # Index maps
        self.nodes_by_id: Dict[int, Dict[str, Any]] = {}
        self.nodes_by_label: Dict[str, Dict[str, Any]] = {}
        self.label_to_id: Dict[str, int] = {}

        # Parse vertices
        for v in self.vertices_data:
            vid = int(v["id"])
            lbl = str(v.get("label", f"C{vid}"))
            self.nodes_by_id[vid] = v
            self.nodes_by_label[lbl] = v
            self.nodes_by_label[lbl.upper()] = v
            self.label_to_id[lbl] = vid
            self.label_to_id[lbl.upper()] = vid
            self.label_to_id[str(vid)] = vid

        # Build adjacency list: adj[from_id] = list of (to_id, edge_dict)
        self.adj: Dict[int, List[Tuple[int, Dict[str, Any]]]] = {
            vid: [] for vid in self.nodes_by_id
        }

        for edge in self.edges_data:
            u = int(edge["from"])
            v = int(edge["to"])
            if u in self.adj:
                self.adj[u].append((v, edge))

    @classmethod
    def from_file(cls, filepath: str) -> "DecompositionGraph":
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"Graph file not found: {filepath}")
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(data, filepath=filepath)

    def resolve_node(self, query: Any) -> Tuple[int, Dict[str, Any]]:
        """Resolves a user-provided query (label like 'C0', 'c0', or ID '0' or 0) to (node_id, node_dict)."""
        query_str = str(query).strip()

        # Direct match in label map
        if query_str in self.label_to_id:
            nid = self.label_to_id[query_str]
            return nid, self.nodes_by_id[nid]

        if query_str.upper() in self.label_to_id:
            nid = self.label_to_id[query_str.upper()]
            return nid, self.nodes_by_id[nid]

        # Check if user entered an integer ID directly
        try:
            nid = int(query_str)
            if nid in self.nodes_by_id:
                return nid, self.nodes_by_id[nid]
        except ValueError:
            pass

        # Check if query without 'C' prefix or with 'C' prefix matches
        if query_str.upper().startswith("C"):
            sub = query_str[1:]
            try:
                nid = int(sub)
                if nid in self.nodes_by_id:
                    return nid, self.nodes_by_id[nid]
            except ValueError:
                pass
        else:
            cand = f"C{query_str}"
            if cand in self.label_to_id:
                nid = self.label_to_id[cand]
                return nid, self.nodes_by_id[nid]

        available = [self.nodes_by_id[vid].get("label", f"C{vid}") for vid in sorted(self.nodes_by_id.keys())]
        preview = ", ".join(available[:15]) + ("..." if len(available) > 15 else "")
        raise ValueError(
            f"Polygon label or ID '{query}' not found in graph.\n"
            f"Available polygons ({len(available)} total): [{preview}]"
        )


# ==============================================================================
# Dijkstra Shortest Path Algorithm
# ==============================================================================

def compute_edge_cost(
    u_node: Dict[str, Any],
    v_node: Dict[str, Any],
    edge: Dict[str, Any],
    metric: str = "cost",
    weights: Optional[Tuple[float, float, float, float]] = None
) -> float:
    """Calculates edge traversal cost based on the specified metric."""
    metric = metric.lower().strip()

    if metric in ("cost", "default"):
        if weights is not None:
            w_trav, w_shape, w_corr, w_dist = weights
        else:
            cfg = load_weights_config()
            w_trav, w_shape, w_corr, w_dist = cfg["w_trav"], cfg["w_shape"], cfg["w_corr"], cfg["w_dist"]

        if any(k in edge for k in ("d_trav", "d_shape", "d_corridor", "d_distance")):
            d_t = float(edge.get("d_trav", 0.0))
            d_s = float(edge.get("d_shape", 0.0))
            d_c = float(edge.get("d_corridor", 0.0))
            d_d = float(edge.get("d_distance", 0.0))
            return w_trav * d_t + w_shape * d_s + w_corr * d_c + w_dist * d_d
        if "cost" in edge and edge["cost"] is not None:
            return float(edge["cost"])
        if "weight" in edge and edge["weight"] is not None and edge["weight"] > 0:
            return float(edge["weight"])
        metric = "euclidean"

    if metric in ("weight", "wt"):
        if "weight" in edge and edge["weight"] is not None:
            return float(edge["weight"])
        return float(edge.get("cost", 1.0))

    if metric in ("d_distance", "dist", "distance"):
        if "d_distance" in edge and edge["d_distance"] is not None:
            return float(edge["d_distance"])
        metric = "euclidean"

    if metric in ("d_trav", "trav", "traversability"):
        if "d_trav" in edge and edge["d_trav"] is not None:
            return float(edge["d_trav"])
        return float(edge.get("cost", 1.0))

    if metric in ("d_shape", "shape"):
        if "d_shape" in edge and edge["d_shape"] is not None:
            return float(edge["d_shape"])
        return float(edge.get("cost", 1.0))

    if metric in ("d_corridor", "corridor"):
        if "d_corridor" in edge and edge["d_corridor"] is not None:
            return float(edge["d_corridor"])
        return float(edge.get("cost", 1.0))

    if metric in ("hop", "hops", "unweighted", "step"):
        return 1.0

    if metric in ("euclidean", "geom", "geometry"):
        cu = u_node.get("centroid", [0.0, 0.0])
        cv = v_node.get("centroid", [0.0, 0.0])
        dx = cu[0] - cv[0]
        dy = cu[1] - cv[1]
        return math.hypot(dx, dy)

    # Fallback to edge cost or 1.0
    return float(edge.get("cost", edge.get("weight", 1.0)))


def dijkstra_shortest_path(
    graph: DecompositionGraph,
    start_query: Any,
    goal_query: Any,
    metric: str = "cost",
    undirected: bool = False,
    weights: Optional[Tuple[float, float, float, float]] = None
) -> Dict[str, Any]:
    """
    Executes Dijkstra's algorithm from start to goal.

    Returns a comprehensive result dictionary with path details, costs, and metadata.
    """
    start_id, start_node = graph.resolve_node(start_query)
    goal_id, goal_node = graph.resolve_node(goal_query)

    t0 = time.perf_counter()

    # If undirected traversal requested, also index reverse edges
    adj_map: Dict[int, List[Tuple[int, Dict[str, Any]]]] = {vid: list(edges) for vid, edges in graph.adj.items()}
    if undirected:
        for u, edges in graph.adj.items():
            for v, edata in edges:
                # Add reverse edge if not already present
                if not any(neigh == u for neigh, _ in adj_map[v]):
                    adj_map[v].append((u, edata))

    # Dijkstra data structures
    distances: Dict[int, float] = {vid: float("inf") for vid in graph.nodes_by_id}
    parent: Dict[int, Optional[Tuple[int, Dict[str, Any], float]]] = {vid: None for vid in graph.nodes_by_id}

    distances[start_id] = 0.0
    # Min-heap elements: (current_dist, node_id)
    pq = [(0.0, start_id)]

    visited_count = 0
    found = False

    while pq:
        curr_dist, u = heapq.heappop(pq)

        if curr_dist > distances[u]:
            continue

        visited_count += 1

        if u == goal_id:
            found = True
            break

        u_node_data = graph.nodes_by_id[u]

        for v, edge_data in adj_map.get(u, []):
            if v not in graph.nodes_by_id:
                continue
            v_node_data = graph.nodes_by_id[v]
            step_cost = compute_edge_cost(u_node_data, v_node_data, edge_data, metric=metric, weights=weights)

            new_dist = curr_dist + step_cost
            if new_dist < distances[v]:
                distances[v] = new_dist
                parent[v] = (u, edge_data, step_cost)
                heapq.heappush(pq, (new_dist, v))

    t1 = time.perf_counter()
    elapsed_ms = (t1 - t0) * 1000.0

    if not found and start_id != goal_id:
        return {
            "found": False,
            "start_id": start_id,
            "goal_id": goal_id,
            "start_label": start_node.get("label", f"C{start_id}"),
            "goal_label": goal_node.get("label", f"C{goal_id}"),
            "metric": metric,
            "total_cost": float("inf"),
            "path_node_ids": [],
            "path_node_labels": [],
            "path_nodes": [],
            "path_edges": [],
            "visited_count": visited_count,
            "elapsed_ms": elapsed_ms,
        }

    # Reconstruct path
    path_node_ids = []
    path_edges = []
    curr = goal_id

    if start_id == goal_id:
        path_node_ids = [start_id]
    else:
        while curr is not None and curr != start_id:
            path_node_ids.append(curr)
            prev_info = parent[curr]
            if prev_info is not None:
                p_node, p_edge, step_c = prev_info
                path_edges.append({
                    "from": p_node,
                    "to": curr,
                    "edge_data": p_edge,
                    "step_cost": step_c,
                    "d_trav": p_edge.get("d_trav", 0.0) if p_edge else None,
                    "d_shape": p_edge.get("d_shape", 0.0) if p_edge else None,
                    "d_corridor": p_edge.get("d_corridor", 0.0) if p_edge else None,
                    "d_distance": p_edge.get("d_distance", 0.0) if p_edge else None,
                    "shared_length": p_edge.get("shared_length", 0.0) if p_edge else 0.0,
                    "interface_pt": p_edge.get("interface_pt", None) if p_edge else None,
                })
                curr = p_node
            else:
                break
        path_node_ids.append(start_id)
        path_node_ids.reverse()
        path_edges.reverse()

    path_nodes = [graph.nodes_by_id[nid] for nid in path_node_ids]
    path_node_labels = [n.get("label", f"C{n['id']}") for n in path_nodes]

    # Calculate cumulative costs along path
    cumulative = 0.0
    for e in path_edges:
        cumulative += e["step_cost"]
        e["cumulative_cost"] = cumulative

    return {
        "found": True,
        "start_id": start_id,
        "goal_id": goal_id,
        "start_label": start_node.get("label", f"C{start_id}"),
        "goal_label": goal_node.get("label", f"C{goal_id}"),
        "metric": metric,
        "total_cost": distances[goal_id],
        "path_node_ids": path_node_ids,
        "path_node_labels": path_node_labels,
        "path_nodes": path_nodes,
        "path_edges": path_edges,
        "hop_count": len(path_edges),
        "visited_count": visited_count,
        "elapsed_ms": elapsed_ms,
    }


# ==============================================================================
# Rich Matplotlib Visualization
# ==============================================================================

def visualize_decomposition_and_path(
    graph: DecompositionGraph,
    result: Dict[str, Any],
    output_file: Optional[str] = None,
    show: bool = True,
    theme: str = "dark",
    show_all_edges: bool = False,
    show_labels: bool = True,
    title: Optional[str] = None,
    dpi: int = 200,
) -> plt.Figure:
    """
    Renders an aesthetically superior visualization of the convex decomposition
    and highlights the Dijkstra shortest path taken with rich styling.
    """
    is_dark = (theme.lower() == "dark")

    # Theme colors
    bg_color = "#0B0F19" if is_dark else "#F8FAFC"
    panel_bg = "#151C2C" if is_dark else "#FFFFFF"
    text_color = "#E2E8F0" if is_dark else "#1E293B"
    text_muted = "#94A3B8" if is_dark else "#64748B"
    grid_color = "#1E293B" if is_dark else "#E2E8F0"
    base_border_color = "#334155" if is_dark else "#CBD5E1"
    edge_line_color = "#3B82F6" if is_dark else "#60A5FA"

    # Path colors
    start_color = "#10B981"   # Emerald Green
    goal_color = "#EF4444"    # Crimson Red
    path_fill_color = "#F59E0B"  # Vibrant Amber
    path_line_color = "#FBBF24"  # Golden Amber
    path_border_color = "#FFFFFF" if is_dark else "#0F172A"

    fig, ax = plt.subplots(figsize=(12, 10), facecolor=bg_color)
    ax.set_facecolor(bg_color)

    path_node_ids_set = set(result.get("path_node_ids", []))
    start_id = result.get("start_id")
    goal_id = result.get("goal_id")

    # 1. Draw All Background Polygons
    for v in graph.vertices_data:
        vid = int(v["id"])
        coords = v.get("polygon", [])
        if len(coords) < 3:
            continue

        centroid = v.get("centroid", [0.0, 0.0])
        is_in_path = vid in path_node_ids_set
        is_start = (vid == start_id)
        is_goal = (vid == goal_id)

        # Skip path polygons for now (they'll be drawn in front with high emphasis)
        if is_in_path:
            continue

        # Polygon distinct base color
        r, g, b = get_piece_color(vid)
        if is_dark:
            # Subtle tinted dark tone for non-path polygons
            face_col = (r * 0.5 + 0.1, g * 0.5 + 0.1, b * 0.5 + 0.1, 0.40)
            edge_col = (r * 0.8, g * 0.8, b * 0.8, 0.60)
        else:
            face_col = (r * 0.7 + 0.3, g * 0.7 + 0.3, b * 0.7 + 0.3, 0.45)
            edge_col = (r * 0.6, g * 0.6, b * 0.6, 0.75)

        poly_patch = MplPoly(
            coords,
            closed=True,
            facecolor=face_col,
            edgecolor=edge_col,
            linewidth=1.0,
            linestyle="-",
            zorder=2
        )
        ax.add_patch(poly_patch)

        # Subtle label for background polygons
        if show_labels:
            lbl = v.get("label", f"C{vid}")
            ax.text(
                centroid[0], centroid[1],
                lbl,
                color=text_muted,
                fontsize=7,
                ha="center",
                va="center",
                alpha=1,
                zorder=4,
                fontweight="normal"
            )

    # 2. Optionally draw all graph connectivity edges
    if show_all_edges:
        for u, edges in graph.adj.items():
            cu = graph.nodes_by_id[u].get("centroid", [0, 0])
            for v, _ in edges:
                cv = graph.nodes_by_id[v].get("centroid", [0, 0])
                ax.plot(
                    [cu[0], cv[0]], [cu[1], cv[1]],
                    color=edge_line_color,
                    linestyle="--",
                    linewidth=0.7,
                    alpha=0.25,
                    zorder=3
                )

    # 3. Draw Path Polygons (Foreground Layer with Glow & High Contrast)
    path_len = len(result.get("path_node_ids", []))
    for step_idx, vid in enumerate(result.get("path_node_ids", [])):
        v = graph.nodes_by_id[vid]
        coords = v.get("polygon", [])
        centroid = v.get("centroid", [0.0, 0.0])
        is_start = (vid == start_id)
        is_goal = (vid == goal_id)

        # Dynamic color progression along path
        if is_start:
            fill_c = (*matplotlib.colors.to_rgb(start_color), 0.75)
            border_c = start_color
            border_w = 2.8
        elif is_goal:
            fill_c = (*matplotlib.colors.to_rgb(goal_color), 0.75)
            border_c = goal_color
            border_w = 2.8
        else:
            # Gradient between Cyan/Emerald and Amber
            prog = step_idx / max(1, path_len - 1)
            # Blend from emerald-cyan to golden-amber
            r_c = 0.05 + 0.90 * prog
            g_c = 0.75 + 0.15 * (1.0 - prog)
            b_c = 0.85 * (1.0 - prog)
            fill_c = (r_c, g_c, b_c, 0.65)
            border_c = "#FDE047" if is_dark else "#D97706"
            border_w = 2.2

        path_poly = MplPoly(
            coords,
            closed=True,
            facecolor=fill_c,
            edgecolor=border_c,
            linewidth=border_w,
            linestyle="-",
            zorder=5
        )
        ax.add_patch(path_poly)

        # Highlight Centroid & Step Tag
        lbl = v.get("label", f"C{vid}")
        badge_text = lbl
        if is_start:
            badge_text = f"START\n{lbl}"
        elif is_goal:
            badge_text = f"GOAL\n{lbl}"

        # Label box
        ax.text(
            centroid[0], centroid[1],
            f" {badge_text} ",
            color="#FFFFFF" if is_dark else "#0F172A",
            fontsize=8 if not (is_start or is_goal) else 9,
            fontweight="bold",
            ha="center",
            va="center",
            zorder=9,
            bbox=dict(
                boxstyle="round,pad=0.25",
                facecolor=panel_bg if not (is_start or is_goal) else (start_color if is_start else goal_color),
                edgecolor=border_c,
                linewidth=1.2,
                alpha=0.92
            )
        )

    # 4. Draw Directed Path Trajectory Line connecting Centroids
    if result.get("found") and len(result.get("path_nodes", [])) > 1:
        path_nodes = result["path_nodes"]
        for i in range(len(path_nodes) - 1):
            p1 = path_nodes[i]["centroid"]
            p2 = path_nodes[i + 1]["centroid"]

            # Draw glowing line backing
            ax.plot(
                [p1[0], p2[0]], [p1[1], p2[1]],
                color="#F59E0B" if is_dark else "#D97706",
                linewidth=5.0,
                alpha=0.35,
                zorder=6
            )

            # Draw sharp arrow path
            arrow = FancyArrowPatch(
                (p1[0], p1[1]), (p2[0], p2[1]),
                arrowstyle="-|>",
                mutation_scale=16,
                color="#FFFFFF" if is_dark else "#1E293B",
                linewidth=2.4,
                linestyle="-",
                zorder=7
            )
            ax.add_patch(arrow)

            # Draw step cost bubble along the edge midpoint
            mid_x = 0.5 * (p1[0] + p2[0])
            mid_y = 0.5 * (p1[1] + p2[1])
            edge_info = result["path_edges"][i]
            cost_val = edge_info["step_cost"]

            # Slightly offset step cost text
            ax.text(
                mid_x, mid_y,
                f"c={cost_val:.2f}",
                fontsize=7,
                color="#FDE047" if is_dark else "#92400E",
                fontweight="bold",
                ha="center",
                va="center",
                zorder=10,
                bbox=dict(
                    boxstyle="round,pad=0.15",
                    facecolor=bg_color,
                    edgecolor="#F59E0B",
                    linewidth=0.8,
                    alpha=0.85
                )
            )

    # 5. Styling, Grid, & Bounds
    ax.autoscale()
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", color=grid_color, alpha=0.6, zorder=1)

    for spine in ax.spines.values():
        spine.set_color(base_border_color)
        spine.set_linewidth(1.0)

    ax.tick_params(colors=text_muted, labelsize=8)

    # Title & Metadata
    decomp_type = graph.decomposition_type
    base_filename = os.path.basename(graph.filepath) if graph.filepath else "Decomposition Graph"
    main_title = title or f"Dijkstra Shortest Path: {result['start_label']} ➔ {result['goal_label']}"
    plt.suptitle(main_title, fontsize=14, fontweight="bold", color=text_color, y=0.96)

    # Custom Legend
    # legend_elements = [
    #     plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=start_color, markersize=10, label=f"Start: {result['start_label']}"),
    #     plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=goal_color, markersize=10, label=f"Goal: {result['goal_label']}"),
    #     plt.Line2D([0], [0], marker='s', color='w', markerfacecolor=path_fill_color, markersize=10, label="Path Polygon"),
    #     plt.Line2D([0], [0], color=path_line_color, linewidth=2.5, label="Shortest Trajectory"),
    # ]
    # leg = ax.legend(
    #     handles=legend_elements,
    #     loc="lower right",
    #     facecolor=panel_bg,
    #     edgecolor=base_border_color,
    #     fontsize=8.5,
    #     framealpha=0.92
    # )
    # if leg:
    #     for txt in leg.get_texts():
    #         txt.set_color(text_color)
    # leg.set_zorder(12)

    plt.tight_layout(rect=[0.02, 0.02, 0.98, 0.94])

    if output_file:
        out_dir = os.path.dirname(output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        plt.savefig(output_file, dpi=dpi, facecolor=fig.get_facecolor(), edgecolor="none")
        print(f"\n[✓] Visualization successfully saved to: {os.path.abspath(output_file)}")

    if show:
        try:
            plt.show()
        except Exception as err:
            print(f"[!] Could not launch interactive GUI window ({err}). Image saved if requested.")

    return fig


# ==============================================================================
# Console Reporting
# ==============================================================================

def print_text_report(graph: DecompositionGraph, result: Dict[str, Any]):
    """Outputs a clean, readable ASCII summary to terminal."""
    print("=" * 78)
    print(" DECOMPOSITION GRAPH DIJKSTRA SHORTEST PATH FINDER")
    print("=" * 78)
    print(f" Graph File      : {graph.filepath}")
    print(f" Decomposition   : {graph.decomposition_type}")
    print(f" Total Polygons  : {len(graph.nodes_by_id)}")
    print(f" Directed Edges  : {len(graph.edges_data)}")
    print(f" Cost Metric     : {result['metric']}")
    print("-" * 78)
    print(f" Start Polygon   : {result['start_label']} (ID: {result['start_id']})")
    print(f" Goal Polygon    : {result['goal_label']} (ID: {result['goal_id']})")
    print("-" * 78)

    if not result.get("found"):
        print(f" [✗] RESULT: No valid path exists from {result['start_label']} to {result['goal_label']}.")
        print(f"     Visited {result['visited_count']} vertices before exhausting reachable components.")
        print("=" * 78)
        return

    print(" [✓] RESULT: Shortest Path Found!")
    print(f" Total Path Cost : {result['total_cost']:.4f}")
    print(f" Hops (Edges)    : {result['hop_count']}")
    print(f" Polygons on Path: {len(result['path_node_ids'])}")
    print(f" Solve Time      : {result['elapsed_ms']:.3f} ms")
    print("-" * 78)
    print(" Path Sequence:")
    print("   " + " ➔ ".join(result["path_node_labels"]))
    print("-" * 78)

    if result.get("path_edges"):
        header = (
            f" {'Step':<5} | {'From':<6} ➔ {'To':<6} | "
            f"{'d_trav':<7} | {'d_shape':<7} | {'d_corr':<7} | {'d_dist':<7} | "
            f"{'Step Cost':<10} | {'Cumulative':<10}"
        )
        print(header)
        print(" " + "-" * 74)
        for idx, e in enumerate(result["path_edges"], start=1):
            u_lbl = graph.nodes_by_id[e["from"]].get("label", f"C{e['from']}")
            v_lbl = graph.nodes_by_id[e["to"]].get("label", f"C{e['to']}")
            dt = f"{e['d_trav']:.3f}" if e.get("d_trav") is not None else "-"
            ds = f"{e['d_shape']:.3f}" if e.get("d_shape") is not None else "-"
            dc = f"{e['d_corridor']:.3f}" if e.get("d_corridor") is not None else "-"
            dd = f"{e['d_distance']:.3f}" if e.get("d_distance") is not None else "-"
            sc = f"{e['step_cost']:.4f}"
            cc = f"{e['cumulative_cost']:.4f}"
            print(f" {idx:<5} | {u_lbl:<6} ➔ {v_lbl:<6} | {dt:<7} | {ds:<7} | {dc:<7} | {dd:<7} | {sc:<10} | {cc:<10}")
        print(" " + "-" * 74)

    print("=" * 78)


# ==============================================================================
# CLI Entry Point
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Finds and visualizes the shortest path between 2 convex polygons in a decomposition graph using Dijkstra's algorithm."
    )
    parser.add_argument("graph_json", nargs="?", default="testenv/narrow.poly-ACD.graph.json",
                        help="Path to the JSON graph file (e.g., testenv/narrow.poly-ACD.graph.json)")
    parser.add_argument("start", nargs="?", default="C0",
                        help="Start polygon label or ID (e.g., C0, 0)")
    parser.add_argument("goal", nargs="?", default="C1",
                        help="Goal polygon label or ID (e.g., C1, 15)")

    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Path to save the output visualization image (e.g., path_result.png)")
    parser.add_argument("-m", "--metric", type=str, default="cost",
                        choices=["cost", "weight", "d_distance", "d_trav", "d_shape", "d_corridor", "euclidean", "hop"],
                        help="Edge cost metric to optimize with Dijkstra (default: cost)")
    parser.add_argument("-t", "--theme", type=str, default="dark",
                        choices=["dark", "light"],
                        help="Visualization color theme (dark or light, default: dark)")
    parser.add_argument("--undirected", action="store_true",
                        help="Treat graph edges as bidirectional/undirected")
    parser.add_argument("--show-edges", action="store_true",
                        help="Draw all graph connectivity edges in the visualization")
    parser.add_argument("--no-labels", action="store_true",
                        help="Hide polygon label annotations on the plot")
    parser.add_argument("--no-show", action="store_true",
                        help="Do not display interactive matplotlib window")
    parser.add_argument("--dpi", type=int, default=200,
                        help="Image resolution DPI for saved figure (default: 200)")
    parser.add_argument("--list-labels", action="store_true",
                        help="List all polygon labels in the graph file and exit")

    args = parser.parse_args()

    # Exclusively load linear weights from weights.json
    cfg_weights = load_weights_config()
    w_trav = cfg_weights["w_trav"]
    w_shape = cfg_weights["w_shape"]
    w_corr = cfg_weights["w_corr"]
    w_dist = cfg_weights["w_dist"]
    cost_weights = (w_trav, w_shape, w_corr, w_dist)

    if args.metric == "cost":
        metric_display = f"cost ({w_trav}*d_trav + {w_shape}*d_shape + {w_corr}*d_corr + {w_dist}*d_dist)"
    else:
        metric_display = args.metric

    # Load graph
    try:
        graph = DecompositionGraph.from_file(args.graph_json)
    except Exception as err:
        sys.exit(f"Error loading graph '{args.graph_json}': {err}")

    # Handle --list-labels
    if args.list_labels:
        labels = [graph.nodes_by_id[vid].get("label", f"C{vid}") for vid in sorted(graph.nodes_by_id.keys())]
        print(f"\nGraph: {args.graph_json} ({graph.decomposition_type})")
        print(f"Polygons ({len(labels)} total):")
        for i in range(0, len(labels), 10):
            print("  " + ", ".join(labels[i:i+10]))
        return

    # Run Dijkstra
    try:
        result = dijkstra_shortest_path(
            graph=graph,
            start_query=args.start,
            goal_query=args.goal,
            metric=args.metric,
            undirected=args.undirected,
            weights=cost_weights
        )
        result["metric"] = metric_display
    except Exception as err:
        sys.exit(f"Dijkstra Execution Error: {err}")

    # Output text summary
    print_text_report(graph, result)

    # Output file handling: if output not specified and --no-show is active, auto-save default
    output_path = args.output
    if not output_path and args.no_show:
        output_path = "shortest_path.png"

    # Visualize
    visualize_decomposition_and_path(
        graph=graph,
        result=result,
        output_file=output_path,
        show=not args.no_show,
        theme=args.theme,
        show_all_edges=args.show_edges,
        show_labels=not args.no_labels,
        dpi=args.dpi
    )


if __name__ == "__main__":
    main()
