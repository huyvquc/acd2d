#!/usr/bin/env python3
"""
Dijkstra Shortest Path Finder & Visualizer for Convex Decomposition Graphs (ACD2D)
=================================================================================

Reads an ACD2D convex decomposition JSON graph file, takes two polygon labels
(start and goal), executes Dijkstra's algorithm to compute the minimum-cost path, and generates
a high-contrast visualization of the original polygon mesh and the path taken.

Usage:
  python3 dijkstra_path.py <graph_json> <start_label> <goal_label> [-o output.png]

Examples:
  python3 dijkstra_path.py polygons/hole.poly-ACD.graph.json C1 C6
  python3 dijkstra_path.py polygons/room.poly-ACD.graph.json C0 C20 -o room_path.png
"""

import argparse
import colorsys
import heapq
import json
import os
import sys
import time
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
# If running in non-GUI / headless environment, default to Agg when needed
try:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPoly, FancyArrowPatch
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

        # Index vertices by ID
        self.nodes_by_id: Dict[int, Dict[str, Any]] = {}
        for v in self.vertices_data:
            vid = int(v["id"])
            v.setdefault("label", f"C{vid}")
            self.nodes_by_id[vid] = v

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


# ==============================================================================
# Dijkstra Shortest Path Algorithm
# ==============================================================================

def compute_edge_cost(
    edge: Dict[str, Any],
    weights: Optional[Tuple[float, float, float, float]] = None
) -> float:
    """Calculates edge traversal cost using linear weights from weights.json."""
    if weights is not None:
        w_trav, w_shape, w_corr, w_dist = weights
    else:
        cfg = load_weights_config()
        w_trav, w_shape, w_corr, w_dist = cfg["w_trav"], cfg["w_shape"], cfg["w_corr"], cfg["w_dist"]

    d_t = float(edge.get("d_trav", 0.0))
    d_s = float(edge.get("d_shape", 0.0))
    d_c = float(edge.get("d_corridor", 0.0))
    d_d = float(edge.get("d_distance", 0.0))
    return w_trav * d_t + w_shape * d_s + w_corr * d_c + w_dist * d_d


def dijkstra_shortest_path(
    graph: DecompositionGraph,
    start_label: str,
    goal_label: str,
    weights: Optional[Tuple[float, float, float, float]] = None
) -> Dict[str, Any]:

    def parse_cx(label: str) -> Tuple[int, Dict[str, Any]]:
        s = str(label).strip()
        if not ((s.startswith("C") or s.startswith("c")) and s[1:].isdigit()):
            raise ValueError(f"Invalid polygon label '{label}'. Must be in 'CX' format (e.g., C0, C1).")
        nid = int(s[1:])
        if nid not in graph.nodes_by_id:
            raise ValueError(f"Polygon '{label}' not found in graph.")
        return nid, graph.nodes_by_id[nid]

    start_id, start_node = parse_cx(start_label)
    goal_id, goal_node = parse_cx(goal_label)

    t0 = time.perf_counter()

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

        for v, edge_data in graph.adj.get(u, []):
            if v not in graph.nodes_by_id:
                continue
            step_cost = compute_edge_cost(edge_data, weights=weights)

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
    title: Optional[str] = None,
) -> plt.Figure:
    """
    Renders an aesthetically superior visualization of the convex decomposition
    and highlights the Dijkstra shortest path taken with rich styling.
    """
    # Theme colors (Dark mode)
    bg_color = "#0B0F19"
    panel_bg = "#151C2C"
    text_color = "#E2E8F0"
    text_muted = "#94A3B8"
    grid_color = "#1E293B"
    base_border_color = "#334155"

    # Path colors
    start_color = "#10B981"   # Emerald Green
    goal_color = "#EF4444"    # Crimson Red

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
        # Subtle tinted dark tone for non-path polygons
        face_col = (r * 0.5 + 0.1, g * 0.5 + 0.1, b * 0.5 + 0.1, 0.40)
        edge_col = (r * 0.8, g * 0.8, b * 0.8, 0.60)

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
            border_c = "#FDE047"
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
            color="#FFFFFF",
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
                color="#F59E0B",
                linewidth=5.0,
                alpha=0.35,
                zorder=6
            )

            # Draw sharp arrow path
            arrow = FancyArrowPatch(
                (p1[0], p1[1]), (p2[0], p2[1]),
                arrowstyle="-|>",
                mutation_scale=16,
                color="#FFFFFF",
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
                color="#FDE047",
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

    # Title
    main_title = title or f"Dijkstra Shortest Path: {result['start_label']} ➔ {result['goal_label']}"
    plt.suptitle(main_title, fontsize=14, fontweight="bold", color=text_color, y=0.96)

    plt.tight_layout(rect=[0.02, 0.02, 0.98, 0.94])

    if output_file:
        out_dir = os.path.dirname(output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        plt.savefig(output_file, dpi=200, facecolor=fig.get_facecolor(), edgecolor="none")
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
    """Outputs a concise summary with path sequence and step breakdown."""
    if not result.get("found"):
        print(f"No valid path exists from {result['start_label']} to {result['goal_label']}.")
        return

    print(f"Path Sequence: {' ➔ '.join(result['path_node_labels'])}")
    print(f"Total Cost   : {result['total_cost']:.4f}\n")

    if result.get("path_edges"):
        header = (
            f" {'Step':<5} | {'From':<6} ➔ {'To':<6} | "
            f"{'d_trav':<7} | {'d_shape':<7} | {'d_corr':<7} | {'d_dist':<7} | "
            f"{'Step Cost':<10} | {'Cumulative':<10}"
        )
        print(header)
        print("-" * 75)
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
        print("-" * 75)


# ==============================================================================
# CLI Entry Point
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Finds and visualizes the shortest path between 2 convex polygons in a decomposition graph using Dijkstra's algorithm."
    )
    parser.add_argument("graph_json",
                        help="Path to the JSON graph file (e.g., polygons/hole.poly-ACD.graph.json)")
    parser.add_argument("start",
                        help="Start polygon label or ID (e.g., C1, 1)")
    parser.add_argument("goal",
                        help="Goal polygon label or ID (e.g., C6, 6)")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Path to save the output visualization image (optional)")

    args = parser.parse_args()

    # Exclusively load linear weights from weights.json
    cfg_weights = load_weights_config()
    w_trav = cfg_weights["w_trav"]
    w_shape = cfg_weights["w_shape"]
    w_corr = cfg_weights["w_corr"]
    w_dist = cfg_weights["w_dist"]
    cost_weights = (w_trav, w_shape, w_corr, w_dist)

    # Load graph
    try:
        graph = DecompositionGraph.from_file(args.graph_json)
    except Exception as err:
        sys.exit(f"Error loading graph '{args.graph_json}': {err}")

    # Run Dijkstra
    try:
        result = dijkstra_shortest_path(
            graph=graph,
            start_label=args.start,
            goal_label=args.goal,
            weights=cost_weights
        )
    except Exception as err:
        sys.exit(f"Dijkstra Execution Error: {err}")

    # Output text summary
    print_text_report(graph, result)

    # Visualize
    visualize_decomposition_and_path(
        graph=graph,
        result=result,
        output_file=args.output,
        show=True if not args.output else False,
    )


if __name__ == "__main__":
    main()
