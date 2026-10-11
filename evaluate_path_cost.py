#!/usr/bin/env python3
"""
Convex Decomposition Sequence Cost Evaluator
===========================================
Calculates step-by-step and cumulative traversal costs for an explicit sequence
of convex polygon regions (e.g. C1 -> C8 -> C7 -> C6 -> C4).

Features:
- Validates 1D shared edge adjacency between consecutive regions.
- Computes detailed feature breakdown: d_trav, d_shape, d_corridor, d_distance.
- Linear cost weights strictly loaded from weights.json: a*d_trav + b*d_shape + c*d_corridor + d*d_distance.
- Clean ASCII reporting table and optional rich Matplotlib visualization.

Usage Examples:
  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json C1 C8 C7 C6 C4
  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json "C1, C8, C7, C6, C4"
  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json "C1 -> C8 -> C7 -> C6 -> C4"
  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json C1 C8 C7 C6 C4 --theme dark --output custom_path.png
"""

import argparse
import json
import os
import re
import sys
from typing import Any, Dict, List, Optional, Tuple

try:
    from dijkstra_path import DecompositionGraph, compute_edge_cost, visualize_decomposition_and_path, load_weights_config
except ImportError:
    # If imported from another directory, add script directory to sys.path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    from dijkstra_path import DecompositionGraph, compute_edge_cost, visualize_decomposition_and_path, load_weights_config


# ==============================================================================
# Path String Parser
# ==============================================================================

def parse_region_sequence(raw_tokens: List[str]) -> List[str]:
    """
    Parses region tokens from CLI args (space-separated, comma-separated, or arrow-separated).
    Examples:
      ['C1', 'C8', 'C7'] -> ['C1', 'C8', 'C7']
      ['C1, C8, C7']     -> ['C1', 'C8', 'C7']
      ['C1 -> C8 -> C7'] -> ['C1', 'C8', 'C7']
    """
    cleaned: List[str] = []
    for token in raw_tokens:
        # Split by comma, arrow (-> or ➔), or whitespace
        parts = re.split(r'[,➔\->\s]+', token.strip())
        for p in parts:
            p_str = p.strip()
            if p_str:
                cleaned.append(p_str)
    return cleaned


# ==============================================================================
# Path Evaluator
# ==============================================================================

def evaluate_polygon_path(
    graph: DecompositionGraph,
    regions: List[str],
    weights: Optional[Tuple[float, float, float, float]] = None,
    metric: str = "cost"
) -> Dict[str, Any]:
    """
    Evaluates costs for a sequential list of regions.
    """
    if len(regions) < 1:
        raise ValueError("At least 1 region must be provided.")

    resolved_nodes = []
    for r in regions:
        nid, ndata = graph.resolve_node(r)
        resolved_nodes.append((nid, ndata))

    if len(resolved_nodes) == 1:
        nid, ndata = resolved_nodes[0]
        lbl = ndata.get("label", f"C{nid}")
        return {
            "valid": True,
            "start_id": nid,
            "goal_id": nid,
            "start_label": lbl,
            "goal_label": lbl,
            "metric": metric,
            "total_cost": 0.0,
            "path_node_ids": [nid],
            "path_node_labels": [lbl],
            "path_nodes": [ndata],
            "path_edges": [],
            "hop_count": 0,
            "errors": []
        }

    path_edges = []
    errors = []
    cumulative = 0.0
    total_cost = 0.0
    all_connected = True

    for i in range(len(resolved_nodes) - 1):
        u_id, u_node = resolved_nodes[i]
        v_id, v_node = resolved_nodes[i + 1]
        u_lbl = u_node.get("label", f"C{u_id}")
        v_lbl = v_node.get("label", f"C{v_id}")

        if u_id == v_id:
            # Zero-cost loop step
            step_dict = {
                "from": u_id,
                "to": v_id,
                "step_cost": 0.0,
                "cumulative_cost": cumulative,
                "d_trav": 0.0,
                "d_shape": 0.0,
                "d_corridor": 0.0,
                "d_distance": 0.0,
                "shared_length": 0.0,
                "interface_pt": u_node.get("centroid", [0.0, 0.0]),
                "is_adjacent": True
            }
            path_edges.append(step_dict)
            continue

        # Find direct edge u -> v in graph
        edge_data = None
        for neigh, edata in graph.adj.get(u_id, []):
            if neigh == v_id:
                edge_data = edata
                break

        if edge_data is None:
            # Try reverse edge if symmetric
            for neigh, edata in graph.adj.get(v_id, []):
                if neigh == u_id:
                    edge_data = edata
                    break

        if edge_data is None:
            all_connected = False
            available_neighbors = [
                graph.nodes_by_id[neigh].get("label", f"C{neigh}")
                for neigh, _ in graph.adj.get(u_id, [])
            ]
            err_msg = (
                f"Step {i+1}: Polygon {u_lbl} (ID: {u_id}) and {v_lbl} (ID: {v_id}) "
                f"do NOT share an edge (Not Adjacent). "
                f"Direct neighbors of {u_lbl} are: [{', '.join(available_neighbors)}]"
            )
            errors.append(err_msg)

            # Mark step as invalid
            step_dict = {
                "from": u_id,
                "to": v_id,
                "step_cost": float("inf"),
                "cumulative_cost": float("inf"),
                "d_trav": None,
                "d_shape": None,
                "d_corridor": None,
                "d_distance": None,
                "shared_length": 0.0,
                "interface_pt": None,
                "is_adjacent": False
            }
            path_edges.append(step_dict)
        else:
            step_cost = compute_edge_cost(u_node, v_node, edge_data, metric=metric, weights=weights)
            cumulative += step_cost
            step_dict = {
                "from": u_id,
                "to": v_id,
                "step_cost": step_cost,
                "cumulative_cost": cumulative,
                "d_trav": edge_data.get("d_trav", 0.0),
                "d_shape": edge_data.get("d_shape", 0.0),
                "d_corridor": edge_data.get("d_corridor", 0.0),
                "d_distance": edge_data.get("d_distance", 0.0),
                "shared_length": edge_data.get("shared_length", 0.0),
                "interface_pt": edge_data.get("interface_pt", None),
                "is_adjacent": True
            }
            path_edges.append(step_dict)

    start_id, start_node = resolved_nodes[0]
    goal_id, goal_node = resolved_nodes[-1]
    start_lbl = start_node.get("label", f"C{start_id}")
    goal_lbl = goal_node.get("label", f"C{goal_id}")

    return {
        "valid": all_connected,
        "start_id": start_id,
        "goal_id": goal_id,
        "start_label": start_lbl,
        "goal_label": goal_lbl,
        "metric": metric,
        "total_cost": cumulative if all_connected else float("inf"),
        "path_node_ids": [nid for nid, _ in resolved_nodes],
        "path_node_labels": [ndata.get("label", f"C{nid}") for nid, ndata in resolved_nodes],
        "path_nodes": [ndata for _, ndata in resolved_nodes],
        "path_edges": path_edges,
        "hop_count": len(path_edges),
        "errors": errors
    }


# ==============================================================================
# Console Reporting
# ==============================================================================

def print_evaluation_report(graph: DecompositionGraph, result: Dict[str, Any], detailed: bool = True):
    """Prints a structured ASCII report."""
    print("=" * 78)
    print(" CONVEX DECOMPOSITION SEQUENCE COST EVALUATOR")
    print("=" * 78)
    print(f" Graph File      : {graph.filepath}")
    print(f" Decomposition   : {graph.decomposition_type}")
    print(f" Cost Metric     : {result['metric']}")
    print(f" Input Sequence  : {' ➔ '.join(result['path_node_labels'])}")
    print(f" Total Polygons  : {len(result['path_node_labels'])}")
    print(f" Total Hops      : {result['hop_count']}")
    print("-" * 78)

    if not result["valid"]:
        print(" [!] WARNING: Path contains non-adjacent transitions!")
        for err in result["errors"]:
            print(f"     • {err}")
        print("-" * 78)
    else:
        print(" [✓] Path Connectivity: Continuous (All adjacent transitions share a 1D boundary)")
        print(f" [★] TOTAL PATH COST  : {result['total_cost']:.4f}")
        print("-" * 78)

    if result["path_edges"]:
        if detailed:
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

                if not e.get("is_adjacent", True):
                    print(f" {idx:<5} | {u_lbl:<6} ➔ {v_lbl:<6} | {'DISCONNECTED (No shared edge)':<47}")
                    continue

                dt = f"{e['d_trav']:.3f}" if e['d_trav'] is not None else "-"
                ds = f"{e['d_shape']:.3f}" if e['d_shape'] is not None else "-"
                dc = f"{e['d_corridor']:.3f}" if e['d_corridor'] is not None else "-"
                dd = f"{e['d_distance']:.3f}" if e['d_distance'] is not None else "-"
                sc = f"{e['step_cost']:.4f}"
                cc = f"{e['cumulative_cost']:.4f}"
                print(f" {idx:<5} | {u_lbl:<6} ➔ {v_lbl:<6} | {dt:<7} | {ds:<7} | {dc:<7} | {dd:<7} | {sc:<10} | {cc:<10}")
            print(" " + "-" * 74)
        else:
            print(f" {'Step':<5} | {'From':<8} ➔ {'To':<8} | {'Step Cost':<12} | {'Cumulative Cost':<16}")
            print(" " + "-" * 56)
            for idx, e in enumerate(result["path_edges"], start=1):
                u_lbl = graph.nodes_by_id[e["from"]].get("label", f"C{e['from']}")
                v_lbl = graph.nodes_by_id[e["to"]].get("label", f"C{e['to']}")
                sc = f"{e['step_cost']:.4f}" if e.get("is_adjacent", True) else "INF (No Edge)"
                cc = f"{e['cumulative_cost']:.4f}" if e.get("is_adjacent", True) else "INF"
                print(f" {idx:<5} | {u_lbl:<8} ➔ {v_lbl:<8} | {sc:<12} | {cc:<16}")
            print(" " + "-" * 56)

    print("=" * 78)


# ==============================================================================
# CLI Entry Point
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Calculates step-by-step and total cost for a user-specified sequence of convex polygon regions."
    )
    parser.add_argument("graph_json", help="Path to decomposition graph JSON file (e.g., polygons/hole.poly-ACD.graph.json)")
    parser.add_argument("regions", nargs="*", help="Sequence of polygon labels/IDs (e.g. C1 C8 C7 C6 C4 or 'C1,C8,C7,C6,C4')")

    parser.add_argument("-m", "--metric", type=str, default="cost",
                        choices=["cost", "weight", "d_distance", "d_trav", "d_shape", "d_corridor", "euclidean", "hop"],
                        help="Metric to evaluate (default: cost)")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Path to save visualization plot (e.g., evaluated_path.png)")
    parser.add_argument("-t", "--theme", type=str, default="dark",
                        choices=["dark", "light"],
                        help="Color theme for visualization (default: dark)")
    parser.add_argument("--show", action="store_true",
                        help="Display interactive matplotlib window")
    parser.add_argument("--show-edges", action="store_true",
                        help="Draw all graph connectivity edges in visualization")
    parser.add_argument("--simple-table", action="store_true",
                        help="Display concise step cost table without component breakdown")
    parser.add_argument("--file", type=str, default=None,
                        help="Read region sequence from a text file")

    args = parser.parse_args()

    # Read regions
    raw_tokens = list(args.regions)
    if args.file:
        if not os.path.exists(args.file):
            sys.exit(f"Error: sequence file not found '{args.file}'")
        with open(args.file, "r", encoding="utf-8") as fp:
            raw_tokens.extend(fp.read().split())

    regions = parse_region_sequence(raw_tokens)
    if not regions:
        sys.exit("Error: No region sequence provided. Example usage:\n  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json C1 C8 C7 C6 C4")

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

    # Evaluate sequence
    try:
        result = evaluate_polygon_path(
            graph=graph,
            regions=regions,
            weights=cost_weights,
            metric=args.metric
        )
        result["metric"] = metric_display
    except Exception as err:
        sys.exit(f"Path Evaluation Error: {err}")

    # Print report
    print_evaluation_report(graph, result, detailed=not args.simple_table)

    # Visualization
    if args.output or args.show:
        output_path = args.output
        if not output_path and not args.show:
            output_path = "evaluated_path.png"

        visualize_decomposition_and_path(
            graph=graph,
            result=result,
            output_file=output_path,
            show=args.show,
            theme=args.theme,
            show_all_edges=args.show_edges,
            title=f"Custom Path Evaluation: {' ➔ '.join(result['path_node_labels'])}"
        )
        if output_path:
            print(f"\n[✓] Path visualization saved to: {os.path.abspath(output_path)}")


if __name__ == "__main__":
    main()
