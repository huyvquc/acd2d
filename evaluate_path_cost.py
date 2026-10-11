#!/usr/bin/env python3
"""
Convex Decomposition Sequence Cost Evaluator
===========================================
Calculates step-by-step and cumulative traversal costs for an explicit sequence
of convex polygon regions (e.g. C1 -> C8 -> C7 -> C6 -> C4).

Usage:
  python3 evaluate_path_cost.py <graph_json> <regions...> [-o output.png]

Examples:
  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json C1 C8 C7 C6 C4
  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json "C1 -> C8 -> C7 -> C6 -> C4" -o path.png
"""

import argparse
import os
import re
import sys
from typing import Any, Dict, List, Optional, Tuple

try:
    from dijkstra_path import DecompositionGraph, compute_edge_cost, visualize_decomposition_and_path, load_weights_config
except ImportError:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    from dijkstra_path import DecompositionGraph, compute_edge_cost, visualize_decomposition_and_path, load_weights_config


def parse_region_sequence(raw_tokens: List[str]) -> List[str]:
    """Parses region tokens from CLI args (space-separated, comma-separated, or arrow-separated)."""
    cleaned: List[str] = []
    for token in raw_tokens:
        parts = re.split(r'[,➔\->\s]+', token.strip())
        for p in parts:
            p_str = p.strip()
            if p_str:
                cleaned.append(p_str)
    return cleaned


def evaluate_polygon_path(
    graph: DecompositionGraph,
    regions: List[str],
    weights: Optional[Tuple[float, float, float, float]] = None
) -> Dict[str, Any]:
    """Evaluates costs for a sequential list of regions."""
    if len(regions) < 1:
        raise ValueError("At least 1 region must be provided.")

    def parse_cx(label: str) -> Tuple[int, Dict[str, Any]]:
        s = str(label).strip()
        if not ((s.startswith("C") or s.startswith("c")) and s[1:].isdigit()):
            raise ValueError(f"Invalid polygon label '{label}'. Must be in 'CX' format (e.g., C0, C1).")
        nid = int(s[1:])
        if nid not in graph.nodes_by_id:
            raise ValueError(f"Polygon '{label}' not found in graph.")
        return nid, graph.nodes_by_id[nid]

    resolved_nodes = [parse_cx(r) for r in regions]

    if len(resolved_nodes) == 1:
        nid, ndata = resolved_nodes[0]
        lbl = ndata.get("label", f"C{nid}")
        return {
            "valid": True,
            "start_id": nid,
            "goal_id": nid,
            "start_label": lbl,
            "goal_label": lbl,
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
    all_connected = True

    for i in range(len(resolved_nodes) - 1):
        u_id, u_node = resolved_nodes[i]
        v_id, v_node = resolved_nodes[i + 1]
        u_lbl = u_node.get("label", f"C{u_id}")
        v_lbl = v_node.get("label", f"C{v_id}")

        if u_id == v_id:
            step_dict = {
                "from": u_id,
                "to": v_id,
                "step_cost": 0.0,
                "cumulative_cost": cumulative,
                "d_trav": 0.0,
                "d_shape": 0.0,
                "d_corridor": 0.0,
                "d_distance": 0.0,
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

            step_dict = {
                "from": u_id,
                "to": v_id,
                "step_cost": float("inf"),
                "cumulative_cost": float("inf"),
                "d_trav": None,
                "d_shape": None,
                "d_corridor": None,
                "d_distance": None,
                "is_adjacent": False
            }
            path_edges.append(step_dict)
        else:
            step_cost = compute_edge_cost(edge_data, weights=weights)
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
        "total_cost": cumulative if all_connected else float("inf"),
        "path_node_ids": [nid for nid, _ in resolved_nodes],
        "path_node_labels": [ndata.get("label", f"C{nid}") for nid, ndata in resolved_nodes],
        "path_nodes": [ndata for _, ndata in resolved_nodes],
        "path_edges": path_edges,
        "hop_count": len(path_edges),
        "errors": errors
    }


def print_evaluation_report(graph: DecompositionGraph, result: Dict[str, Any]):
    """Outputs a concise summary with path sequence and step breakdown."""
    if not result["valid"]:
        print(" [!] WARNING: Path contains non-adjacent transitions!")
        for err in result["errors"]:
            print(f"     • {err}")
        print()

    print(f"Path Sequence: {' ➔ '.join(result['path_node_labels'])}")
    print(f"Total Cost   : {result['total_cost']:.4f}\n")

    if result["path_edges"]:
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
        print("-" * 75)


def main():
    parser = argparse.ArgumentParser(
        description="Calculates step-by-step and total cost for a user-specified sequence of convex polygon regions."
    )
    parser.add_argument("graph_json", help="Path to decomposition graph JSON file (e.g., polygons/hole.poly-ACD.graph.json)")
    parser.add_argument("regions", nargs="+", help="Sequence of polygon labels (e.g. C1 C8 C7 C6 C4 or 'C1 -> C8 -> C7')")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Path to save visualization plot (optional)")

    args = parser.parse_args()

    regions = parse_region_sequence(args.regions)
    if not regions:
        sys.exit("Error: No region sequence provided. Example usage:\n  python3 evaluate_path_cost.py polygons/hole.poly-ACD.graph.json C1 C8 C7 C6 C4")

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

    # Evaluate sequence
    try:
        result = evaluate_polygon_path(
            graph=graph,
            regions=regions,
            weights=cost_weights
        )
    except Exception as err:
        sys.exit(f"Path Evaluation Error: {err}")

    # Print report
    print_evaluation_report(graph, result)

    # Visualization
    visualize_decomposition_and_path(
        graph=graph,
        result=result,
        output_file=args.output,
        show=True if not args.output else False,
        title=f"Path Traversal: {result['start_label']} ➔ {result['goal_label']}",
    )


if __name__ == "__main__":
    main()
