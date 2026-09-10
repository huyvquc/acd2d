//------------------------------------------------------------------------------
//  Copyright 2007-2012 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#include "acd2d.h"
#include "acd2d_util.h"
#include "acd2d_cut.h"
#include "acd2d_dir.h"
#include "acd2d_graph.h"

#ifdef WIN32
#pragma warning(disable : 4786)
#endif

namespace acd2d
{
	///////////////////////////////////////////////////////////////////////////
	
	cd_2d::cd_2d(bool save_diagonal)
	{
		store_diagoanls=save_diagonal;
		alpha=0; beta=1;
		m_next_poly_id = 0;
	}
	
	cd_2d::~cd_2d()
	{
		destroy();
	}
	
	///////////////////////////////////////////////////////////////////////////
	// polygon functions
	
	void cd_2d::addPolygon(const cd_polygon& poly)
	{
		if(poly.valid())
		{
			cd_polygon mypoly;
			mypoly.id = m_next_poly_id++;
			todo_list.push_back(mypoly);
			todo_list.back().copy(poly);
			todo_list.back().id = mypoly.id;
			todo_list.back().buildDependency();

			IncrementalNode node;
			node.id = mypoly.id;
			node.vertices = extractPolygonVertices(todo_list.back());
			m_incremental_graph[node.id] = node;
		}
		else
			cerr<<"! Error: acd_2d::addPolygon: Not a valid polygon"<<endl;
	}
	
	void cd_2d::destroy()
	{
		typedef list<cd_polygon>::iterator IT;
		for(IT i=todo_list.begin();i!=todo_list.end();i++) i->destroy();
		for(IT i=done_list.begin();i!=done_list.end();i++) i->destroy();
		todo_list.clear();
		done_list.clear();
		m_incremental_graph.clear();
		m_next_poly_id = 0;
	}
	
	///////////////////////////////////////////////////////////////////////////
	void cd_2d::decomposeAll(double d, IConcavityMeasure * measure)
	{
		if( d<1e-20 ) d=1e-20;
		int steps = 0;
		do{
			decompose(d,measure);
			steps++;
			if (steps > 1000) {
				cerr << "! Warning: decomposeAll reached max steps limit (1000)" << endl;
				break;
			}
		}
		while(!todo_list.empty());
		cout << "Decomposition finished! Total convex pieces: " << done_list.size() << endl;
	}
	
	void cd_2d::decompose(double d, IConcavityMeasure * measure)
	{
		list<cd_polygon> ps;
		ps.swap(todo_list);
		list<cd_polygon>::iterator ips=ps.begin();
		m_measure=measure;
		if( m_measure==NULL ) {
			cerr<<"! ERROR: cd_2d::decompose: measure si NULL"<<endl;
			return;
		}
		if( d<1e-20 ) d=1e-20;
	
		for(;ips!=ps.end();ips++){
			cd_polygon& polys=*ips;
			if (polys.empty()) continue;
			decompose(d,polys);
		}
	}
	
	void cd_2d::decompose(double d, cd_polygon& polys)
	{
		if (polys.empty()) return;
		//if there are inner polys, random pick one and find the cut
		cd_poly poly=polys.next();
		if( poly.getType()==cd_poly::PIN ) // hole
			decompose_IN(d,polys,poly);
		else //out most boundary
			decompose_OUT(d,polys,findOutMost(polys));
	}
	
	void cd_2d::decompose_OUT(double d, cd_polygon& polys, cd_poly& poly)
	{
		cd_line cut_l; //cut line
	
		//check if we need to cut it.
		cd_vertex * r=poly.findCW(m_measure).first;
	
		if( r==NULL || !r->isReflex() || r->getConcavity() >= FLT_MAX - 1.0 || r->getConcavity()<=d ){
			done_list.push_back(polys);
			return;
		}
	
		find_a_good_cutline(cut_l,r,alpha,beta);
	
		//cut into two polys
		pair<cd_polygon,cd_polygon> sub_polys;
		cd_diagonal dia=cutPolys(sub_polys,polys.front(),cut_l);
	
		if (sub_polys.first.empty() && sub_polys.second.empty()) {
			done_list.push_back(polys);
			return;
		}

		int parent_id = polys.id;
		int child1_id = m_next_poly_id++;
		int child2_id = m_next_poly_id++;
		sub_polys.first.id = child1_id;
		sub_polys.second.id = child2_id;

		// Perform incremental graph maintenance for the cut
		updateCutAdjacency(parent_id, child1_id, sub_polys.first, child2_id, sub_polys.second, dia);

		//add into to do
		if (!sub_polys.first.empty()) todo_list.push_back(sub_polys.first);
		if (!sub_polys.second.empty()) todo_list.push_back(sub_polys.second);
	
		//store cut line
		if(store_diagoanls) dia_list.push_back(dia);
	}
	
	void cd_2d::decompose_IN(double d, cd_polygon& polys, cd_poly& poly)
	{
		//find the out most boundary
		cd_poly& out=findOutMost(polys);
	
		//find concavity witness
		cd_vertex * r=poly.getCW().first;
	
		//find which cw is better
		cd_line cut_l;
		find_a_good_cutline_for_hole(cut_l,r,out);
		cd_diagonal dia=mergeHole(out,poly,cut_l);

		// Update the vertices of polys in m_incremental_graph (since outmost changed)
		if (m_incremental_graph.find(polys.id) != m_incremental_graph.end()) {
			m_incremental_graph[polys.id].vertices = extractPolygonVertices(polys);
		}

		todo_list.push_back(polys);
	
		//store cut line
		if(store_diagoanls) dia_list.push_back(dia);
	}

	std::vector<Point2d> cd_2d::extractPolygonVertices(const cd_polygon& polys)
	{
		std::vector<Point2d> piece;
		for (const auto& poly : polys) {
			cd_vertex* ptr = poly.getHead();
			if (ptr != NULL) {
				do {
					piece.push_back(ptr->getPos());
					ptr = ptr->getNext();
				} while (ptr != poly.getHead());
			}
		}
		return piece;
	}

	bool cd_2d::checkPolygonOverlap(
		const std::vector<Point2d>& p1,
		const std::vector<Point2d>& p2,
		double& shared_len,
		Point2d& interface_pt
	) {
		int m1 = static_cast<int>(p1.size());
		int m2 = static_cast<int>(p2.size());
		if (m1 < 3 || m2 < 3) return false;

		// 1. Collinear shared boundary segments (Shared edge contact L > 0)
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
				if (ConvexGraph::checkEdgeCollinearOverlap(a1, b1, a2, b2, seg_len, seg_mid)) {
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

		// 2. Single point / vertex contact (0D point sharing: vertex-vertex or vertex-edge)
		// Vertex-Vertex check
		for (int i = 0; i < m1; ++i) {
			for (int j = 0; j < m2; ++j) {
				if ((p1[i] - p2[j]).norm() < 1e-3) {
					shared_len = 0.0;
					interface_pt = p1[i];
					return true;
				}
			}
		}

		// Vertex-Edge check: p1 vertex on p2 edge
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

		// Vertex-Edge check: p2 vertex on p1 edge
		for (int j = 0; j < m2; ++j) {
			const Point2d& pt = p2[j];
			for (int i = 0; i < m1; ++i) {
				const Point2d& a = p1[i];
				const Point2d& b = p1[(i + 1) % m1];
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

	void cd_2d::exportToConvexGraph(ConvexGraph& out_graph) const
	{
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

}//namespace acd2d//namespace acd2d