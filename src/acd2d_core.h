//------------------------------------------------------------------------------
//  Copyright 2007-2012 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _CD2D_H_
#define _CD2D_H_

#include <list>
#include <map>
#include <vector>
using namespace std;

#include "acd2d_data.h"

namespace acd2d
{

	typedef cd_polygon::iterator PLYIT;
	typedef cd_polygon::const_iterator PLYCIT;
	
	class IConcavityMeasure; //the interface for concavity measurement
	class ConvexGraph;       //forward declaration for graph export

	struct IncrementalEdge {
		int target;            // Neighbor polygon ID
		double shared_length;  // Length of shared 1D boundary (0.0 for 0D point contact)
		Point2d interface_pt;  // Contact interface midpoint or contact point
	};

	struct IncrementalNode {
		int id;
		std::vector<Point2d> vertices;
		std::map<int, IncrementalEdge> adj; // target_id -> edge
	};
	
	class cd_2d
	{
	public:
	
		///////////////////////////////////////////////////////////////////////////
		// constructors
	
		//if store_cut_line is ture, cutlines will be stored.
		cd_2d(bool save_diagonal=false); 
		~cd_2d();
		
		///////////////////////////////////////////////////////////////////////////
		//polygon functions
		void addPolygon(const cd_polygon& poly);
		void destroy(); //remove all polygons from acd	
		
		///////////////////////////////////////////////////////////////////////////
		// Do decomposition once/All
		void decomposeAll(double d, IConcavityMeasure * measure);
		void decompose(double d, IConcavityMeasure * measure);
	
		///////////////////////////////////////////////////////////////////////////
		//access functions
		const list<cd_polygon>& getTodoList() const { return todo_list; }
		const list<cd_polygon>& getDoneList() const { return done_list; }
		const list<cd_diagonal>& getDiagonal() const { return dia_list; }
		void updateCutDirParameters( double a, double b ){ alpha=a; beta=b;  }
		const std::map<int, IncrementalNode>& getIncrementalGraph() const { return m_incremental_graph; }

		///////////////////////////////////////////////////////////////////////////
		//graph export
		void exportToConvexGraph(ConvexGraph& out_graph) const;
	
	protected:
	
		void decompose(double d, cd_polygon& polys );
		void decompose_OUT(double d, cd_polygon& polys, cd_poly& poly);
		void decompose_IN(double d, cd_polygon& polys, cd_poly& poly);
		void updateCutAdjacency(int parent_id, int child1_id, const cd_polygon& p1, int child2_id, const cd_polygon& p2, const cd_diagonal& dia);
	
		static std::vector<Point2d> extractPolygonVertices(const cd_polygon& poly);
		static bool checkPolygonOverlap(
			const std::vector<Point2d>& p1,
			const std::vector<Point2d>& p2,
			double& shared_len,
			Point2d& interface_pt
		);

	private:
	
		list<cd_polygon> todo_list;
		list<cd_polygon> done_list;
	
		IConcavityMeasure * m_measure;
	
		double alpha, beta; // for selecting cutting direction
	
		//cut lines
		list<cd_diagonal> dia_list;
		bool store_diagoanls;    //if set, all cut lines will be stored

		//incremental dual graph
		std::map<int, IncrementalNode> m_incremental_graph;
		int m_next_poly_id;
	};

}

#endif //_CD2D_H_

