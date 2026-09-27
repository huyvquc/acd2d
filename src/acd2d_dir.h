//------------------------------------------------------------------------------
//  Copyright 2007-2012 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _CD2D_CUT_DIR_H_
#define _CD2D_CUT_DIR_H_

#include <vector>
#include "acd2d_data.h"
#include "acd2d_edge_visibility.h"
 
namespace acd2d
{
	
	///////////////////////////////////////////////////////////////////////////////
	// For genereal
	inline
	void setupCutLine(cd_vertex * r,cd_vertex * r2,cd_line& line)
	{
		Vector2d dir;
		if( r2 != NULL )
			dir = r2->getPos() - r->getPos();
		
		if( r2 == NULL || dir.normsqr() < 1e-12 )
			dir = -(r->getNormal() + r->getPre()->getNormal());

		if (dir.normsqr() < 1e-12)
			dir.set(1.0, 0.0);

		line.vec=dir.normalize();
		line.support=r;
		line.origin=r->getPos();
		line.normal.set(-line.vec[1],line.vec[0]);
	}
	
	//check if vv_ can resolve v
	inline bool isResolved(cd_vertex * v, cd_vertex * v_)
	{
		static Vector2d v1;
		static Vector2d v2;
		static cd_vertex * oldV=NULL;
		static double c=0;
	
		if( oldV!=v ){
			oldV=v;
			v1=v->getPos()-v->getPre()->getPos();
			v2=v->getPos()-v->getNext()->getPos();
			c=(v1[1]*v2[0]-v1[0]*v2[1]);
		}
	
		//check if tmp can resolve v
		Vector2d vec=v_->getPos()-v->getPos();
		double a=(vec[0]*v2[1]-vec[1]*v2[0]);
		double b=(vec[0]*v1[1]-vec[1]*v1[0]);
		return ((a*c)<0) && ((b*c)>0);
	}
	
	///////////////////////////////////////////////////////////////////////////////
	// For out most
	
	inline cd_vertex * 
	cut_dir_heuristic( cd_vertex * v, double alpha, double beta )
	{
		Vector2d nv= (v->getNormal()+v->getPre()->getNormal()).normalize();
	
		double max_score=0;
		cd_vertex * bestV=NULL;
		cd_vertex * ptr=v->getNext();
		
		do{
			Vector2d vec=ptr->getPos()-v->getPos();
			double score= (1+alpha*ptr->getConcavity())/(beta*vec.norm());
	
			if( score>max_score ){
				//check if it is facing the diff side
				Vector2d nvis= (ptr->getNormal()+ptr->getPre()->getNormal()).normalize();
				//see if this can resolve the notch
				if( isResolved(v,ptr) ){
					max_score=score;
					bestV=ptr;
				}
			}//end if score
			ptr=ptr->getNext();
		}while(ptr!=v);
	
		return bestV;
	}//end
	
	inline void find_a_good_cutline
	(cd_line& line, cd_vertex * r, double alpha, double beta)
	{
		cd_vertex * r2=cut_dir_heuristic(r,alpha,beta);
		setupCutLine(r,r2,line);
	}
	
	///////////////////////////////////////////////////////////////////////////////
	// For hole
	
	inline bool is_hole_ray_valid(cd_vertex* v, const Point2d& pt)
	{
		if (v == NULL || v->getPre() == NULL || v->getNext() == NULL) return true;
		Vector2d e0 = v->getPos() - v->getPre()->getPos();
		Vector2d e1 = v->getNext()->getPos() - v->getPos();
		Vector2d vec = pt - v->getPos();
		
		double c0 = e0[0]*vec[1] - e0[1]*vec[0];
		double c1 = vec[0]*e1[1] - vec[1]*e1[0];
		
		// In a CW hole, the obstacle interior has c0 < -1e-5 && c1 < -1e-5.
		// A ray pointing outward into free space has at least one >= -1e-5.
		return (c0 >= -1e-5 || c1 >= -1e-5);
	}

	inline
	Point2d find_MP(cd_vertex * v, cd_poly& poly)
	{
		if (v == NULL) return Point2d(0, 0);
		cd_vertex * head = poly.getHead();
		if (head == NULL) return v->getPos();
		
		cd_vertex * cur = head;
		Point2d best = head->getPos();
		double min_dist_sqr = 1e20;

		do {
			Point2d pt = cur->computeClosePt(v->getPos());
			double d2 = (pt - v->getPos()).normsqr();
			if (d2 < min_dist_sqr) {
				if (is_hole_ray_valid(v, pt)) {
					Vector2d dir = pt - v->getPos();
					if (d2 > 1e-8) {
						Vector2d dir_norm = dir.normalize();
						cd_line ray;
						ray.origin = v->getPos();
						ray.vec = dir_norm;
						ray.normal.set(-dir_norm[1], dir_norm[0]);
						ray.support = v;
						
						list<cd_vertex*> coll;
						poly.findCollEdges(coll, ray);
						
						double min_u = FLT_MAX;
						for (auto cv : coll) {
							if (cv->getU() > 1e-5 && cv->getU() < min_u) {
								min_u = cv->getU();
							}
						}
						double dist = sqrt(d2);
						if (min_u >= dist - 1e-3) {
							min_dist_sqr = d2;
							best = pt;
						}
					} else {
						min_dist_sqr = d2;
						best = pt;
					}
				}
			}
			cur = cur->getNext();
		} while (cur != head);

		if (min_dist_sqr > 1e19) {
			Vector2d dir = -(v->getNormal() + (v->getPre() ? v->getPre()->getNormal() : v->getNormal()));
			if (dir.normsqr() < 1e-12) dir.set(1.0, 0.0);
			best = v->getPos() + dir.normalize();
		}

		return best;
	}

	inline void find_a_good_cutline_for_hole
	(cd_line& line, cd_vertex * v, cd_poly& poly)
	{
		Point2d pt=find_MP(v,poly);
		cd_vertex tmp(pt);
		setupCutLine(v,&tmp,line);
	}

} //namespace acd2d

#endif //_CD2D_CUT_DIR_H_

