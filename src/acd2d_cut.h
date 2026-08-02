//------------------------------------------------------------------------------
//  Copyright 2007-2012 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------

#ifndef _CD2D_CUT_H_
#define _CD2D_CUT_H_

#include "acd2d_data.h"
#include "acd2d_bridge.h"

///////////////////////////////////////////////////////////////////////////////
//
//  Following procedures will cut the polygon into two pieces
//  for a given cut line and reulting polygons will be put into.
//  result list.
//  
//  The main function is : cutPolys
//
///////////////////////////////////////////////////////////////////////////////

namespace acd2d
{
	
	inline cd_vertex* checkDegeneracy(cd_vertex* v1)
	{
		cd_vertex * v=v1;
		if (v == NULL) return NULL;
	   
		if( v->getNext() != NULL && v->getPos().almost_equ(v->getNext()->getPos()) ){ //dup
			cd_vertex * dup=v->getNext();
			v->setNext(dup->getNext());
			removeBridge(dup);
			delete dup;
		}
		else {
			v=v->getNext();
		}
		
		if( v != NULL && v->getNext() != NULL && v->getPos().almost_equ(v->getNext()->getPos()) ){ //dup
			cd_vertex * dup=v->getNext();
			v->setNext(dup->getNext());
			removeBridge(dup);
			delete dup;
		}
		else if (v != NULL) {
			v=v->getNext();
		}
		
		if( v != NULL && v->getNext() != NULL && v->getPos().almost_equ(v->getNext()->getPos()) ){ //dup
			cd_vertex * prev = v->getPre();
			cd_vertex * next = v->getNext();
			if (prev != NULL) {
				prev->setNext(next);
				removeBridge(v);
				delete v;
				return prev;
			}
		}
		return v1;
	}
	
	/**
	 * update the information, such as normal and check
	 * reflectivity.
	 */
	inline void updateInfo(cd_vertex* v)
	{
		if (v == NULL) return;
		cd_vertex* cur = v;
		for( int i=0; i<4; i++ ){
			if (cur == NULL) break;
			cur->computeNormal();
			cur->computeReflex();
			if( !cur->isReflex() )
				cur->setConcavity(0);
			cur=cur->getNext();
		}
	}
	
	/**
	 * split edges in the cut.
	 */
	inline void addDiagnal( cd_vertex* & v1, cd_vertex* & v2 )
	{
		cd_vertex * v1n=v1->getNext();
		cd_vertex * v2n=v2->getNext();
		
		cd_vertex * n11=new cd_vertex(v1->getInterPt());
		cd_vertex * n12=new cd_vertex(v1->getInterPt());
		cd_vertex * n21=new cd_vertex(v2->getInterPt());
		cd_vertex * n22=new cd_vertex(v2->getInterPt());
		
		v1->setNext(n11);
		n11->setNext(n22);
		n22->setNext(v2n);
		
		v2->setNext(n21);
		n21->setNext(n12);
		n12->setNext(v1n);
		
		v1 = checkDegeneracy(v1);
		v2 = checkDegeneracy(v2);
		if (v1 != NULL) updateInfo(v1);
		if (v2 != NULL) updateInfo(v2);
	}
	
	///////////////////////////////////////////////////////////////////////////////
	//
	//  For the out most boundary
	//
	///////////////////////////////////////////////////////////////////////////////
	
	
	/**
	 * Find the edges that will split the out most chain
	 */
	inline pair<cd_vertex*,cd_vertex*> 
	FindCut_Out( cd_poly& poly, cd_line& cut_l )
	{
		list<cd_vertex*> coll;
		typedef list<cd_vertex*>::iterator VIT;
		poly.findCollEdges(coll, cut_l);
		
		double min_U=FLT_MAX;
		cd_vertex* closest=NULL;
		for( VIT iv=coll.begin();iv!=coll.end();iv++){
			cd_vertex* cur=*iv;
			if( cur==cut_l.support ) continue;
			if( cur==cut_l.support->getPre() ) continue;
			if( cur->getU()<-1e-5 ) continue;
			if( cur->getU()<min_U ){
				min_U=cur->getU();
				closest=cur;
			}
		}
	
		if(closest!=NULL) 
			return pair<cd_vertex*,cd_vertex*>(cut_l.support,closest);
	
		return pair<cd_vertex*,cd_vertex*>(NULL, NULL);
	}
	
	/**
	 * cut polys using cut_l into two polygons.
	 */
	inline cd_diagonal cutPolys
	(pair<cd_polygon,cd_polygon>& result, cd_poly& poly, cd_line& cut_l)
	{
		//find cuts
		pair<cd_vertex*, cd_vertex*> cut = FindCut_Out(poly,cut_l);
		cd_vertex * v1=cut.first;
		cd_vertex * v2=cut.second;
		if (v1 == NULL || v2 == NULL) {
			return cd_diagonal(poly.getHead()->getPos(), poly.getHead()->getPos());
		}
	
		cd_vertex * nv1=v1->getNext();
		cd_vertex * nv2=v2->getNext();
		UpdateBridge(v1,nv1,v2,nv2);
	
		addDiagnal(v1,v2);
		
		//store polygonal chains
		cd_poly p1(cd_poly::POUT),p2(cd_poly::POUT);
		p1.set(cd_poly::POUT,v1); p2.set(cd_poly::POUT,v2);
		p1.updateSize(); p2.updateSize();
	
		result.first.push_back(p1);
		result.second.push_back(p2); 
	
		return cd_diagonal(v1->getInterPt(),v2->getInterPt());
	}
	
	///////////////////////////////////////////////////////////////////////////////
	//
	//  For the hole boundary
	//
	///////////////////////////////////////////////////////////////////////////////
	
	inline pair<cd_vertex*,cd_vertex*> 
	FindCut_In( cd_poly& out, cd_poly& in, cd_line& cut_l )
	{
		list<cd_vertex*> coll;
		out.findCollEdges(coll, cut_l);
		
		double min_u=FLT_MAX;
		cd_vertex * min_v=NULL;
        for(auto v : coll )
        {
			if( v->getU()<0 ) continue; //not in the right dir
			if( v->getU()<min_u ){ 
				min_u=v->getU();
				min_v=v;
			}//end if
		}//end for
	
		if( min_v!=NULL )
			return pair<cd_vertex*,cd_vertex*> (cut_l.support,min_v);

		return pair<cd_vertex*,cd_vertex*>(NULL, NULL);
	}
	
	inline cd_diagonal
	mergeHole(cd_poly& out, cd_poly& hole, cd_line& cut_l)
	{
		//find cuts
		cd_vertex * v1, *v2;
		pair<cd_vertex*, cd_vertex*> cut = FindCut_In(out,hole,cut_l);
		v1=cut.first;
		v2=cut.second;
		if (v1 == NULL || v2 == NULL) {
			v1 = cut_l.support ? cut_l.support : hole.getHead();
			v2 = out.getHead();
		} else {
			v1->Intersect(cut_l);
		}
		if (v1 != NULL && v2 != NULL) {
			addDiagnal(v1,v2);
			out.updateSize();
			return cd_diagonal(v1->getPos(),v2->getPos());
		}
		return cd_diagonal(hole.getHead()->getPos(), hole.getHead()->getPos());
	}
}//namespace acd2d

#endif //_CD2D_CUT_H_
