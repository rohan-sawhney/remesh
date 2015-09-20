#ifndef HALF_EDGE_H
#define HALF_EDGE_H

#include "Types.h"

class HalfEdge {
public:
    // next halfedge around the current face
    HalfEdgeIter next;
    
    // other halfedge associated with this edge
    HalfEdgeIter flip;
    
    // vertex at the tail of the halfedge
    VertexIter vertex;
    
    // edge associated with this halfedge
    EdgeIter edge;
    
    // face associated with this halfedge
    FaceIter face;
    
    // checks if this halfedge is contained in boundary loop
    bool onBoundary;
    
    // flag for deletion
    bool remove;
};

#endif
