#ifndef VERTEX_H
#define VERTEX_H

#include "Types.h"

class Vertex {
public:
    // outgoing halfedge
    HalfEdgeIter he;
    
    // location in 3d
    Eigen::Vector3d position;
    
    // id between 0 and |V|-1
    int index;
    
    // checks if vertex is contained in any edge or face
    bool isIsolated() const;
    
    // checks if v is on boundary
    bool onBoundary() const;
    
    // returns target valence
    int targetValence() const;
    
    // returns valence
    int valence() const;
    
    // returns angle weighted vertex normal
    Eigen::Vector3d normal() const;
    
    // checks if vertices share an edge
    bool shareEdge(VertexCIter& v) const;
    
    // flag for deletion
    bool remove;
};

#endif
