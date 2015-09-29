#ifndef EDGE_H
#define EDGE_H

#include "Types.h"

class Edge {
public:
    // one of the two half edges associated with this edge
    HalfEdgeIter he;
    
    double lengthSquared() const;
    
    // id between 0 and |E|-1
    int index;
    
    // flag for deletion
    bool remove;
    
    // feature flag
    bool feature;
};

#endif
