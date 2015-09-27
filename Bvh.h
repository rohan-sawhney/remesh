#ifndef BVH_H
#define BVH_H

#include "Types.h"
#include "BoundingBox.h"

class Node {
public:
    // member variables
    BoundingBox boundingBox;
    int startId, range, rightOffset;
};

class Bvh {
public:
    Bvh(const int leafSize0 = 1);
    
    // builds the bvh
    void build(const Mesh& mesh);
    
    // checks if a face overlaps with another face. Returns face id
    double nearestPoint(const Eigen::Vector3d& p, Eigen::Vector3d& np) const;
    
private:    
    
    int nodeCount, leafCount, leafSize;
    std::vector<Node> flatTree;
    std::vector<Face> faces;
};

#endif
