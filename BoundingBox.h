#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "Types.h"

class BoundingBox {
public:
    // default constructor
    BoundingBox();
    
    // initialize with specified components
    BoundingBox(const Eigen::Vector3d& min0, const Eigen::Vector3d& max0);
    
    // initialize with specified components
    BoundingBox(const Eigen::Vector3d& p);
    
    // expand bounding box to include point/ bbox
    void expandToInclude(const Eigen::Vector3d& p);
    void expandToInclude(const BoundingBox& b);

    // return the max dimension
    int maxDimension() const;
    
    // check if bounding box and face intersect
    bool intersect(const Eigen::Vector3d& p, double& dist) const;
    
    // member variables
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    Eigen::Vector3d extent;
};

#endif