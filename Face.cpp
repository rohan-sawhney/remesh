#include "HalfEdge.h"
#include "Mesh.h"
#include "BoundingBox.h"

bool Face::isBoundary() const
{
    return he->onBoundary;
}

double Face::area() const
{
    if (isBoundary()) {
        return 0.0;
    }
    
    return 0.5 * normal().norm();
}

Eigen::Vector3d Face::normal() const
{
    Eigen::Vector3d a = he->vertex->position;
    Eigen::Vector3d b = he->next->vertex->position;
    Eigen::Vector3d c = he->next->next->vertex->position;
    
    Eigen::Vector3d v1 = b - a;
    Eigen::Vector3d v2 = c - a;
    
    return v1.cross(v2);
}

BoundingBox Face::boundingBox() const
{
    Eigen::Vector3d p1 = he->vertex->position;
    
    if (isBoundary()) {
        return BoundingBox(p1, p1);
    }
    
    Eigen::Vector3d p2 = he->next->vertex->position;
    Eigen::Vector3d p3 = he->next->next->vertex->position;
    
    Eigen::Vector3d min = p1;
    Eigen::Vector3d max = p1;
    
    if (p2.x() < min.x()) min.x() = p2.x();
    if (p3.x() < min.x()) min.x() = p3.x();
    
    if (p2.y() < min.y()) min.y() = p2.y();
    if (p3.y() < min.y()) min.y() = p3.y();
    
    if (p2.z() < min.z()) min.z() = p2.z();
    if (p3.z() < min.z()) min.z() = p3.z();
    
    if (p2.x() > max.x()) max.x() = p2.x();
    if (p3.x() > max.x()) max.x() = p3.x();
    
    if (p2.y() > max.y()) max.y() = p2.y();
    if (p3.y() > max.y()) max.y() = p3.y();
    
    if (p2.z() > max.z()) max.z() = p2.z();
    if (p3.z() > max.z()) max.z() = p3.z();
    
    return BoundingBox(min, max);
}

Eigen::Vector3d Face::centroid() const
{
    if (isBoundary()) {
        return he->vertex->position;
    }
    
    Eigen::Vector3d centroid = (he->vertex->position +
                                he->next->vertex->position +
                                he->next->next->vertex->position) / 3.0;
    return centroid;
}

double Face::closestPoint(const Eigen::Vector3d& p, Eigen::Vector3d& c) const
{
    Eigen::Vector3d p1 = he->vertex->position;
    Eigen::Vector3d p2 = he->next->vertex->position;
    Eigen::Vector3d p3 = he->next->next->vertex->position;
    
    Eigen::Vector3d e1 = p2 - p1;
    Eigen::Vector3d e2 = p3 - p1;
    Eigen::Vector3d e3 = p3 - p2;
    
    // check if p is outside vertex region p1
    Eigen::Vector3d v1 = p - p1;
    double d1 = e1.dot(v1), d2 = e2.dot(v1);
    if (d1 <= 0 && d2 <= 0) {
        c = p1;
        return (p-c).squaredNorm();
    }
    
    // check if p is outside vertex region p2
    Eigen::Vector3d v2 = p - p2;
    double d3 = e1.dot(v2), d4 = e2.dot(v2);
    if (d3 >= 0 && d4 <= d3) {
        c = p2;
        return (p-c).squaredNorm();
    }
    
    // check if p is in edge region e1, if so return projection of p onto e1
    double vc = d1*d4 - d3*d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        double v = d1 / (d1-d3);
        c = p1 + v * e1;
        return (p-c).squaredNorm();
    }
    
    // check if p in vertex region outside p3
    Eigen::Vector3d v3 = p - p3;
    double d5 = e1.dot(v3), d6 = e2.dot(v3);
    if (d6 >= 0 && d5 <= d6) {
        c = p3;
        return (p-c).squaredNorm();
    }
    
    // check if p is in edge region e2, if so return projection of p onto e2
    double vb = d5*d2 - d1*d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        double w = d2 / (d2-d6);
        c = p1 + w * e2;
        return (p-c).squaredNorm();
    }
    
    // check if p is in edge region e3, if so return projection of p onto e3
    double va = d3*d6 - d5*d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        c = p2 + w * e3;
        return (p-c).squaredNorm();
    }
    
    // p inside face region. Compute point through its barycentric coordinates (u,v,w)
    double d = 1.0 / (va + vb + vc);
    double v = vb * d;
    double w = vc * d;
    c = p1 + e1 * v + e2 * w;
    return (p-c).squaredNorm();
}
