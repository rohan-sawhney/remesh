#include "Vertex.h"
#include "HalfEdge.h"
#include "Edge.h"
#include "Face.h"

std::vector<HalfEdge> isolated;

bool Vertex::isIsolated() const
{
    return he == isolated.begin();
}

bool Vertex::onBoundary() const
{
    HalfEdgeCIter h = he;
    do {
        if (h->onBoundary) {
            return true;
        }
        h = h->flip->next;
        
    } while (h != he);

    return false;
}

int Vertex::targetValence() const
{
    if (onBoundary()) return 4;

    return 6;
}

int Vertex::valence() const
{
    int v = 0;
    HalfEdgeCIter h = he;
    do {
        v ++;
        h = h->flip->next;

    } while (h != he);

    return v;
}

Eigen::Vector3d Vertex::normal() const
{
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    if (isIsolated()) return normal;
    
    HalfEdgeCIter h = he;
    do {
        Eigen::Vector3d e1 = h->next->vertex->position - position;
        Eigen::Vector3d e2 = h->next->next->vertex->position - position;
        
        double d = e1.dot(e2) / sqrt(e1.squaredNorm() * e2.squaredNorm());
        if (d < -1.0) d = -1.0;
        else if (d >  1.0) d = 1.0;
        double angle = acos(d);
        
        Eigen::Vector3d n = h->face->normal();
        normal += angle * n;
        
        h = h->flip->next;
    } while (h != he);
    
    if (!normal.isZero()) normal.normalize();
    return normal;
}

bool Vertex::shareEdge(VertexCIter& v) const
{
    HalfEdgeCIter h = he;
    do {
        if (h->flip->vertex == v) {
            return true;
        }
        
        h = h->flip->next;
        
    } while (h != he);

    return false;
}

bool Vertex::isFeature() const
{
    HalfEdgeCIter h = he;
    do {
        if (h->edge->feature) return true;
        
        h = h->flip->next;
        
    } while (h != he);
    
    return false;
}
