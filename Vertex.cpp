#include "Vertex.h"
#include "HalfEdge.h"
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
        if (h->onBoundary) return true;
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
    Eigen::Vector3d normal;
    normal.setZero();
    
    double angle;
    
    HalfEdgeCIter h = he;
    do {
        Eigen::Vector3d e1 = h->flip->vertex->position - position;
        Eigen::Vector3d e2 = h->flip->next->flip->vertex->position - position;
        
        e1.normalize();
        e2.normalize();
        
        angle = acos(e1.dot(e2) / sqrt(e1.dot(e1) * e2.dot(e2)));
        normal += angle * h->face->normal();
        
        h = h->flip->next;
        
    } while (h != he);
    
    normal.normalize();
    return normal;
}