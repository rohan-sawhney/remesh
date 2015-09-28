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
    Eigen::Vector3d normal;
    Eigen::Vector3d n;
    normal.setZero();
    
    if (isIsolated()) {
        return normal;
    }
    
    double angle;
    
    HalfEdgeCIter h = he;
    do {        
        Eigen::Vector3d e1 = h->flip->vertex->position - position;
        Eigen::Vector3d e2 = h->flip->next->flip->vertex->position - position;
        
        double c = e1.dot(e2) / sqrt(e1.dot(e1) * e2.dot(e2));
        if (c < -1.0) c = -1.0;
        else if (c >  1.0) c = 1.0;
        angle = acos(c);
            
        n = h->face->normal();
        if (n.norm() == 0) {
            n.setZero();
            
        } else {
            n.normalize();
        }
        
        normal += angle * n;
        
        h = h->flip->next;
        
    } while (h != he);
    
    normal.normalize();
    return normal;
}

bool Vertex::shareEdge(VertexCIter& v) const
{
    HalfEdgeCIter h1 = he;
    do {
        HalfEdgeCIter h2 = v->he;
        do {
            if (h1 == h2->flip) return true;
            h2 = h2->flip->next;
            
        } while (h2 != v->he);
        
        h1 = h1->flip->next;
        
    } while (h1 != he);
    
    return false;
}