#include "Mesh.h"
#include "MeshIO.h"
#include <limits>

Mesh::Mesh()
{
    
}

Mesh::Mesh(const Mesh& mesh)
{
    *this = mesh;
}

bool Mesh::read(const std::string& fileName)
{
    std::ifstream in(fileName.c_str());

    if (!in.is_open()) {
        std::cerr << "Error: Could not open file for reading" << std::endl;
        return false;
    }
    
    bool readSuccessful = false;
    if ((readSuccessful = MeshIO::read(in, *this))) {
        normalize();
    }
    
    return readSuccessful;
}

bool Mesh::write(const std::string& fileName) const
{
    std::ofstream out(fileName.c_str());
    
    if (!out.is_open()) {
        std::cerr << "Error: Could not open file for writing" << std::endl;
        return false;
    }
    
    MeshIO::write(out, *this);
    
    return false;
}

void Mesh::splitEdge(EdgeIter& e, const Eigen::Vector3d& position)
{
    HalfEdgeIter he = e->he;
    HalfEdgeIter flip = he->flip;
    
    VertexIter v2 = flip->vertex;
    
    FaceIter f0 = he->face;
    FaceIter f3 = flip->face;
    
    // insert new vertex
    VertexIter v = vertices.insert(vertices.end(), Vertex());
    v->position = position;
    v->index = (int)vertices.size()-1;
    
    // insert new halfedge
    EdgeIter e1 = edges.insert(edges.end(), Edge());
    e1->index = (int)edges.size()-1;
    
    // insert new halfedge
    HalfEdgeIter he1 = halfEdges.insert(halfEdges.end(), HalfEdge());
    he1->index = (int)halfEdges.size()-1;
    HalfEdgeIter flip1 = halfEdges.insert(halfEdges.end(), HalfEdge());
    flip1->index = (int)halfEdges.size()-1;
    
    // set flip
    he1->flip = flip1;
    flip1->flip = he1;
    
    // set halfedge vertex
    flip->vertex = v;
    he1->vertex = v;
    flip1->vertex = v2;
 
    // set vertex halfedge
    v->he = he1;
    v2->he = flip1;
    
    // set halfedge edge
    he1->edge = e1;
    flip1->edge = e1;
    
    // set edge halfedge
    e1->he = he1;

    if (!he->onBoundary) {
        HalfEdgeIter heNext = he->next;
        HalfEdgeIter heNextNext = heNext->next;
        
        EdgeIter e2 = edges.insert(edges.end(), Edge());
        e2->index = (int)edges.size()-1;
        
        HalfEdgeIter he2 = halfEdges.insert(halfEdges.end(), HalfEdge());
        he2->index = (int)halfEdges.size()-1;
        HalfEdgeIter flip2 = halfEdges.insert(halfEdges.end(), HalfEdge());
        flip2->index = (int)halfEdges.size()-1;
        
        FaceIter f1 = faces.insert(faces.end(), Face());
        f1->index = (int)faces.size()-1;
        
        // set flip
        he2->flip = flip2;
        flip2->flip = he2;
        
        // set halfedge next
        he->next = he2;
        he2->next = heNextNext;
        
        he1->next = heNext;
        heNext->next = flip2;
        flip2->next = he1;
        
        // set halfedge vertex
        he2->vertex = v;
        flip2->vertex = heNextNext->vertex;
        
        // set halfedge edge
        he2->edge = e2;
        flip2->edge = e2;
        
        // set edge halfedge
        e2->he = he2;
        
        // set halfedge face
        he2->face = f0;
        he1->face = f1;
        heNext->face = f1;
        flip2->face = f1;
        
        // set face halfedge
        f0->he = he2;
        f1->he = flip2;
        
    } else {
        // set halfedge next
        he1->next = he->next;
        he->next = he1;
        he1->onBoundary = true;
        
        // set vertex and edge halfedge
        v->he = flip;
        e1->he = flip1;
        
        // set halfedge face
        FaceIter newFace = faces.insert(faces.end(), Face());
        newFace->index = (int)faces.size()-1;
        he1->face = newFace;
        newFace->he = he1;
    }
    
    if (!flip->onBoundary) {
        HalfEdgeIter flipNext = flip->next;
        HalfEdgeIter flipNextNext = flipNext->next;
        
        EdgeIter e3 = edges.insert(edges.end(), Edge());
        e3->index = (int)edges.size()-1;
        
        HalfEdgeIter he3 = halfEdges.insert(halfEdges.end(), HalfEdge());
        he3->index = (int)halfEdges.size()-1;
        HalfEdgeIter flip3 = halfEdges.insert(halfEdges.end(), HalfEdge());
        flip3->index = (int)halfEdges.size()-1;
    
        FaceIter f2 = faces.insert(faces.end(), Face());
        f2->index = (int)faces.size()-1;
        
        // set flip
        he3->flip = flip3;
        flip3->flip = he3;
        
        // set halfedge next
        flipNext->next = he3;
        he3->next = flip;
        
        flip1->next = flip3;
        flip3->next = flipNextNext;
        flipNextNext->next = flip1;
        
        // set halfedge vertex
        flip3->vertex = v;
        he3->vertex = flipNextNext->vertex;
        
        // set halfedge edge
        flip3->edge = e3;
        he3->edge = e3;
        
        // set edge halfedge
        e3->he = flip3;
        
        // set halfedge face
        flip1->face = f2;
        flip3->face = f2;
        flipNextNext->face = f2;
        he3->face = f3;
        
        // set face halfedge
        f2->he = flip3;
        f3->he = he3;
        
    } else {
        // set halfedge next
        flip1->next = flip;
        
        // set flip1 prev next to flip1
        HalfEdgeCIter h = flip1;
        do {
            if (h->flip->onBoundary) {
                h->flip->next = flip1;
                break;
            }
            
            h = h->flip->next;
        } while (h != flip1);
        
        flip1->onBoundary = true;
        v2->he = he1->next;
        
        // set halfedge face
        FaceIter newFace = faces.insert(faces.end(), Face());
        newFace->index = (int)faces.size()-1;
        flip1->face = newFace;
        newFace->he = flip1;
    }
}

bool Mesh::validCollapse(EdgeIter& e)
{
    HalfEdgeCIter he = e->he;
    HalfEdgeCIter flip = he->flip;
    
    VertexCIter v2 = flip->vertex;
    VertexCIter v3 = he->next->next->vertex;
    VertexCIter v4 = flip->next->next->vertex;
    
    // check for one ring intersection
    HalfEdgeCIter h = he;
    do {
        VertexCIter v = h->flip->vertex;
        if (v != v2 && v != v3 && v != v4) {
            if (v->shareEdge(v2)) return false;
        }
    
        h = h->flip->next;
        
    } while (h != he);
    
    return true;
}

void Mesh::collapseEdge(EdgeIter& e)
{
    HalfEdgeIter he = e->he;
    HalfEdgeIter heNext = he->next;
    HalfEdgeIter heNextNext = heNext->next;
    
    HalfEdgeIter flip = he->flip;
    HalfEdgeIter flipNext = flip->next;
    HalfEdgeIter flipNextNext = flipNext->next;
    
    VertexIter v1 = he->vertex;
    VertexIter v2 = flip->vertex;
    VertexIter v3 = heNextNext->vertex;
    VertexIter v4 = flipNextNext->vertex;
    
    EdgeIter e2 = heNextNext->edge;
    EdgeIter e3 = flipNext->edge;
    
    FaceIter f = he->face;
    FaceIter fFlip = flip->face;
    
    // set halfEdge vertex
    HalfEdgeIter h = flip;
    do {
        h->vertex = v1;
        h = h->flip->next;
    
    } while (h != flip);
    
    // set vertex halfEdge
    v1->he = heNext;
    v3->he = heNextNext->flip->next;
    v4->he = flipNextNext;

    // set next halfEdge
    heNext->next = heNextNext->flip->next;
    heNextNext->flip->next->next->next = heNext;
    
    flipNextNext->next = flipNext->flip->next;
    flipNext->flip->next->next->next = flipNextNext;
    
    // set halfEdge face
    heNext->face = heNextNext->flip->face;
    flipNextNext->face = flipNext->flip->face;
    
    // set face halfEdge
    if (heNextNext->flip->face->he == heNextNext->flip) heNextNext->flip->face->he = heNext;
    if (flipNext->flip->face->he == flipNext->flip) flipNext->flip->face->he = flipNextNext;
    
    // mark for deletion
    v2->remove = true;
    e->remove = true;
    e2->remove = true;
    e3->remove = true;
    he->remove = true;
    flip->remove = true;
    heNextNext->remove = true;
    heNextNext->flip->remove = true;
    flipNext->remove = true;
    flipNext->flip->remove = true;
    f->remove = true;
    fFlip->remove = true;
}

template <typename T>
void swapMarkedRemove(std::vector<T>& vec, int& size)
{
    int n = (int)vec.size();

    if (n > 0) {
        int start = 0, end = n-1;
        
        while (true) {
            // find first removed and valid T
            while (!vec[start].remove && start < end) start++;
            while (vec[end].remove && start < end) end--;
            
            if (start >= end) break;
            
            // swap
            std::swap(vec[start], vec[end]);
        }
        
        size = vec[start].remove ? start : start+1;
    }
}

void Mesh::resetLists()
{
    int nV, nE, nF, nHE;
    
    swapMarkedRemove(vertices, nV);
    swapMarkedRemove(edges, nE);
    swapMarkedRemove(halfEdges, nHE);
    swapMarkedRemove(faces, nF);
    
    // reassign iterators
    for (int i = 0; i < nV; i++) {
        if (!vertices[i].isIsolated()) {
            vertices[i].he = halfEdges.begin() + vertices[i].he->index;
        }
    }
    
    for (int i = 0; i < nE; i++) {
        edges[i].he = halfEdges.begin() + edges[i].he->index;
    }
    
    for (int i = 0; i < nHE; i++) {
        halfEdges[i].vertex = vertices.begin() + halfEdges[i].vertex->index;
        halfEdges[i].edge = edges.begin() + halfEdges[i].edge->index;
        halfEdges[i].flip = halfEdges.begin() + halfEdges[i].flip->index;
        halfEdges[i].next = halfEdges.begin() + halfEdges[i].next->index;
        halfEdges[i].face = faces.begin() + halfEdges[i].face->index;
    }
    
    for (int i = 0; i < nF; i++) {
        faces[i].he = halfEdges.begin() + faces[i].he->index;
    }
    
    // erase
    vertices.resize(nV);
    edges.resize(nE);
    halfEdges.resize(nHE);
    faces.resize(nF);
    
    // reindex
    MeshIO::indexElements(*this);
}

bool Mesh::validFlip(EdgeIter& e)
{
    HalfEdgeCIter he = e->he;
    HalfEdgeCIter flip = he->flip;
    
    if (he->onBoundary || flip->onBoundary) return false;
    
    VertexCIter v1 = he->next->next->vertex;
    VertexCIter v2 = flip->next->next->vertex;
    
    // check if v1 and v2 share an edge
    if (v1->shareEdge(v2)) return false;

    return true;
}

void Mesh::flipEdge(EdgeIter& e)
{
    HalfEdgeIter he = e->he;
    HalfEdgeIter heNext = he->next;
    HalfEdgeIter heNextNext = heNext->next;
    
    HalfEdgeIter flip = he->flip;
    HalfEdgeIter flipNext = flip->next;
    HalfEdgeIter flipNextNext = flipNext->next;
    
    VertexIter v1 = he->vertex;
    VertexIter v2 = heNext->vertex;
    VertexIter v3 = heNextNext->vertex;
    VertexIter v4 = flipNextNext->vertex;
    
    FaceIter f = he->face;
    FaceIter fFlip = flip->face;
    
    // set halfEdge vertex
    he->vertex = v4;
    flip->vertex = v3;
    
    // set next halfEdge
    he->next = heNextNext;
    heNextNext->next = flipNext;
    flipNext->next = he;
    
    flip->next = flipNextNext;
    flipNextNext->next = heNext;
    heNext->next = flip;

    // set halfEdge face
    heNext->face = fFlip;
    flipNext->face = f;
    
    // set face halEdge
    f->he = he;
    fFlip->he = flip;
    
    // set vertex halfEdge
    if (v1->he == he) v1->he = flipNext;
    if (v2->he == flip) v2->he = heNext;
}

void Mesh::splitLongEdges(const double high)
{    
    EdgeIter eEnd = edges.end();
    for (EdgeIter e = edges.begin(); e != eEnd; e++) {
        if (e->lengthSquared() > high) {
            
            Eigen::Vector3d v1 = e->he->vertex->position;
            Eigen::Vector3d v2 = e->he->flip->vertex->position;
            
            splitEdge(e, v1 + (v2-v1) * 0.5);
        }
    }
}

void Mesh::collapseShortEdges(const double low, const double high)
{
    for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
        
        double l = e->lengthSquared();
        if (!e->remove && l < low && l > std::numeric_limits<double>::epsilon()) {
            
            bool collapse = true;
            VertexCIter v1 = e->he->vertex;
            VertexCIter v2 = e->he->flip->vertex;
        
            // check if edge should be collapsed
            HalfEdgeCIter he = v1->he;
            do {
                double d = (v2->position - he->flip->vertex->position).squaredNorm();
                if (he->onBoundary || d > high) {
                    collapse = false;
                    break;
                }
            
                he = he->flip->next;
            
            } while (he != v1->he);
            
            if (collapse && validCollapse(e)) {
                collapseEdge(e);
            }
        }
    }
    
    resetLists();
}

void Mesh::equalizeValences()
{
    for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
        
        if (validFlip(e)) {
            // compute pre and post deviation and decide whether to flip
            VertexCIter v1 = e->he->vertex;
            VertexCIter v2 = e->he->flip->vertex;
            VertexCIter v3 = e->he->next->next->vertex;
            VertexCIter v4 = e->he->flip->next->next->vertex;
            
            const int preDeviation = std::abs(v1->valence() - v1->targetValence()) +
                                     std::abs(v2->valence() - v2->targetValence()) +
                                     std::abs(v3->valence() - v3->targetValence()) +
                                     std::abs(v4->valence() - v4->targetValence());
            
            flipEdge(e);

            const int postDeviation = std::abs(v1->valence() - v1->targetValence()) +
                                      std::abs(v2->valence() - v2->targetValence()) +
                                      std::abs(v3->valence() - v3->targetValence()) +
                                      std::abs(v4->valence() - v4->targetValence());

            if (preDeviation <= postDeviation) {
                flipEdge(e);
            }
        }
    }
}

void Mesh::tangentialRelaxation()
{
    // compute barycenter of vertex neighborhoods
    std::unordered_map<int, Eigen::Vector3d> q;
    std::unordered_map<int, Eigen::Vector3d> n;
    
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        
        Eigen::Vector3d barycenter;
        barycenter.setZero();
        int N = 0;
        HalfEdgeCIter he = v->he;
        do {
            barycenter += he->flip->vertex->position;
            N ++;
            he = he->flip->next;
            
        } while (he != v->he);
        
        q[v->index] = barycenter / (double)N;
        n[v->index] = v->normal();
    }
    
    // move vertices to new position
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        if (!v->onBoundary()) {
            v->position = q[v->index] + n[v->index].dot(v->position - q[v->index]) * n[v->index];
        }
    }
}

void Mesh::projectToSurface(const Mesh& mesh)
{
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        if (!v->onBoundary()) {
            
            Eigen::Vector3d nearestPoint;
            bvh.nearestPoint(v->position, nearestPoint);
            v->position = nearestPoint;
        }
    }
}

void Mesh::remesh(const double edgeLength, const int iterations)
{
    // low and high edge lengths
    const double low  = (4.0 / 5.0) * edgeLength;
    const double high = (4.0 / 3.0) * edgeLength;
    double low2 = low*low;
    double high2 = high*high;
    
    // build bvh
    Mesh mesh(*this);
    bvh.build(&mesh);
    
    // run algorithm
    for (int i = 0; i < iterations; i++) {
        splitLongEdges(high2);
        collapseShortEdges(low2, high2);
        equalizeValences();
        tangentialRelaxation();
        //projectToSurface(mesh);
    }
}

void Mesh::normalize()
{
    // compute center of mass
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        cm += v->position;
    }
    cm /= (double)vertices.size();
    
    // translate to origin
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position -= cm;
    }
    
    // determine radius
    double rMax = 0;
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        rMax = std::max(rMax, v->position.norm());
    }
    
    // rescale to unit sphere
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position /= rMax;
    }
}
