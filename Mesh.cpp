#include "Mesh.h"
#include "MeshIO.h"

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

void Mesh::splitEdge(const int eIdx, const Eigen::Vector3d& position)
{
    Edge e = edges[eIdx];
    
    HalfEdgeIter he = e.he;
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
    HalfEdgeIter flip1 = halfEdges.insert(halfEdges.end(), HalfEdge());
    he1->flip = flip1;
    flip1->flip = he1;
    
    // set halfedge vertex
    flip->vertex = v;
    he1->vertex = v;
    flip1->vertex = v2;
 
    // set vertex halfedge
    v->he = he1;
    if (v2->he == flip) v2->he = flip1;
    
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
        HalfEdgeIter flip2 = halfEdges.insert(halfEdges.end(), HalfEdge());
        he2->flip = flip2;
        flip2->flip = he2;
        
        FaceIter f1 = faces.insert(faces.end(), Face());
        f1->index = (int)faces.size()-1;
        
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
        HalfEdgeIter flip3 = halfEdges.insert(halfEdges.end(), HalfEdge());
        he3->flip = flip3;
        flip3->flip = he3;
        
        FaceIter f2 = faces.insert(faces.end(), Face());
        f2->index = (int)faces.size()-1;
        
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
        HalfEdgeCIter h = v2->he;
        do {
            if (h->flip->onBoundary) {
                h->flip->next = flip1;
                break;
            }
            
            h = h->flip->next;
        } while (h != v2->he);
        
        flip1->onBoundary = true;
        
        // set halfedge face
        FaceIter newFace = faces.insert(faces.end(), Face());
        newFace->index = (int)faces.size()-1;
        flip1->face = newFace;
        newFace->he = flip1;
    }
}

void Mesh::collapseEdge(const int eIdx)
{
    Edge e1 = edges[eIdx];
    
    HalfEdgeIter he = e1.he;
    HalfEdgeIter heNext = he->next;
    HalfEdgeIter heNextNext = heNext->next;
    
    HalfEdgeIter flip = he->flip;
    HalfEdgeIter flipNext = flip->next;
    HalfEdgeIter flipNextNext = flipNext->next;
    
    VertexIter v1 = he->vertex;
    VertexIter v2 = heNext->vertex;
    
    EdgeIter e2 = heNextNext->edge;
    EdgeIter e3 = flipNextNext->edge;
    
    FaceIter f = he->face;
    FaceIter fFlip = flip->face;
    
    // set halfEdge vertex
    HalfEdgeIter h = v2->he;
    do {
        h->vertex = v1;
        h = h->flip->next;
        
    } while (h != v2->he);

    // set next halfEdge
    heNext->next = heNextNext->flip->next;
    heNextNext->flip->next->next->next = heNext;
    
    flipNext->next = flipNextNext->flip->next;
    flipNextNext->flip->next->next->next = flipNext;
    
    // set halfEdge face
    heNext->face = heNextNext->flip->face;
    flipNext->face = flipNextNext->flip->face;
    
    // set face halfEdge
    if (heNextNext->flip->face->he == heNextNext->flip) heNextNext->flip->face->he = heNext;
    if (flipNextNext->flip->face->he == flipNextNext->flip) flipNextNext->flip->face->he = flipNext;
    
    // set vertex halfEdge
    v1->he = heNext;
    
    // mark for deletion
    v2->remove = true;
    e1.remove = true;
    e2->remove = true;
    e3->remove = true;
    he->remove = true;
    flip->remove = true;
    heNextNext->remove = true;
    heNextNext->flip->remove = true;
    flipNextNext->remove = true;
    flipNextNext->flip->remove = true;
    f->remove = true;
    fFlip->remove = true;
}

void Mesh::resetLists()
{
    /*
    // vertices
    std::vector<Vertex> newVertexList;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        if (!v->remove) {
            VertexIter nv = newVertexList.insert(newVertexList.end(), Vertex());
            
            // update the vertex and its associated halfedge
            nv->index = (int)newVertexList.size()-1;
            nv->he = v->he;
            v->he->vertex = nv;
        }
    }
    vertices = newVertexList;
    
    // edges
    std::vector<Edge> newEdgeList;
    for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
        if (!e->remove) {
            EdgeIter ne = newEdgeList.insert(newEdgeList.end(), Edge());
            
            // update the edge and its associated halfedge
            ne->index = (int)newEdgeList.size()-1;
            ne->he = e->he;
            e->he->edge = ne;
        }
    }
    edges = newEdgeList;
    
    // faces
    std::vector<Face> newFaceList;
    for (FaceIter f = faces.begin(); f != faces.end(); f++) {
        if (!f->remove) {
            FaceIter nf = newFaceList.insert(newFaceList.end(), Face());
            
            // update the edge and its associated halfedge
            nf->index = (int)newFaceList.size()-1;
            nf->he = f->he;
            f->he->face = nf;
        }
    }
    faces = newFaceList;
     */
}

void Mesh::flipEdge(const int eIdx)
{
    Edge e = edges[eIdx];
    
    HalfEdgeIter he = e.he;
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
    int edgeCount = (int)edges.size();
    for (int i = 0; i < edgeCount; i++) {
        Edge e = edges[i];
        
        if (e.lengthSquared() > high) {
            
            Eigen::Vector3d v1 = e.he->vertex->position;
            Eigen::Vector3d v2 = e.he->flip->vertex->position;
            Eigen::Vector3d mid = v1 + 0.5 * (v2-v1);
            
            splitEdge(e.index, mid);
        }
    }
}

void Mesh::collapseShortEdges(const double low, const double high)
{
    for (EdgeCIter e = edges.begin(); e != edges.end(); e++) {
        
        if (!e->remove && e->lengthSquared() < low) {
            
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
            
            if (collapse) {
                collapseEdge(e->index);
            }
        }
    }
    
    resetLists();
}

void Mesh::equalizeValences()
{
    for (EdgeCIter e = edges.begin(); e != edges.end(); e++) {
        
        if (!(e->he->onBoundary || e->he->flip->onBoundary)) {
            // compute pre and post deviation and decide whether to flip
            VertexCIter v1 = e->he->vertex;
            VertexCIter v2 = e->he->flip->vertex;
            VertexCIter v3 = e->he->next->next->vertex;
            VertexCIter v4 = e->he->flip->next->next->vertex;
            
            const int preDeviation = std::abs(v1->valence() - v1->targetValence()) +
                                     std::abs(v2->valence() - v2->targetValence()) +
                                     std::abs(v3->valence() - v3->targetValence()) +
                                     std::abs(v4->valence() - v4->targetValence());
            
            flipEdge(e->index);

            const int postDeviation = std::abs(v1->valence() - v1->targetValence()) +
                                      std::abs(v2->valence() - v2->targetValence()) +
                                      std::abs(v3->valence() - v3->targetValence()) +
                                      std::abs(v4->valence() - v4->targetValence());

            if (preDeviation <= postDeviation) {
                flipEdge(e->index);
            }
        }
    }
}

void Mesh::tangentialRelaxation()
{
    // compute barycenter of vertex neighborhoods
    std::unordered_map<int, Eigen::Vector3d> q;
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        
        Eigen::Vector3d barycenter;
        barycenter.setZero();
        int n = 0;
        HalfEdgeCIter he = v->he;
        do {
            barycenter += he->flip->vertex->position;
            n ++;
            he = he->flip->next;
            
        } while (he != v->he);
        
        q[v->index] = barycenter / (double)n;
    }
    
    // move vertices to new position
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        if (!v->onBoundary()) {
            Eigen::Vector3d normal = v->normal();
            v->position = q[v->index] + normal.dot(v->position - q[v->index]) * normal;
        }
    }
}

void Mesh::projectToSurface()
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
    oldVertices = vertices;
    bvh.build(this);
    
    // run algorithm
    for (int i = 0; i < iterations; i++) {
        //splitLongEdges(high2);
        //collapseShortEdges(low2, high2);
        //equalizeValences();
        //tangentialRelaxation();
        //projectToSurface();
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
