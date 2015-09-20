#ifndef MESH_H
#define MESH_H

#include "Types.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "HalfEdge.h"
#include "Bvh.h"

class Mesh {
public:
    // default constructor
    Mesh();
    
    // copy constructor
    Mesh(const Mesh& mesh);
        
    // read mesh from file
    bool read(const std::string& fileName);
    
    // write mesh to file
    bool write(const std::string& fileName) const;
    
    // remesh
    void remesh(const double edgeLength, const int iterations);
    
    // member variables
    std::vector<HalfEdge> halfEdges;
    std::vector<Vertex> vertices;
    std::vector<Vertex> oldVertices;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<HalfEdgeIter> boundaries;

private:
    // center mesh about origin and rescale to unit radius
    void normalize();
    
    // splits edge and creates a vertex and two new faces
    void splitEdge(const int eIdx, const Eigen::Vector3d& position);
    
    // collapses edge and removes adjacent faces
    void collapseEdge(const int eIdx);
    
    // removes edges marked for deletion
    void resetLists();
    
    // flips edge
    void flipEdge(const int eIdx);
    
    // splits longer edges
    void splitLongEdges(const double high);
    
    // collapses shorter edges
    void collapseShortEdges(const double low, const double high);
    
    // equalizes vertex valences
    void equalizeValences();
    
    // applies an iterative smoothing filer
    void tangentialRelaxation();
    
    // maps vertex back to surface
    void projectToSurface();
    
    Bvh bvh;
};

#endif