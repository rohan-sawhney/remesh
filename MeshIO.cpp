#include "MeshIO.h"
#include "Mesh.h"
#include <set>
#include <map>

class Index {
public:
    Index() {}
    
    Index(int v, int vt, int vn): position(v), uv(vt), normal(vn) {}
    
    bool operator<(const Index& i) const {
        if (position < i.position) return true;
        if (position > i.position) return false;
        if (uv < i.uv) return true;
        if (uv > i.uv) return false;
        if (normal < i.normal) return true;
        if (normal > i.normal) return false;
        
        return false;
    }
    
    int position;
    int uv;
    int normal;
};

class MeshData {
public:
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> uvs;
    std::vector<Eigen::Vector3d> normals;
    std::vector<std::vector<Index>> indices;
};

Index parseFaceIndex(const std::string& token)
{
    std::stringstream in(token);
    std::string indexString;
    int indices[3] = {-1, -1, -1};
    
    int i = 0;
    while(getline(in,indexString,'/')) {
        if (indexString != "\\") {
            std::stringstream ss(indexString);
            ss >> indices[i++];
        }
    }
    
    // decrement since indices in OBJ files are 1-based
    return Index(indices[0]-1,
                 indices[1]-1,
                 indices[2]-1);
}

std::string stringRep(const Eigen::Vector3d& v)
{
    return std::to_string(v.x()) + " " + std::to_string(v.y()) + " " + std::to_string(v.z());
}

void MeshIO::preallocateMeshElements(const MeshData& data, Mesh& mesh)
{
    // count the number of edges
    std::set<std::pair<int,int>> edges;
    for (std::vector<std::vector<Index>>::const_iterator f  = data.indices.begin();
                                                         f != data.indices.end();
                                                         f ++) {
        for (unsigned int I = 0; I < f->size(); I++) {
            int J = (I+1) % f->size();
            int i = (*f)[I].position;
            int j = (*f)[J].position;
            
            if (i > j) std::swap(i, j);
            
            edges.insert(std::pair<int,int>(i, j));
        }
    }
    
    size_t nV = data.positions.size();
    size_t nE = edges.size();
    size_t nF = data.indices.size();
    size_t nHE = 2*nE;
    size_t chi = nV - nE + nF;
    int nB = std::max(0, 2 - (int)chi); // conservative approximation of number of boundary cycles
    
    mesh.halfEdges.clear();
    mesh.vertices.clear();
    mesh.edges.clear();
    mesh.faces.clear();
    mesh.boundaries.clear();
    
    // assigning extra space for edge split operation in remeshing
    mesh.halfEdges.reserve(3*nHE);
    mesh.vertices.reserve(3*nV);
    mesh.edges.reserve(3*nE);
    mesh.faces.reserve(3*(nF + nB));
}

void MeshIO::indexElements(Mesh& mesh)
{
    int index = 0;
    for (VertexIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        v->index = index;
        index ++;
    }
    
    index = 0;
    for (EdgeIter e = mesh.edges.begin(); e != mesh.edges.end(); e++) {
        e->index = index;
        index ++;
    }
    
    index = 0;
    for (HalfEdgeIter he = mesh.halfEdges.begin(); he != mesh.halfEdges.end(); he++) {
        he->index = index;
        index ++;
    }
    
    index = 0;
    for (FaceIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        f->index = index;
        index ++;
    }
}

void MeshIO::checkIsolatedVertices(const Mesh& mesh)
{
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        if (v->isIsolated()) {
            std::cerr << "Warning: vertex " << v->index
                      << " is isolated (not contained in any face)."
                      << std::endl;
        }
    }
}

void MeshIO::checkNonManifoldVertices(const Mesh& mesh)
{
    std::unordered_map<std::string, int> vertexFaceMap;
    
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        HalfEdgeCIter he = f->he;
        do {
            vertexFaceMap[stringRep(he->vertex->position)] ++;
            he = he->next;
            
        } while (he != f->he);
    }
    
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        int valence = 0;
        HalfEdgeCIter he = v->he;
        do {
            valence ++;
            he = he->flip->next;
            
        } while (he != v->he);
        
        if (vertexFaceMap[stringRep(v->position)] != valence) {
            std::cerr << "Warning: vertex " << v->index
                      << " is nonmanifold." << std::endl;
        }
    }
}

extern std::vector<HalfEdge> isolated;

bool MeshIO::buildMesh(const MeshData& data, Mesh& mesh)
{
    std::map<std::pair<int, int>, int> edgeCount;
    std::map<std::pair<int, int>, HalfEdgeIter> existingHalfEdges;
    std::map<int, VertexIter> indexToVertex;
    std::map<HalfEdgeIter, bool> hasFlipEdge;
    
    preallocateMeshElements(data, mesh);
    
    // insert vertices into mesh and map vertex indices to vertex pointers
    for (unsigned int i = 0; i < data.positions.size(); i++) {
        VertexIter vertex = mesh.vertices.insert(mesh.vertices.end(), Vertex());
        vertex->position = data.positions[i];
        vertex->he = isolated.begin();
        indexToVertex[i] = vertex;
    }
    
    // insert faces into mesh
    int faceIndex = 0;
    bool degenerateFaces = false;
    for (std::vector<std::vector<Index>>::const_iterator f  = data.indices.begin();
                                                         f != data.indices.end();
                                                         f ++) {
        int n = (int)f->size();
        
        // check if face is degenerate
        if (n < 3) {
            std::cerr << "Error: face " << faceIndex << " is degenerate" << std::endl;
            degenerateFaces = true;
            continue;
        }
        
        // create face
        FaceIter newFace = mesh.faces.insert(mesh.faces.end(), Face());
        
        // create a halfedge for each edge of the face
        std::vector<HalfEdgeIter> halfEdges(n);
        for (int i = 0; i < n; i++) {
            halfEdges[i] = mesh.halfEdges.insert(mesh.halfEdges.end(), HalfEdge());
        }
        
        // initialize the halfedges
        for (int i = 0; i < n; i++) {
            // vertex indices
            int a = (*f)[i].position;
            int b = (*f)[(i+1)%n].position;
            
            // set halfedge attributes
            halfEdges[i]->next = halfEdges[(i+1)%n];
            halfEdges[i]->vertex = indexToVertex[a];
            
            halfEdges[i]->onBoundary = false;
            
            // keep track of which halfedges have flip edges defined (for deteting boundaries)
            hasFlipEdge[halfEdges[i]] = false;
            
            // point vertex a at the current halfedge
            indexToVertex[a]->he = halfEdges[i];
            
            // point new face and halfedge to each other
            halfEdges[i]->face = newFace;
            newFace->he = halfEdges[i];
            
            // if an edge between a and b has been created in the past, it is the flip edge of the current halfedge
            if (a > b) std::swap(a, b);
            if (existingHalfEdges.find(std::pair<int, int>(a, b)) != existingHalfEdges.end()) {
                halfEdges[i]->flip = existingHalfEdges[std::pair<int, int>(a, b)];
                halfEdges[i]->flip->flip = halfEdges[i];
                halfEdges[i]->edge = halfEdges[i]->flip->edge;
                hasFlipEdge[halfEdges[i]] = true;
                hasFlipEdge[halfEdges[i]->flip] = true;
                
            } else {
                // create an edge and set its halfedge
                halfEdges[i]->edge = mesh.edges.insert(mesh.edges.end(), Edge());
                halfEdges[i]->edge->he = halfEdges[i];
                edgeCount[std::pair<int, int>(a, b)] = 0;
            }
            
            // record that halfedge has been created from a to b
            existingHalfEdges[std::pair<int, int>(a, b)] = halfEdges[i];
            
            // check for nonmanifold edges
            edgeCount[std::pair<int, int>(a, b)] ++;
            if (edgeCount[std::pair<int, int>(a, b)] > 2) {
                std::cerr << "Error: edge " << a << ", " << b << " is non manifold" << std::endl;
                return false;
            }
        }
        
        faceIndex++;
    }
    
    if (degenerateFaces) {
        return false;
    }
    
    // insert extra faces for boundary cycle
    for (HalfEdgeIter currHe = mesh.halfEdges.begin(); currHe != mesh.halfEdges.end(); currHe++) {
        // if a halfedge with no flip edge is found, create a new face and link it the corresponding boundary cycle
        if (!hasFlipEdge[currHe]) {
            // create face
            FaceIter newFace = mesh.faces.insert(mesh.faces.end(), Face());
            
            // walk along boundary cycle
            std::vector<HalfEdgeIter> boundaryCycle;
            HalfEdgeIter he = currHe;
            do {
                // create a new halfedge on the boundary face
                HalfEdgeIter newHe = mesh.halfEdges.insert(mesh.halfEdges.end(), HalfEdge());
                newHe->onBoundary = true;
                
                // link the current halfedge in the cycle to its new flip edge
                he->flip = newHe;
                
                // grab the next halfedge along the boundary by finding
                // the next halfedge around the current vertex that doesn't
                // have a flip edge defined
                HalfEdgeIter nextHe = he->next;
                while (hasFlipEdge[nextHe]) {
                    nextHe = nextHe->flip->next;
                }
                
                // set attritubes for new halfedge
                newHe->flip = he;
                newHe->vertex = nextHe->vertex;
                newHe->edge = he->edge;
                newHe->face = newFace;
                
                // set face's halfedge to boundary halfedge
                newFace->he = newHe;
                
                boundaryCycle.push_back(newHe);
                
                // continue walk along cycle
                he = nextHe;
                
            } while (he != currHe);
            
            // link the cycle of boundary halfedges together
            int n = (int)boundaryCycle.size();
            for (int i = 0; i < n; i++) {
                boundaryCycle[i]->next = boundaryCycle[(i+n-1)%n];
                hasFlipEdge[boundaryCycle[i]] = true;
                hasFlipEdge[boundaryCycle[i]->flip] = true;
            }
            mesh.boundaries.insert(mesh.boundaries.end(), boundaryCycle[0]);
        }
    }
    
    indexElements(mesh);
    checkIsolatedVertices(mesh);
    checkNonManifoldVertices(mesh);
    
    return true;
}

bool MeshIO::read(std::ifstream& in, Mesh& mesh)
{
    MeshData data;
    
    // parse obj format
    std::string line;
    while(getline(in, line)) {
        std::stringstream ss(line);
        std::string token;
        
        ss >> token;
        
        if (token == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            
            data.positions.push_back(Eigen::Vector3d(x, y, z));
            
        } else if (token == "vt") {
            double u, v;
            ss >> u >> v;
            
            data.uvs.push_back(Eigen::Vector3d(u,v,0));
            
        } else if (token == "vn") {
            double x, y, z;
            ss >> x >> y >> z;
            
            data.normals.push_back(Eigen::Vector3d(x, y, z));
            
        } else if (token == "f") {
            std::vector<Index> faceIndices;
            
            while (ss >> token) {
                Index index = parseFaceIndex(token);
                if (index.position < 0) {
                    getline(in, line);
                    size_t i = line.find_first_not_of("\t\n\v\f\r ");
                    index = parseFaceIndex(line.substr(i));
                }
                
                faceIndices.push_back(index);
            }
            
            data.indices.push_back(faceIndices);
        }
    }
    
    return buildMesh(data, mesh);
}

void MeshIO::write(std::ofstream& out, const Mesh& mesh)
{
    std::unordered_map<std::string, int> vertexMap;
    std::unordered_map<std::string, int> uvMap;
    std::unordered_map<std::string, int> normalMap;
    
    // write vertices
    int index = 1;
    for (VertexCIter v = mesh.vertices.begin(); v != mesh.vertices.end(); v++) {
        out << "v " << v->position.x() << " "
                    << v->position.y() << " "
                    << v->position.z() << std::endl;
        
        vertexMap[stringRep(v->position)] = index;
        index++;
    }
    
    // write faces
    index = 0;
    for (FaceCIter f = mesh.faces.begin(); f != mesh.faces.end(); f++) {
        HalfEdgeIter he = mesh.faces[index].he;
        
        if (he->onBoundary) {
            continue;
        }
        
        out << "f ";
        int j = 0;
        do {
            out << vertexMap[stringRep(he->vertex->position)] << " ";
            j++;
            
            he = he->next;
        } while (he != mesh.faces[index].he);
        
        out << std::endl;
        index ++;
    }
}