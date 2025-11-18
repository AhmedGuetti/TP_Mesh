#include "mesh.h"
#include <unordered_map>
#include <iostream>
#include <algorithm>

// A helper structure map of veritcie and half edges
struct EdgeKey {
    int v1, v2;

    EdgeKey(int a, int b) : v1(a), v2(b) {}

    bool operator==(const EdgeKey& other) const {
        return v1 == other.v1 && v2 == other.v2;
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& k) const {
        return std::hash<int>()(k.v1) ^ (std::hash<int>()(k.v2) << 1);
    }
};


// A Function to add a vertex into the mesh
Vertex* Mesh::addVertex(double x, double y, double z) {
    auto v = std::make_unique<Vertex>(x, y, z);
    v->id = static_cast<int>(vertices_.size());
    Vertex* ptr = v.get();
    vertices_.push_back(std::move(v));
    return ptr;
}


// This function is going to be responsible of building the mesh by creating the half edges data structure and construct the Edge map
void Mesh::build(const std::vector<std::array<double, 3>>& positions,
    const std::vector<std::array<int, 3>>& faceIndices) {
    clear();

    // Create vertices from a list of positions
    for (const auto& pos : positions) {
        addVertex(pos[0], pos[1], pos[2]);
    }

    // Map for finding twin halfedges: (v_from, v_to) -> halfedge
    std::unordered_map<EdgeKey, HalfEdge*, EdgeKeyHash> edgeMap;




    for (const auto& faceVerts : faceIndices) {
        // Validate vertex indices
        for (int vIdx : faceVerts) {
            if (vIdx < 0 || vIdx >= numVertices()) {
                std::cerr << "Error: Invalid vertex index " << vIdx << std::endl;
                continue;
            }
        }

        // Create the face, we use make_unique to create a auto managed pointer not more memory pain
        auto f = std::make_unique<Face>();
        f->id = static_cast<int>(faces_.size());
        Face* facePtr = f.get();
        faces_.push_back(std::move(f));

        // Create halfedges for this face (3 for triangles)
        std::vector<HalfEdge*> faceHalfEdges;
        for (int i = 0; i < 3; ++i) {
            auto he = std::make_unique<HalfEdge>();
            he->id = static_cast<int>(halfedges_.size());
            he->vertex = vertices_[faceVerts[(i + 1) % 3]].get();  // Target vertex
            he->face = facePtr;

            HalfEdge* hePtr = he.get();
            faceHalfEdges.push_back(hePtr);
            halfedges_.push_back(std::move(he));
        }

        // Link next and prev pointers tot those halfedges
        for (int i = 0; i < 3; ++i) {
            faceHalfEdges[i]->next = faceHalfEdges[(i + 1) % 3];
            faceHalfEdges[i]->prev = faceHalfEdges[(i + 2) % 3];
        }

        // Set face's halfedge
        facePtr->he = faceHalfEdges[0];


        // Store in edge map and set vertex halfedges, thanks to AI this part wasn't that hard at the end
        for (int i = 0; i < 3; ++i) {
            int v_from = faceVerts[i];
            int v_to = faceVerts[(i + 1) % 3];

            HalfEdge* he = faceHalfEdges[i];

            // Set vertex outgoing halfedge (if not already set)
            if (!vertices_[v_from]->he) {
                vertices_[v_from]->he = he;
            }

            // Store directed edge in map
            EdgeKey key(v_from, v_to);
            edgeMap[key] = he;
        }
    }

    // Link twin halfedges
    for (const auto& pair : edgeMap) {
        const EdgeKey& key = pair.first;
        HalfEdge* he = pair.second;

        // Look for opposite edge
        EdgeKey oppositeKey(key.v2, key.v1);
        auto it = edgeMap.find(oppositeKey);

        if (it != edgeMap.end()) {
            HalfEdge* oppositeHe = it->second;
            he->twin = oppositeHe;
            oppositeHe->twin = he;
        }
    }

    // Create boundary halfedges for edges without twins
    // Yet for our Mesh this is nor necessery because we have a closed mesh but i thinks it's nice to have it.
    linkBoundaryHalfEdges();

    std::cout << "Mesh built: " << numVertices() << " vertices, "
        << numFaces() << " faces, " << numEdges() << " edges, "
        << numHalfEdges() << " halfedges" << std::endl;
}

void Mesh::linkBoundaryHalfEdges() {
    std::vector<HalfEdge*> boundaryHEs;

    // Create boundary halfedges for unmatched edges
    for (auto& he : halfedges_) {
        if (!he->twin) {
            auto boundaryHe = std::make_unique<HalfEdge>();
            boundaryHe->id = static_cast<int>(halfedges_.size());
            boundaryHe->vertex = he->from();  // Reverse direction
            boundaryHe->face = nullptr;       // Boundary no face linked to it 
            boundaryHe->twin = he.get();
            he->twin = boundaryHe.get();

            HalfEdge* boundaryPtr = boundaryHe.get();
            boundaryHEs.push_back(boundaryPtr);
            halfedges_.push_back(std::move(boundaryHe));
        }
    }

    // Link boundary halfedges in loops
    // For each boundary halfedge, find its next boundary halfedge it's all linked clockwise
    for (HalfEdge* he : boundaryHEs) {
        if (he->next) continue;  // Already linked

        // The next boundary halfedge starts where this one ends
        Vertex* targetVertex = he->to();
        if (!targetVertex) continue;

        // Find outgoing boundary halfedge from target vertex
        HalfEdge* current = targetVertex->he;
        if (!current) continue;

        // Search around the vertex for a boundary halfedge
        do {
            if (current->isBorder()) {
                he->next = current;
                current->prev = he;
                break;
            }

            // Move to next outgoing halfedge
            if (current->twin && current->twin->next) {
                current = current->twin->next;
            }
            else {
                break;
            }
        } while (current != targetVertex->he);
    }
}

void Mesh::clear() {
    vertices_.clear();
    faces_.clear();
    halfedges_.clear();
}

Vertex* Mesh::getVertex(int id) const {
    if (id < 0 || id >= numVertices()) return nullptr;
    return vertices_[id].get();
}

Face* Mesh::getFace(int id) const {
    if (id < 0 || id >= numFaces()) return nullptr;
    return faces_[id].get();
}

HalfEdge* Mesh::getHalfEdge(int id) const {
    if (id < 0 || id >= numHalfEdges()) return nullptr;
    return halfedges_[id].get();
}

bool Mesh::isClosed() const {
    return getBorderHalfEdges().empty();
}

int Mesh::eulerCharacteristic() const {
    return numVertices() - numEdges() + numFaces();
}

std::vector<HalfEdge*> Mesh::getBorderHalfEdges() const {
    std::vector<HalfEdge*> result;
    for (const auto& he : halfedges_) {
        if (he->isBorder()) {
            result.push_back(he.get());
        }
    }
    return result;
}

std::vector<Vertex*> Mesh::getBorderVertices() const {
    std::vector<Vertex*> result;
    for (const auto& v : vertices_) {
        if (v->isBorder()) {
            result.push_back(v.get());
        }
    }
    return result;
}

void Mesh::getPositionsAndFaces(std::vector<std::array<double, 3>>& positions,
    std::vector<std::array<int, 3>>& faceIndices) const {
    positions.clear();
    faceIndices.clear();

    // Get positions
    for (const auto& v : vertices_) {
        positions.push_back({ v->x, v->y, v->z });
    }

    // Get face indices
    for (const auto& f : faces_) {
        auto verts = f->vertices();
        if (verts.size() == 3) {
            faceIndices.push_back({ verts[0]->id, verts[1]->id, verts[2]->id });
        }
    }
}

//void Mesh::laplacianSmooth(int iterations, double lambda) {
//    for (size_t i = 0; i < iterations; i++)
//    {
//        for (const auto& v : vertices_) {
//            if (v->isBorder()) {
//                continue; // Skip boundary vertices even if we don't have them in this mesh :D
//            }
//            auto neighbors = v->neighbors();
//
//            // Calculate centroid of neighbors
//            double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
//            for (const auto& n : neighbors) {
//                sumX += n->x;
//                sumY += n->y;
//                sumZ += n->z;
//            }
//            double avgX = sumX / neighbors.size();
//            double avgY = sumY / neighbors.size();
//            double avgZ = sumZ / neighbors.size();
//
//            // Apply the new position directly to the mesh 
//            v->x += lambda * (avgX - v->x);
//            v->y += lambda * (avgY - v->y);
//            v->z += lambda * (avgZ - v->z);
//        }
//    }
// }



// Inside mesh.cpp

void Mesh::laplacianSmooth(int iterations, double lambda) {
    // Temporary storage for new positions to ensure simultaneous update
    std::vector<std::array<double, 3>> newPositions;
    newPositions.reserve(numVertices());

    for (int iter = 0; iter < iterations; ++iter) {
        newPositions.clear();

        // Step 1: Calculate new positions for all vertices
        for (const auto& v : vertices_) {
            // Optional: Pin boundary vertices to prevent the mesh from shrinking at the edges
            // if (v->isBorder()) {
            //     newPositions.push_back({v->x, v->y, v->z});
            //     continue;
            // }

            auto neighbors = v->neighbors();

            if (neighbors.empty()) {
                newPositions.push_back({ v->x, v->y, v->z });
                continue;
            }

            // Calculate Centroid (Average position of neighbors)
            double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
            for (const auto& n : neighbors) {
                sumX += n->x;
                sumY += n->y;
                sumZ += n->z;
            }

            double avgX = sumX / neighbors.size();
            double avgY = sumY / neighbors.size();
            double avgZ = sumZ / neighbors.size();

            // Apply Laplacian update rule: v' = v + lambda * (average - v)
            double newX = v->x + lambda * (avgX - v->x);
            double newY = v->y + lambda * (avgY - v->y);
            double newZ = v->z + lambda * (avgZ - v->z);

            newPositions.push_back({ newX, newY, newZ });
        }

        // Step 2: Apply the calculated positions to the mesh
        for (int i = 0; i < numVertices(); ++i) {
            vertices_[i]->x = newPositions[i][0];
            vertices_[i]->y = newPositions[i][1];
            vertices_[i]->z = newPositions[i][2];
        }
    }

    std::cout << "Applied Laplacian smoothing (" << iterations << " iterations)" << std::endl;
}