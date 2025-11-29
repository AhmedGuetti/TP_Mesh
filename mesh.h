#pragma once

#include "forward_declarations.h"
#include "vertex.h"
#include "halfedge.h"
#include "face.h"
#include <vector>
#include <memory>
#include <string>
#include <array>
#include <Eigen/Sparse>

class Mesh {
public:
    Mesh() = default;
    ~Mesh() = default;

    // Building
    void build(const std::vector<std::array<double, 3>>& positions,
        const std::vector<std::array<int, 3>>& faces);
    void clear();

    // Accessors
    int numVertices() const { return static_cast<int>(vertices_.size()); }
    int numFaces() const { return static_cast<int>(faces_.size()); }
    int numHalfEdges() const { return static_cast<int>(halfedges_.size()); }
    int numEdges() const { return numHalfEdges() / 2; }

    Vertex* getVertex(int id) const;
    Face* getFace(int id) const;
    HalfEdge* getHalfEdge(int id) const;

    // Topology queries
    bool isClosed() const;
    int eulerCharacteristic() const;
    std::vector<HalfEdge*> getBorderHalfEdges() const;
    std::vector<Vertex*> getBorderVertices() const;

    // Visualization helpers
    void getPositionsAndFaces(std::vector<std::array<double, 3>>& positions,
        std::vector<std::array<int, 3>>& faceIndices) const;

    void laplacianSmooth(int iterations, double lambda);
    void MatrixLaplacianSmooth(double lambda);

    std::array<double, 3> getVertexPosition(int vIndex) const;
    void setVertexPosition(int vIndex, const std::array<double, 3>& newPos);

private:
    std::vector<std::unique_ptr<Vertex>> vertices_;
    std::vector<std::unique_ptr<Face>> faces_;
    std::vector<std::unique_ptr<HalfEdge>> halfedges_;

    // Helper for building
    Vertex* addVertex(double x, double y, double z);
    void linkBoundaryHalfEdges();
};