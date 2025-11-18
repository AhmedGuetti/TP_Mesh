#pragma once

#include "mesh.h"
#include <string>
#include <array>

// Forward declaration for polyscope
namespace polyscope {
    class SurfaceMesh;
}

namespace MeshViz {
    // Basic mesh visualization
    polyscope::SurfaceMesh* showMesh(const Mesh& mesh, const std::string& name = "Mesh");

    // Topology visualization
    void showBorderVertices(const Mesh& mesh, const std::string& name = "Border Vertices");
    void showBorderEdges(const Mesh& mesh, const std::string& name = "Border Edges");
    void showHalfEdges(const Mesh& mesh, const std::string& name = "HalfEdges");

    // Normals
    void showVertexNormals(const Mesh& mesh, const std::string& meshName = "Mesh");
    void showFaceNormals(const Mesh& mesh, const std::string& meshName = "Mesh");

    // Selection
    void highlightVertex(const Mesh& mesh, int vertexId,
        const std::string& name = "Selected Vertex",
        std::array<double, 3> color = { 0.0, 1.0, 0.0 }, double size = 0.02);

    void highlightFace(const Mesh& mesh, int faceId,
        const std::string& name = "Selected Face");

    void showVertexRing(const Mesh& mesh, int vertexId, int ringSize,
        const std::string& name = "Vertex Ring");

    // Visualize vertex with gradient color map
    void showVertexWithGradient(const Mesh& mesh, int vertexId,
        int numRings = 3,
        const std::string& name = "Vertex Gradient");

    void showVertexGradientOnMesh(
        const Mesh& mesh,
        const std::string& meshName, 
        int vertexId,
        int numRings,
        const std::string& quantityName
    );

    void showMeanCurvature(const Mesh& mesh, const std::string& meshName = "Mesh");


}