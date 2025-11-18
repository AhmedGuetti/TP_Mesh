#include "mesh.h"
#include "mesh_io.h"
#include "mesh_viz.h"
#include <polyscope/polyscope.h>
#include <imgui.h>
#include <iostream>


// Global variable for the scene 
static Mesh mesh;
static int selectedVertex = 0;
static int selectedFace = 0;
static int ringSize = 1;
static bool showBorder = false;
static bool showNormals = false;


void drawMainGui();


int main() {
    polyscope::init();

    // Load mesh
    std::vector<std::array<double, 3>> positions;
    std::vector<std::array<int, 3>> faces;

    if (!MeshIO::loadOBJ("Obj/Dragon/Dragon.obj", positions, faces)) {
        return EXIT_FAILURE;
    }

    // Build half-edge mesh
    mesh.build(positions, faces);

    // Visualize mesh
    MeshViz::showMesh(mesh, "Dragon");

    polyscope::state::userCallback = drawMainGui;

    polyscope::show();
    return EXIT_SUCCESS;
}


void drawMainGui() {
    ImGui::Begin("Mesh Visualization");

    // Statistics
    ImGui::Text("Mesh Statistics:");
    ImGui::Text("Vertices: %d", mesh.numVertices());
    ImGui::Text("Faces: %d", mesh.numFaces());
    ImGui::Text("Edges: %d", mesh.numEdges());
    ImGui::Text("Euler: %d", mesh.eulerCharacteristic());
    ImGui::Text("Closed: %s", mesh.isClosed() ? "Yes" : "No");

    ImGui::Separator();

    // Boundary visualization
    ImGui::Text("Boundary Visualization:");
    if (ImGui::Button("Show Border Vertices")) {
        MeshViz::showBorderVertices(mesh, "Border Vertices");
    }
    ImGui::SameLine();
    if (ImGui::Button("Show Border Edges")) {
        MeshViz::showBorderEdges(mesh, "Border Edges");
    }

    ImGui::Separator();

    // Normals
    ImGui::Text("Normal Visualization:");
    if (ImGui::Button("Show Vertex Normals")) {
        MeshViz::showVertexNormals(mesh, "Dragon");
    }
    ImGui::SameLine();
    if (ImGui::Button("Show Face Normals")) {
        MeshViz::showFaceNormals(mesh, "Dragon");
    }

    ImGui::Separator();

    // Vertex selection
    ImGui::Text("Vertex Selection:");
    ImGui::InputInt("Vertex ID", &selectedVertex);
    selectedVertex = std::max(0, std::min(selectedVertex, mesh.numVertices() - 1));

    if (ImGui::Button("Highlight Vertex")) {
        MeshViz::highlightVertex(mesh, selectedVertex, "Selected Vertex");
    }

    //if (ImGui::Button("Highlight Vertex")) {
    //    MeshViz::highlightVertex(mesh, selectedVertex, "Selected Vertex");
    //}

    ImGui::Separator();

    // Face selection
    ImGui::Text("Face Selection:");
    ImGui::InputInt("Face ID", &selectedFace);
    selectedFace = std::max(0, std::min(selectedFace, mesh.numFaces() - 1));

    if (ImGui::Button("Highlight Face")) {
        MeshViz::highlightFace(mesh, selectedFace, "Selected Face");
    }

    //ImGui::Separator();

    //ImGui::Text("Geometric Analysis:");
    //if (ImGui::Button("Show Mean Curvature (Laplacian)")) {
    //    MeshViz::showMeanCurvature(mesh, "Dragon");
    //}
    ImGui::Separator();

    // Ring neighborhood
    ImGui::Text("Ring Neighborhood:");
    ImGui::SliderInt("Ring Size", &ringSize, 1, 5);

    if (ImGui::Button("Show Vertex Ring")) {
        MeshViz::showVertexRing(mesh, selectedVertex, ringSize, "Vertex Ring");
    }

    ImGui::SameLine();


    if (ImGui::Button("Show Gradient in Surface")) {
        MeshViz::showVertexGradientOnMesh(mesh, "Dragon", selectedVertex, ringSize, "Surface Gradient");
    }

    ImGui::Separator();

    // Clear button
    if (ImGui::Button("Clear All Visualizations", ImVec2(-1, 0))) {
        polyscope::removeAllStructures();
        MeshViz::showMesh(mesh, "Dragon");
        std::cout << "Cleared all visualizations" << std::endl;
    }

    ImGui::End();
}