#include "mesh.h"
#include "mesh_io.h"
#include "mesh_viz.h"
#include <polyscope/polyscope.h>
#include <imgui.h>
#include <iostream>
#include <polyscope/surface_mesh.h>


// Global variable for the scene 
static Mesh mesh;
static int selectedVertex = 0;
static int selectedFace = 0;
static int ringSize = 1;
static bool showBorder = false;
static bool showNormals = false;

static int lastSelectedVertex = -1;            // To detect selection changes
static float vertexEditPos[3] = {0.0f, 0.0f, 0.0f}; // Buffer for ImGui sliders


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

    ImGui::Text("Geometric Analysis:");
    if (ImGui::Button("10 iteration laplacien")) {
		// 10 iteration seems fun in the dragon lmao
        mesh.laplacianSmooth(10, 0.5);

        std::vector<std::array<double, 3>> newPositions;
		std::vector<std::array<int, 3>> faceIndices; // We need it for the function signature but we won't use it here
        mesh.getPositionsAndFaces(newPositions, faceIndices);

		// Update the polyscope visualization with the new vertex positions
        polyscope::SurfaceMesh* psMesh = polyscope::getSurfaceMesh("Dragon");
        if (psMesh) {
            psMesh->updateVertexPositions(newPositions);
        }
        // TODO: More intresting smothing operation exist where we won't lose volum seems intresting to implement if i had enought time 
    }
    ImGui::Separator();

    if (ImGui::Button("10 iteration laplacien Matrix")) {
        // 10 iteration seems fun in the dragon lmao
        mesh.MatrixLaplacianSmooth(0.5);

        std::vector<std::array<double, 3>> newPositions;
        std::vector<std::array<int, 3>> faceIndices; // We need it for the function signature but we won't use it here
        mesh.getPositionsAndFaces(newPositions, faceIndices);

        // Update the polyscope visualization with the new vertex positions
        polyscope::SurfaceMesh* psMesh = polyscope::getSurfaceMesh("Dragon");
        if (psMesh) {
            psMesh->updateVertexPositions(newPositions);
        }
        // TODO: More intresting smothing operation exist where we won't lose volum seems intresting to implement if i had enought time 
    }
    ImGui::Separator();


    // Ring neighborhood
    ImGui::Text("Ring Neighborhood:");
    ImGui::SliderInt("Ring Size", &ringSize, 1, 5);

    if (ImGui::Button("Show Vertex Ring")) {
        MeshViz::showVertexRing(mesh, selectedVertex, ringSize, "Vertex Ring");
    }

    ImGui::SameLine();


    ImGui::Text("Analyse Différentielle :");

    if (ImGui::Button("Afficher le Laplacien (Couleur)")) {
        MeshViz::showLaplacianMagnitude(mesh, "Dragon"); // Utilisez 'heMesh' (votre objet Mesh C++)
    }
    ImGui::Separator();



    if (ImGui::Button("Show Gradient in Surface")) {
        MeshViz::showVertexGradientOnMesh(mesh, "Dragon", selectedVertex, ringSize, "Surface Gradient");
    }
    ImGui::Separator();

// Vertex selection
    ImGui::Text("Vertex Selection & Editing:");
    
    if (ImGui::InputInt("Vertex ID##2", &selectedVertex)) {
        selectedVertex = std::max(0, std::min(selectedVertex, mesh.numVertices() - 1));
    }

    // If selection changed, fetch the real position from the mesh
    if (selectedVertex != lastSelectedVertex) {
        std::array<double, 3> pos = mesh.getVertexPosition(selectedVertex);
        vertexEditPos[0] = (float)pos[0];
        vertexEditPos[1] = (float)pos[1];
        vertexEditPos[2] = (float)pos[2];
        
        lastSelectedVertex = selectedVertex;
        
        // Auto-highlight when selecting a new ID
        MeshViz::highlightVertex(mesh, selectedVertex, "Selected Vertex");
    }

    bool positionChanged = ImGui::DragFloat3("Position (X Y Z)", vertexEditPos, 0.01f);

    if (ImGui::Button("Re-Highlight Vertex")) { 
        MeshViz::highlightVertex(mesh, selectedVertex, "Selected Vertex");
    }

    // If slider moved, update Mesh and Polyscope
    if (positionChanged) {
        std::array<double, 3> newPos = { 
            (double)vertexEditPos[0], 
            (double)vertexEditPos[1], 
            (double)vertexEditPos[2] 
        };
        mesh.setVertexPosition(selectedVertex, newPos);

        std::vector<std::array<double, 3>> currentPositions;
        std::vector<std::array<int, 3>> dummyFaces; 
        
        mesh.getPositionsAndFaces(currentPositions, dummyFaces);

        polyscope::SurfaceMesh* psMesh = polyscope::getSurfaceMesh("Dragon");
        if (psMesh) {
            psMesh->updateVertexPositions(currentPositions);
            MeshViz::highlightVertex(mesh, selectedVertex, "Selected Vertex");
        }
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