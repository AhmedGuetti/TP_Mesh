#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "imgui.h"
#include <vector>
#include <array>
#include <map>
#include <iostream>
#include <algorithm>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <polyscope/point_cloud.h>

// Half-Edge Data Structure
struct HalfEdge {
    int vertex;           // Target vertex of this half-edge
    int twin;             // Index of twin half-edge (opposite direction)
    int next;             // Next half-edge in the face loop
    int prev;             // Previous half-edge in the face loop
    int face;             // Face this half-edge belongs to

    HalfEdge() : vertex(-1), twin(-1), next(-1), prev(-1), face(-1) {}
};

struct HalfEdgeMesh {
    std::vector<std::array<double, 3>> vertices;
    std::vector<HalfEdge> halfedges;
    std::vector<int> vertex_halfedge;  // One outgoing half-edge per vertex
    std::vector<int> face_halfedge;    // One half-edge per face

    void buildFromFaces(const std::vector<std::array<double, 3>>& verts,
        const std::vector<std::array<int, 3>>& faces);

    void printStatistics() const;
    std::vector<int> getVertexOutgoingHalfEdges(int vertexIdx) const;
    std::vector<int> getFaceHalfEdges(int faceIdx) const;
};

void HalfEdgeMesh::buildFromFaces(const std::vector<std::array<double, 3>>& verts,
    const std::vector<std::array<int, 3>>& faces) {
    vertices = verts;
    vertex_halfedge.resize(vertices.size(), -1);
    face_halfedge.resize(faces.size(), -1);

    // Map to find twin half-edges: (v0, v1) -> halfedge_index
    std::map<std::pair<int, int>, int> edgeMap;

    // Create half-edges for each face
    for (size_t faceIdx = 0; faceIdx < faces.size(); ++faceIdx) {
        const auto& face = faces[faceIdx];
        int he_start = halfedges.size();

        // Create 3 half-edges for this triangle
        for (int i = 0; i < 3; ++i) {
            HalfEdge he;
            he.vertex = face[(i + 1) % 3];  // Target vertex
            he.face = faceIdx;
            halfedges.push_back(he);
        }

        // Set next/prev pointers for this face
        for (int i = 0; i < 3; ++i) {
            int he_idx = he_start + i;
            halfedges[he_idx].next = he_start + (i + 1) % 3;
            halfedges[he_idx].prev = he_start + (i + 2) % 3;
        }

        // Store one half-edge for this face
        face_halfedge[faceIdx] = he_start;

        // Register half-edges in edge map and set vertex half-edges
        for (int i = 0; i < 3; ++i) {
            int he_idx = he_start + i;
            int v0 = face[i];
            int v1 = face[(i + 1) % 3];

            // Store one outgoing half-edge per vertex
            if (vertex_halfedge[v0] == -1) {
                vertex_halfedge[v0] = he_idx;
            }

            // Register in edge map for twin finding
            edgeMap[{v0, v1}] = he_idx;
        }
    }

    // Find twin half-edges
    for (size_t he_idx = 0; he_idx < halfedges.size(); ++he_idx) {
        int v1 = halfedges[he_idx].vertex;
        int v0 = halfedges[halfedges[he_idx].prev].vertex;

        auto it = edgeMap.find({ v1, v0 });
        if (it != edgeMap.end()) {
            halfedges[he_idx].twin = it->second;
        }
    }

    printStatistics();
}

void HalfEdgeMesh::printStatistics() const {
    int boundaryEdges = 0;
    int interiorEdges = 0;

    for (const auto& he : halfedges) {
        if (he.twin == -1) {
            boundaryEdges++;
        }
        else {
            interiorEdges++;
        }
    }

    std::cout << "\n=== Half-Edge Mesh Statistics ===" << std::endl;
    std::cout << "Vertices: " << vertices.size() << std::endl;
    std::cout << "Faces: " << face_halfedge.size() << std::endl;
    std::cout << "Half-Edges: " << halfedges.size() << std::endl;
    std::cout << "Interior Half-Edges (with twin): " << interiorEdges << std::endl;
    std::cout << "Boundary Half-Edges (no twin): " << boundaryEdges << std::endl;
}

std::vector<int> HalfEdgeMesh::getVertexOutgoingHalfEdges(int vertexIdx) const {
    std::vector<int> outgoing;
    if (vertexIdx < 0 || vertexIdx >= (int)vertex_halfedge.size()) return outgoing;

    int start_he = vertex_halfedge[vertexIdx];
    if (start_he == -1) return outgoing;

    int current_he = start_he;
    int iterations = 0;
    const int max_iterations = halfedges.size();

    do {
        outgoing.push_back(current_he);

        // Move to twin, then next
        int twin = halfedges[current_he].twin;
        if (twin == -1) break;  // Boundary edge

        current_he = halfedges[twin].next;
        iterations++;

        if (iterations > max_iterations) break;
    } while (current_he != start_he);

    return outgoing;
}

std::vector<int> HalfEdgeMesh::getFaceHalfEdges(int faceIdx) const {
    std::vector<int> faceHEs;
    if (faceIdx < 0 || faceIdx >= (int)face_halfedge.size()) return faceHEs;

    int start_he = face_halfedge[faceIdx];
    if (start_he == -1) return faceHEs;

    int current_he = start_he;
    do {
        faceHEs.push_back(current_he);
        current_he = halfedges[current_he].next;
    } while (current_he != start_he && faceHEs.size() < 10);

    return faceHEs;
}

// Visualization helper functions
void visualizeHalfEdges(const HalfEdgeMesh& mesh) {
    // Create curve network showing each half-edge as a directed edge
    std::vector<std::array<double, 3>> heNodes;
    std::vector<std::array<size_t, 2>> heEdges;
    std::vector<std::array<double, 3>> heColors;

    for (size_t he_idx = 0; he_idx < mesh.halfedges.size(); ++he_idx) {
        const auto& he = mesh.halfedges[he_idx];

        // Get source vertex (from previous half-edge)
        int v_start_idx = mesh.halfedges[he.prev].vertex;
        int v_end_idx = he.vertex;

        const auto& v_start = mesh.vertices[v_start_idx];
        const auto& v_end = mesh.vertices[v_end_idx];

        // Create edge that's slightly offset to show direction
        // We'll shorten it to 80% to show arrow-like effect
        std::array<double, 3> direction = {
            v_end[0] - v_start[0],
            v_end[1] - v_start[1],
            v_end[2] - v_start[2]
        };

        std::array<double, 3> end_point = {
            v_start[0] + direction[0] * 0.85,
            v_start[1] + direction[1] * 0.85,
            v_start[2] + direction[2] * 0.85
        };

        size_t start_idx = heNodes.size();
        heNodes.push_back(v_start);
        heNodes.push_back(end_point);
        heEdges.push_back({ start_idx, start_idx + 1 });

        // Color based on whether it has a twin
        if (he.twin == -1) {
            heColors.push_back({ 1.0, 0.0, 0.0 }); // Red for boundary
        }
        else {
            heColors.push_back({ 0.0, 0.5, 1.0 }); // Blue for interior
        }
    }

    if (!heNodes.empty()) {
        auto* heNet = polyscope::registerCurveNetwork("Half-Edge Structure", heNodes, heEdges);
        heNet->addEdgeColorQuantity("Edge Type", heColors)->setEnabled(true);
        heNet->setRadius(0.0015);
        heNet->setEnabled(false);
    }
}

void visualizeTwinConnections(const HalfEdgeMesh& mesh) {
    // Create curve network showing twin connections
    std::vector<std::array<double, 3>> twinNodes;
    std::vector<std::array<size_t, 2>> twinEdges;

    std::map<int, size_t> heToNode;

    for (size_t he_idx = 0; he_idx < mesh.halfedges.size(); ++he_idx) {
        const auto& he = mesh.halfedges[he_idx];
        if (he.twin == -1 || he.twin <= (int)he_idx) continue;  // Skip boundary and duplicates

        // Calculate midpoint of this half-edge
        int v_start_idx = mesh.halfedges[he.prev].vertex;
        int v_end_idx = he.vertex;

        const auto& v_start = mesh.vertices[v_start_idx];
        const auto& v_end = mesh.vertices[v_end_idx];

        std::array<double, 3> midpoint = {
            (v_start[0] + v_end[0]) / 2.0,
            (v_start[1] + v_end[1]) / 2.0,
            (v_start[2] + v_end[2]) / 2.0
        };

        size_t node_idx = twinNodes.size();
        heToNode[he_idx] = node_idx;
        twinNodes.push_back(midpoint);
    }

    // Create edges between twin half-edge midpoints
    for (size_t he_idx = 0; he_idx < mesh.halfedges.size(); ++he_idx) {
        const auto& he = mesh.halfedges[he_idx];
        if (he.twin == -1 || he.twin <= (int)he_idx) continue;

        if (heToNode.count(he_idx) && heToNode.count(he.twin)) {
            twinEdges.push_back({ heToNode[he_idx], heToNode[he.twin] });
        }
    }

    if (!twinNodes.empty()) {
        auto* curveNet = polyscope::registerCurveNetwork("Twin Connections", twinNodes, twinEdges);
        curveNet->setEnabled(false);
        curveNet->setRadius(0.001);
    }
}

void visualizeBoundaryEdges(const HalfEdgeMesh& mesh) {
    std::vector<std::array<double, 3>> boundaryNodes;
    std::vector<std::array<size_t, 2>> boundaryEdges;

    for (size_t he_idx = 0; he_idx < mesh.halfedges.size(); ++he_idx) {
        const auto& he = mesh.halfedges[he_idx];
        if (he.twin != -1) continue;  // Only boundary edges

        int v_start_idx = mesh.halfedges[he.prev].vertex;
        int v_end_idx = he.vertex;

        size_t start_node = boundaryNodes.size();
        boundaryNodes.push_back(mesh.vertices[v_start_idx]);
        boundaryNodes.push_back(mesh.vertices[v_end_idx]);

        boundaryEdges.push_back({ start_node, start_node + 1 });
    }

    if (!boundaryNodes.empty()) {
        auto* boundaryNet = polyscope::registerCurveNetwork("Boundary Edges", boundaryNodes, boundaryEdges);
        boundaryNet->setColor({ 1.0, 0.0, 0.0 });
        boundaryNet->setRadius(0.003);
        boundaryNet->setEnabled(false);
    }
}

void visualizeVertexValence(const HalfEdgeMesh& mesh, polyscope::SurfaceMesh* psMesh) {
    std::vector<double> valences(mesh.vertices.size(), 0.0);

    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        auto outgoing = mesh.getVertexOutgoingHalfEdges(v);
        valences[v] = static_cast<double>(outgoing.size());
    }

    auto valenceQ = psMesh->addVertexScalarQuantity("Vertex Valence", valences);
    valenceQ->setColorMap("viridis");
    valenceQ->setEnabled(false);
}

void visualizeFaceAreas(const HalfEdgeMesh& mesh, polyscope::SurfaceMesh* psMesh) {
    std::vector<double> faceAreas;

    for (size_t f = 0; f < mesh.face_halfedge.size(); ++f) {
        auto faceHEs = mesh.getFaceHalfEdges(f);
        if (faceHEs.size() != 3) {
            faceAreas.push_back(0.0);
            continue;
        }

        // Get the three vertices
        int v0 = mesh.halfedges[mesh.halfedges[faceHEs[0]].prev].vertex;
        int v1 = mesh.halfedges[mesh.halfedges[faceHEs[1]].prev].vertex;
        int v2 = mesh.halfedges[mesh.halfedges[faceHEs[2]].prev].vertex;

        const auto& p0 = mesh.vertices[v0];
        const auto& p1 = mesh.vertices[v1];
        const auto& p2 = mesh.vertices[v2];

        // Calculate area using cross product
        std::array<double, 3> v01 = { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
        std::array<double, 3> v02 = { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };

        std::array<double, 3> cross = {
            v01[1] * v02[2] - v01[2] * v02[1],
            v01[2] * v02[0] - v01[0] * v02[2],
            v01[0] * v02[1] - v01[1] * v02[0]
        };

        double area = 0.5 * std::sqrt(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
        faceAreas.push_back(area);
    }

    auto areaQ = psMesh->addFaceScalarQuantity("Face Areas", faceAreas);
    areaQ->setColorMap("blues");
    areaQ->setEnabled(false);
}

// Function to highlight a specific vertex
void highlightVertex(int vertexIdx, const HalfEdgeMesh& mesh, polyscope::SurfaceMesh* psMesh,
    std::array<double, 3> color = { 1.0, 0.0, 0.0 }) {

    if (vertexIdx < 0 || vertexIdx >= (int)mesh.vertices.size()) {
        std::cout << "Invalid vertex index: " << vertexIdx << std::endl;
        return;
    }

    // Create a scalar quantity that highlights the selected vertex
    std::vector<double> vertexHighlight(mesh.vertices.size(), 0.0);
    vertexHighlight[vertexIdx] = 1.0;

    auto highlightQ = psMesh->addVertexScalarQuantity("Selected Vertex", vertexHighlight);
    highlightQ->setEnabled(true);
    highlightQ->setColorMap("reds");

    // Create a point cloud for the selected vertex to make it bigger
    std::vector<std::array<double, 3>> selectedVertexPos = { mesh.vertices[vertexIdx] };
    auto* pointCloud = polyscope::registerPointCloud("Highlighted Vertex", selectedVertexPos);
    pointCloud->setPointColor({ color[0], color[1], color[2] });
    pointCloud->setPointRadius(0.015);  // Make it bigger
    pointCloud->setEnabled(true);

    std::cout << "Highlighted vertex " << vertexIdx << " at position ("
        << mesh.vertices[vertexIdx][0] << ", "
        << mesh.vertices[vertexIdx][1] << ", "
        << mesh.vertices[vertexIdx][2] << ")" << std::endl;
}

// Function to highlight multiple vertices
void highlightVertices(const std::vector<int>& vertexIndices, const HalfEdgeMesh& mesh,
    polyscope::SurfaceMesh* psMesh, const std::string& name = "Selected Vertices") {

    // Create scalar quantity for all vertices
    std::vector<double> vertexHighlight(mesh.vertices.size(), 0.0);

    std::vector<std::array<double, 3>> selectedPositions;
    for (int idx : vertexIndices) {
        if (idx >= 0 && idx < (int)mesh.vertices.size()) {
            vertexHighlight[idx] = 1.0;
            selectedPositions.push_back(mesh.vertices[idx]);
        }
    }

    if (selectedPositions.empty()) {
        std::cout << "No valid vertices to highlight" << std::endl;
        return;
    }

    auto highlightQ = psMesh->addVertexScalarQuantity(name, vertexHighlight);
    highlightQ->setEnabled(true);
    highlightQ->setColorMap("reds");

    // Create point cloud for selected vertices
    auto* pointCloud = polyscope::registerPointCloud(name + " Points", selectedPositions);
    pointCloud->setPointColor({ 1.0, 0.0, 0.0 });
    pointCloud->setPointRadius(0.012);
    pointCloud->setEnabled(true);

    std::cout << "Highlighted " << selectedPositions.size() << " vertices" << std::endl;
}

// Function to highlight vertex and its one-ring neighborhood
void highlightVertexWithNeighborhood(int vertexIdx, const HalfEdgeMesh& mesh, polyscope::SurfaceMesh* psMesh) {

    if (vertexIdx < 0 || vertexIdx >= (int)mesh.vertices.size()) {
        std::cout << "Invalid vertex index: " << vertexIdx << std::endl;
        return;
    }

    // Get one-ring neighbors
    auto outgoingHEs = mesh.getVertexOutgoingHalfEdges(vertexIdx);
    std::vector<int> neighbors;

    for (int he_idx : outgoingHEs) {
        neighbors.push_back(mesh.halfedges[he_idx].vertex);
    }

    // Create scalar quantity: 2.0 for center, 1.0 for neighbors, 0.0 for others
    std::vector<double> neighborhoodHighlight(mesh.vertices.size(), 0.0);
    neighborhoodHighlight[vertexIdx] = 2.0;
    for (int n : neighbors) {
        neighborhoodHighlight[n] = 1.0;
    }

    auto highlightQ = psMesh->addVertexScalarQuantity("Vertex Neighborhood", neighborhoodHighlight);
    highlightQ->setEnabled(true);
    highlightQ->setColorMap("viridis");

    // Create point clouds
    std::vector<std::array<double, 3>> centerPos = { mesh.vertices[vertexIdx] };
    auto* centerCloud = polyscope::registerPointCloud("Center Vertex", centerPos);
    centerCloud->setPointColor({ 1.0, 0.0, 0.0 });
    centerCloud->setPointRadius(0.018);
    centerCloud->setEnabled(true);

    std::vector<std::array<double, 3>> neighborPos;
    for (int n : neighbors) {
        neighborPos.push_back(mesh.vertices[n]);
    }

    if (!neighborPos.empty()) {
        auto* neighborCloud = polyscope::registerPointCloud("Neighbor Vertices", neighborPos);
        neighborCloud->setPointColor({ 0.0, 1.0, 0.0 });
        neighborCloud->setPointRadius(0.012);
        neighborCloud->setEnabled(true);
    }

    std::cout << "Highlighted vertex " << vertexIdx << " with " << neighbors.size()
        << " neighbors" << std::endl;
}

int main() {
    polyscope::init();

    // Load OBJ using tinyobjloader
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    std::string filename = "Obj/Dragon/Dragon.obj";
    std::string basedir = "";

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
        filename.c_str(), basedir.c_str(), true);

    if (!warn.empty()) {
        std::cout << "Warning: " << warn << std::endl;
    }

    if (!err.empty()) {
        std::cerr << "Error: " << err << std::endl;
    }

    if (!ret) {
        std::cerr << "Failed to load OBJ file: " << filename << std::endl;
        return 1;
    }

    // Convert to polyscope format
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<int, 3>> faces;

    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        vertices.push_back({ attrib.vertices[i],
                           attrib.vertices[i + 1],
                           attrib.vertices[i + 2] });
    }

    for (auto& shape : shapes) {
        for (size_t i = 0; i < shape.mesh.indices.size(); i += 3) {
            faces.push_back({ shape.mesh.indices[i].vertex_index,
                            shape.mesh.indices[i + 1].vertex_index,
                            shape.mesh.indices[i + 2].vertex_index });
        }
    }

    std::cout << "Loaded mesh with " << vertices.size() << " vertices and "
        << faces.size() << " faces" << std::endl;

    // Build half-edge data structure
    HalfEdgeMesh heMesh;
    heMesh.buildFromFaces(vertices, faces);

    // Register the mesh in polyscope
    auto* psMesh = polyscope::registerSurfaceMesh("Dragon Mesh", vertices, faces);

    // Create all visualizations
    std::cout << "\nCreating visualizations..." << std::endl;
    visualizeHalfEdges(heMesh);
    visualizeTwinConnections(heMesh);
    visualizeBoundaryEdges(heMesh);
    visualizeVertexValence(heMesh, psMesh);
    visualizeFaceAreas(heMesh, psMesh);

    std::cout << "\n=== Available Visualizations ===" << std::endl;
    std::cout << "1. Half-Edge Structure - Shows all half-edges with direction (blue=interior, red=boundary)" << std::endl;
    std::cout << "2. Twin Connections - Shows which half-edges are twins (connecting edge midpoints)" << std::endl;
    std::cout << "3. Boundary Edges - Highlights edges with no twin (red curves)" << std::endl;
    std::cout << "4. Vertex Valence - Number of edges connected to each vertex (color-coded)" << std::endl;
    std::cout << "5. Face Areas - Area of each triangle face (color-coded)" << std::endl;
    std::cout << "\nEnable them in the Polyscope GUI by clicking on each structure!" << std::endl;

    // Add interactive vertex selection
    static int selectedVertex = 0;
    static bool showNeighborhood = false;

    polyscope::state::userCallback = [&]() {
        ImGui::Begin("Vertex Highlighter");

        ImGui::Text("Total vertices: %d", (int)heMesh.vertices.size());
        ImGui::Separator();

        ImGui::InputInt("Vertex Index", &selectedVertex);

        if (selectedVertex < 0) selectedVertex = 0;
        if (selectedVertex >= (int)heMesh.vertices.size()) {
            selectedVertex = (int)heMesh.vertices.size() - 1;
        }

        ImGui::Checkbox("Show Neighborhood", &showNeighborhood);

        if (ImGui::Button("Highlight Vertex")) {
            // Remove previous highlights
            polyscope::removePointCloud("Highlighted Vertex", false);
            polyscope::removePointCloud("Center Vertex", false);
            polyscope::removePointCloud("Neighbor Vertices", false);

            if (showNeighborhood) {
                highlightVertexWithNeighborhood(selectedVertex, heMesh, psMesh);
            }
            else {
                highlightVertex(selectedVertex, heMesh, psMesh);
            }
        }

        ImGui::SameLine();

        if (ImGui::Button("Clear Highlights")) {
            polyscope::removePointCloud("Highlighted Vertex", false);
            polyscope::removePointCloud("Selected Vertices Points", false);
            polyscope::removePointCloud("Center Vertex", false);
            polyscope::removePointCloud("Neighbor Vertices", false);
        }

        ImGui::Separator();
        ImGui::Text("Instructions:");
        ImGui::BulletText("Enter vertex index (0-%d)", (int)heMesh.vertices.size() - 1);
        ImGui::BulletText("Click 'Highlight Vertex' to show");
        ImGui::BulletText("Enable 'Show Neighborhood' for 1-ring");

        ImGui::End();
        };

    polyscope::show();
    return 0;
}
