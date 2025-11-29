#include "mesh_viz.h"
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>
#include <glm/glm.hpp>
#include <unordered_map>
#include <queue>
#include <iostream>


static std::unordered_map<Vertex*, int> computeRingDistances(
    const Vertex* center, int numRings)
{
    std::unordered_map<Vertex*, int> ringDistance;
    if (!center || numRings <= 0) {
        return ringDistance;
    }

    std::queue<Vertex*> queue;

    // We must cast away 'const' to push onto the queue,
    // as v->neighbors() returns non-const Vertex*.
    Vertex* startNode = const_cast<Vertex*>(center);

    ringDistance[startNode] = 0; // The cente of the ring or the current veretex is always going to have a 0 distance 
    queue.push(startNode);

    while (!queue.empty()) {
        Vertex* current = queue.front();
        queue.pop();

        int currentDist = ringDistance[current];
        if (currentDist >= numRings) continue;

        // we get all the neighbors of current that could be in the ring 2 or 3 not the center in any case 
        auto neighbors = current->neighbors();

        for (Vertex* neighbor : neighbors) {
            if (ringDistance.find(neighbor) == ringDistance.end()) {
                ringDistance[neighbor] = currentDist + 1;
                queue.push(neighbor);
            }
        }
    }

    return ringDistance;
}

namespace MeshViz {

    polyscope::SurfaceMesh* showMesh(const Mesh& mesh, const std::string& name) {
        std::vector<std::array<double, 3>> positions;
        std::vector<std::array<int, 3>> faces;

        mesh.getPositionsAndFaces(positions, faces);

        auto* psMesh = polyscope::registerSurfaceMesh(name, positions, faces);

        std::cout << "Registered mesh '" << name << "' with "
            << positions.size() << " vertices and "
            << faces.size() << " faces" << std::endl;

        return psMesh;
    }

    void showBorderVertices(const Mesh& mesh, const std::string& name) {
        auto borderVerts = mesh.getBorderVertices();

        if (borderVerts.empty()) {
            std::cout << "No border vertices - mesh is closed" << std::endl;
            return;
        }

        std::vector<std::array<double, 3>> positions;
        for (Vertex* v : borderVerts) {
            positions.push_back({ v->x, v->y, v->z });
        }

        auto* cloud = polyscope::registerPointCloud(name, positions);
        cloud->setPointRadius(0.001);
        cloud->setPointColor({ 1.0, 0.0, 0.0 });

        std::cout << "Showing " << borderVerts.size() << " border vertices" << std::endl;
    }

    void showBorderEdges(const Mesh& mesh, const std::string& name) {
        auto borderHEs = mesh.getBorderHalfEdges();

        if (borderHEs.empty()) {
            std::cout << "No border edges - mesh is closed" << std::endl;
            return;
        }

        std::vector<std::array<double, 3>> nodes;
        std::vector<std::array<int, 2>> edges;
        std::unordered_map<Vertex*, int> vertexMap;

        for (HalfEdge* he : borderHEs) {
            Vertex* v1 = he->from();
            Vertex* v2 = he->to();

            if (!v1 || !v2) continue;

            // Add vertices if not already added
            if (vertexMap.find(v1) == vertexMap.end()) {
                vertexMap[v1] = static_cast<int>(nodes.size());
                nodes.push_back({ v1->x, v1->y, v1->z });
            }
            if (vertexMap.find(v2) == vertexMap.end()) {
                vertexMap[v2] = static_cast<int>(nodes.size());
                nodes.push_back({ v2->x, v2->y, v2->z });
            }

            edges.push_back({ vertexMap[v1], vertexMap[v2] });
        }

        auto* network = polyscope::registerCurveNetwork(name, nodes, edges);
        network->setColor({ 1.0, 0.0, 0.0 });
        network->setRadius(0.005);

        std::cout << "Showing " << borderHEs.size() << " border edges" << std::endl;
    }

    void showHalfEdges(const Mesh& mesh, const std::string& name) {
        std::vector<std::array<double, 3>> nodes;
        std::vector<std::array<int, 2>> edges;
        std::unordered_map<Vertex*, int> vertexMap;

        for (int i = 0; i < mesh.numHalfEdges(); ++i) {
            HalfEdge* he = mesh.getHalfEdge(i);
            if (!he) continue;

            Vertex* v1 = he->from();
            Vertex* v2 = he->to();

            if (!v1 || !v2) continue;

            if (vertexMap.find(v1) == vertexMap.end()) {
                vertexMap[v1] = static_cast<int>(nodes.size());
                nodes.push_back({ v1->x, v1->y, v1->z });
            }
            if (vertexMap.find(v2) == vertexMap.end()) {
                vertexMap[v2] = static_cast<int>(nodes.size());
                nodes.push_back({ v2->x, v2->y, v2->z });
            }

            edges.push_back({ vertexMap[v1], vertexMap[v2] });
        }

        auto* network = polyscope::registerCurveNetwork(name, nodes, edges);
        network->setRadius(0.002);

        std::cout << "Showing " << mesh.numHalfEdges() << " halfedges" << std::endl;
    }

    void showVertexNormals(const Mesh& mesh, const std::string& meshName) {
        auto* psMesh = polyscope::getSurfaceMesh(meshName);
        if (!psMesh) {
            std::cerr << "Mesh '" << meshName << "' not found" << std::endl;
            return;
        }

        std::vector<glm::vec3> normals;

        for (int i = 0; i < mesh.numVertices(); ++i) {
            Vertex* v = mesh.getVertex(i);
            if (!v) continue;

            auto faces = v->faces();
            if (faces.empty()) {
                normals.push_back(glm::vec3(0.0)); // a hanging vertex
                continue;
            }

            double nx = 0.0, ny = 0.0, nz = 0.0;
            for (Face* f : faces) {
                double fnx, fny, fnz;
                f->normal(fnx, fny, fnz);
                nx += fnx;
                ny += fny;
                nz += fnz;
            }

            double len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 1e-10) {
                nx /= len;
                ny /= len;
                nz /= len;
            }

            normals.push_back(glm::vec3(nx, ny, nz));
        }

        auto* q = psMesh->addVertexVectorQuantity("Vertex Normals", normals);
        q->setEnabled(true);
        q->setVectorLengthScale(0.001);
        q->setVectorRadius(0.002);
        q->setVectorColor(glm::vec3(0.0, 0.8, 1.0));

        std::cout << "Added vertex normals" << std::endl;
    }

    void showFaceNormals(const Mesh& mesh, const std::string& meshName) {
        auto* psMesh = polyscope::getSurfaceMesh(meshName);
        if (!psMesh) {
            std::cerr << "Mesh '" << meshName << "' not found" << std::endl;
            return;
        }

        std::vector<glm::vec3> normals;

        for (int i = 0; i < mesh.numFaces(); ++i) {
            Face* f = mesh.getFace(i);
            if (!f) continue;

            double nx, ny, nz;
            f->normal(nx, ny, nz);
            normals.push_back(glm::vec3(nx, ny, nz));
        }

        auto* q = psMesh->addFaceVectorQuantity("Face Normals", normals);
        q->setEnabled(true);
        q->setVectorLengthScale(0.001);
        q->setVectorRadius(0.002);
        q->setVectorColor(glm::vec3(1.0, 0.5, 0.0));

        std::cout << "Added face normals" << std::endl;
    }

    void highlightVertex(const Mesh& mesh, int vertexId,
        const std::string& name,
        std::array<double, 3> color, double size) {
        Vertex* v = mesh.getVertex(vertexId);
        if (!v) {
            std::cerr << "Vertex " << vertexId << " not found" << std::endl;
            return;
        }
        // Show vertex
        std::vector<std::array<double, 3>> pos = { {v->x, v->y, v->z} };
        auto* cloud = polyscope::registerPointCloud(name, pos);
        cloud->setPointRadius(0.002f);
        cloud->setPointColor({ color[0], color[1], color[2] });

        // Show neighbors
        auto neighbors = v->neighbors();
        if (!neighbors.empty()) {
            std::vector<std::array<double, 3>> neighborPos;
            for (Vertex* n : neighbors) {
                neighborPos.push_back({ n->x, n->y, n->z });
            }

            auto* neighCloud = polyscope::registerPointCloud(name + " Neighbors", neighborPos);
            //neighCloud->setPointRadius((9 * size) / 10);
            neighCloud->setPointRadius(0.001f);

            neighCloud->setPointColor({ 0.0, 0.5, 1.0 });
        }

        std::cout << "Highlighted vertex " << vertexId << " with "
            << neighbors.size() << " neighbors" << std::endl;
    }


    // This function also was fixed by Ai Where it took me too msuch time to found that the error was Z-fighting when rendering a Surface exactly on the older one without distruction the other one and doing so is impossible change topology, well he just added a small offset
    void highlightFace(const Mesh& mesh, int faceId, const std::string& name) {
        Face* f = mesh.getFace(faceId);
        if (!f) {
            std::cerr << "Face " << faceId << " not found" << std::endl;
            return;
        }

        auto verts = f->vertices();
        if (verts.size() != 3) return;

        // --- FIX: Get face normal to apply a small offset ---
        double nx, ny, nz;
        f->normal(nx, ny, nz);
        // This small offset prevents Z-fighting (flickering)
        const double offset = 0.001;

        std::vector<std::array<double, 3>> positions;
        for (Vertex* v : verts) {
            // --- FIX: Apply offset to each vertex position ---
            positions.push_back({ v->x + nx * offset,
                                  v->y + ny * offset,
                                  v->z + nz * offset });
        }

        std::vector<std::array<int, 3>> triangle = { {0, 1, 2} };

        auto* faceMesh = polyscope::registerSurfaceMesh(name, positions, triangle);
        faceMesh->setSurfaceColor(glm::vec3(1.0, 1.0, 0.0));
        faceMesh->setEdgeWidth(1.5);

        std::cout << "Highlighted face " << faceId << std::endl;
    }

    void showVertexRing(const Mesh& mesh, int vertexId, int ringSize,
        const std::string& name) {
        Vertex* v = mesh.getVertex(vertexId);
        if (!v) {
            std::cerr << "Vertex " << vertexId << " not found" << std::endl;
            return;
        }

        if (ringSize <= 0) {
            std::cerr << "Ring size must be positive" << std::endl;
            return;
        }

        auto ringNeighbors = v->nRingNeighbors(ringSize);

        if (ringNeighbors.empty()) {
            std::cout << "No " << ringSize << "-ring neighbors found" << std::endl;
            return;
        }

        // Show center
        std::vector<std::array<double, 3>> centerPos = { {v->x, v->y, v->z} };
        auto* centerCloud = polyscope::registerPointCloud(name + " Center", centerPos);
        centerCloud->setPointRadius(0.001);
        centerCloud->setPointColor(glm::vec3(0.0, 1.0, 0.0));

        // Show ring
        std::vector<std::array<double, 3>> ringPos;
        for (Vertex* neighbor : ringNeighbors) {
            ringPos.push_back({ neighbor->x, neighbor->y, neighbor->z });
        }

        auto* ringCloud = polyscope::registerPointCloud(name + " Neighbors", ringPos);
        ringCloud->setPointRadius(0.001);
        ringCloud->setPointColor(glm::vec3(0.0, 0.5, 1.0));

        std::cout << "Showing " << ringSize << "-ring of vertex " << vertexId
            << " (" << ringNeighbors.size() << " neighbors)" << std::endl;
    }

    glm::vec3 getColorGradient(float t) {
        glm::vec3 color;
        if (t < 0.25f) {
            float localT = t / 0.25f;
            color = glm::mix(glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.5f, 0.0f), localT);
        }
        else if (t < 0.5f) {
            float localT = (t - 0.25f) / 0.25f;
            color = glm::mix(glm::vec3(1.0f, 0.5f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), localT);
        }
        else if (t < 0.75f) {
            float localT = (t - 0.5f) / 0.25f;
            color = glm::mix(glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(0.0f, 1.0f, 1.0f), localT);
        }
        else {
            float localT = (t - 0.75f) / 0.25f;
            color = glm::mix(glm::vec3(0.0f, 1.0f, 1.0f), glm::vec3(0.0f, 0.0f, 1.0f), localT);
        }

        return color;
    }

    // Thanks to Gemini i have foound how to do it !!!
    void showVertexGradientOnMesh(const Mesh& mesh, const std::string& meshName,
        int vertexId, int numRings,
        const std::string& quantityName)
    {
        Vertex* center = mesh.getVertex(vertexId);

        std::unordered_map<Vertex*, int> ringDistance =
            computeRingDistances(center, numRings);

        if (ringDistance.empty()) {
            std::cerr << "Vertex " << vertexId << " not found or ring size <= 0" << std::endl;
            return;
        }

        auto* psMesh = polyscope::getSurfaceMesh(meshName);
        if (!psMesh) {
            std::cerr << "Mesh '" << meshName << "' not found. Cannot add gradient." << std::endl;
            return;
        }

        std::vector<glm::vec3> vertexColors;
        vertexColors.reserve(mesh.numVertices());

        glm::vec3 defaultColor(0.8f, 0.8f, 0.8f); // A light gray

        for (int i = 0; i < mesh.numVertices(); ++i) {
            Vertex* v = mesh.getVertex(i);
            if (!v) {
                vertexColors.push_back(defaultColor);
                continue;
            }

            auto it = ringDistance.find(v);

            if (it == ringDistance.end()) {
                vertexColors.push_back(defaultColor);
            }
            else {
                int dist = it->second;
                float t = static_cast<float>(dist) / static_cast<float>(numRings);

                glm::vec3 color;
                color = getColorGradient(t);
                vertexColors.push_back(color);
            }
        }

        psMesh->addVertexColorQuantity(quantityName, vertexColors)->setEnabled(true);
        std::cout << "Added '" << quantityName << "' to mesh '" << meshName << "'" << std::endl;
        polyscope::removePointCloud("Vertex Gradient", false);
    }

    void MeshViz::showLaplacianMagnitude(const Mesh& mesh, const std::string& meshName) {
        auto* psMesh = polyscope::getSurfaceMesh(meshName);
        if (!psMesh) {
            std::cerr << "Erreur : Le maillage " << meshName << " n'est pas enregistré dans Polyscope." << std::endl;
            return;
        }

        std::vector<double> laplacianMagnitudes;

        for (int i = 0; i < mesh.numVertices(); ++i) {
            Vertex* v = mesh.getVertex(i);
            auto neighbors = v->neighbors();

            if (neighbors.empty()) {
                laplacianMagnitudes.push_back(0.0);
                continue;
            }

            // Calculate the barycentre of all neighbors
            double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
            for (Vertex* n : neighbors) {
                sumX += n->x;
                sumY += n->y;
                sumZ += n->z;
            }

            double avgX = sumX / neighbors.size();
            double avgY = sumY / neighbors.size();
            double avgZ = sumZ / neighbors.size();

            // L = Moyenne - Position actuelle
            double Lx = avgX - v->x;
            double Ly = avgY - v->y;
            double Lz = avgZ - v->z;

            // Norme of vector
            double magnitude = std::sqrt(Lx * Lx + Ly * Ly + Lz * Lz);

            laplacianMagnitudes.push_back(magnitude);
        }

        auto* q = psMesh->addVertexScalarQuantity("Laplacian Magnitude (Curvature)", laplacianMagnitudes);
        q->setEnabled(true); // Afficher immédiatement
        q->setColorMap("jet"); // "jet" ou "viridis" sont bons pour voir les différences
    }

}  // namespace MeshViz