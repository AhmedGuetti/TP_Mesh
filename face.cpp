#include "face.h"
#include "halfedge.h"
#include "vertex.h"
#include <cmath>

Face::Face() : id(-1), he(nullptr) {
}

int Face::degree() const {
    int count = 0;
    forEachHalfEdge([&](HalfEdge* he) {
        ++count;
        });
    return count;
}

std::vector<Vertex*> Face::vertices() const {
    std::vector<Vertex*> result;
    forEachHalfEdge([&](HalfEdge* he) {
        result.push_back(he->from());
        });
    return result;
}

std::vector<HalfEdge*> Face::halfedges() const {
    std::vector<HalfEdge*> result;
    forEachHalfEdge([&](HalfEdge* he) {
        result.push_back(he);
        });
    return result;
}

std::array<double, 3> Face::computeNewellVector() const {
    double nx = 0.0, ny = 0.0, nz = 0.0;
    if (!he) return { nx, ny, nz };

    forEachHalfEdge([&](HalfEdge* current) {
        Vertex* v1 = current->from();
        Vertex* v2 = current->to();

        if (v1 && v2) {
            nx += (v1->y - v2->y) * (v1->z + v2->z);
            ny += (v1->z - v2->z) * (v1->x + v2->x);
            nz += (v1->x - v2->x) * (v1->y + v2->y);
        }
        });

    return { nx, ny, nz };
}

void Face::normal(double& nx, double& ny, double& nz) const {
    auto nVec = computeNewellVector();
    nx = nVec[0];
    ny = nVec[1];
    nz = nVec[2];

    // Normalize
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len > 1e-10) {
        nx /= len;
        ny /= len;
        nz /= len;
    }
}

double Face::area() const {
    auto nVec = computeNewellVector();

    double len = std::sqrt(
        nVec[0] * nVec[0] +
        nVec[1] * nVec[1] +
        nVec[2] * nVec[2]
    );

    return 0.5 * len;
}

template <typename Func>
void Face::forEachHalfEdge(Func&& func) const {
    if (!he) return;

    HalfEdge* current = he;
    do {
        func(current);
        current = current->next;
    } while (current && current != he);
}
