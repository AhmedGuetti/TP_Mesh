#include "halfedge.h"
#include "vertex.h"
#include "face.h"
#include <cmath>

HalfEdge::HalfEdge()
    : id(-1), vertex(nullptr), twin(nullptr),
    next(nullptr), prev(nullptr), face(nullptr) {
}

Vertex* HalfEdge::from() const {
    return prev ? prev->vertex : nullptr;
}

Vertex* HalfEdge::to() const {
    return vertex;
}

bool HalfEdge::isBorder() const {
    return face == nullptr;
}

void HalfEdge::vector(double& vx, double& vy, double& vz) const {
    Vertex* v0 = from();
    Vertex* v1 = to();

    if (!v0 || !v1) {
        vx = vy = vz = 0.0;
        return;
    }

    vx = v1->x - v0->x;
    vy = v1->y - v0->y;
    vz = v1->z - v0->z;
}

double HalfEdge::length() const {
    double vx, vy, vz;
    vector(vx, vy, vz);
    return std::sqrt(vx * vx + vy * vy + vz * vz);
}