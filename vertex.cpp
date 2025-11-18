#include "vertex.h"
#include "halfedge.h"
#include "face.h"
#include <set>
#include <queue>

Vertex::Vertex(double x, double y, double z)
    : id(-1), x(x), y(y), z(z), he(nullptr) {
}

int Vertex::degree() const {
    int count = 0;
    forEachOutgoingHalfEdge([&](HalfEdge* he) {
        ++count;
        });
    return count;
}

bool Vertex::isBorder() const {
    bool completedLoop = forEachOutgoingHalfEdge([&](HalfEdge* he) {
        });
    return !completedLoop;
}

std::vector<Vertex*> Vertex::neighbors() const {
    std::vector<Vertex*> result;
    forEachOutgoingHalfEdge([&](HalfEdge* he) {
        if (he->to()) {
            result.push_back(he->to());
        }
        });
    return result;
}

std::vector<Face*> Vertex::faces() const {
    std::vector<Face*> result;
    forEachOutgoingHalfEdge([&](HalfEdge* he) {
        if (he->face) {
            result.push_back(he->face);
        }
        });
    return result;
}

std::vector<Vertex*> Vertex::nRingNeighbors(int n) const {
    if (n <= 0) return {};

    std::set<Vertex*> visited;
    std::vector<Vertex*> currentRing;

    visited.insert(const_cast<Vertex*>(this));
    currentRing.push_back(const_cast<Vertex*>(this));

    for (int ring = 0; ring < n; ++ring) {
        std::vector<Vertex*> nextRing;

        for (Vertex* v : currentRing) {
            auto neighs = v->neighbors();
            for (Vertex* neighbor : neighs) {
                if (visited.find(neighbor) == visited.end()) {
                    nextRing.push_back(neighbor);
                    visited.insert(neighbor);
                }
            }
        }

        currentRing = std::move(nextRing);
    }

    std::vector<Vertex*> result;
    for (Vertex* v : visited) {
        if (v != this) {
            result.push_back(v);
        }
    }

    return result;
}

template <typename Func>
bool Vertex::forEachOutgoingHalfEdge(Func&& func) const{
    if (!he) return true;

    HalfEdge* current = he;
    do {
        func(current);

        if (!current->twin) return false;
        current = current->twin->next;
        if (!current) return false;

    } while (current != he);

    return true;
}