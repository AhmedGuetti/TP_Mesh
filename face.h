#pragma once

#include "forward_declarations.h"
#include <vector>
#include <array>

class Face {
public:
    int id;
    HalfEdge* he;

    // Constructor
    Face();
    ~Face() = default;

    // Query methods
    int degree() const;
    std::vector<Vertex*> vertices() const;
    std::vector<HalfEdge*> halfedges() const;

    // Geometric queries
    void normal(double& nx, double& ny, double& nz) const;
    double area() const;

    std::array<double, 3> computeNewellVector() const;
private:
    template <typename Func>
    void forEachHalfEdge(Func&& func) const;

};