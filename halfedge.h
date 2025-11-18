#pragma once

#include "forward_declarations.h"

class HalfEdge {
public:
    int id;
    Vertex* vertex;  

    // Navigation pointers
    HalfEdge* twin;
    HalfEdge* next;
    HalfEdge* prev;
    Face* face;

    // Constructor
    HalfEdge();

    // Query methods
    Vertex* from() const;
    Vertex* to() const;
    bool isBorder() const;

    // Geometric queries
    void vector(double& vx, double& vy, double& vz) const;
    double length() const;

    // Destructor
    ~HalfEdge() = default;
};