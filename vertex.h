#pragma once

#include "forward_declarations.h"

#include <vector>

class Vertex {
public:
	int id;
	double x, y, z;
	HalfEdge* he;

	// Constructor
	Vertex(double x = 0.0, double y = 0.0, double z = 0.0);
	~Vertex() = default;


	// Query methods
	int degree() const;
	bool isBorder() const;

	// Neighborhood queries
	std::vector<Vertex*> neighbors() const;
	std::vector<Face*> faces() const;
	std::vector<Vertex*> nRingNeighbors(int n) const;



private:
	template <typename Func>
	bool forEachOutgoingHalfEdge(Func&& func) const;
};