#pragma once

#include <string>
#include <vector>
#include <array>

namespace MeshIO {
    // Load mesh from OBJ file
    bool loadOBJ(const std::string& filename,
        std::vector<std::array<double, 3>>& positions,
        std::vector<std::array<int, 3>>& faces);
}