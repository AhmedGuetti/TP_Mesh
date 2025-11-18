#include "mesh_io.h"
#include <iostream>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

namespace MeshIO {

    bool loadOBJ(const std::string& filename,
        std::vector<std::array<double, 3>>& positions,
        std::vector<std::array<int, 3>>& faces) {

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
            filename.c_str(), nullptr, true);

        if (!warn.empty()) {
            std::cout << "Warning: " << warn << std::endl;
        }

        if (!err.empty()) {
            std::cerr << "Error: " << err << std::endl;
        }

        if (!ret) {
            std::cerr << "Failed to load OBJ file: " << filename << std::endl;
            return false;
        }

        // Extract vertices
        positions.clear();
        for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
            positions.push_back({
                static_cast<double>(attrib.vertices[i]),
                static_cast<double>(attrib.vertices[i + 1]),
                static_cast<double>(attrib.vertices[i + 2])
                });
        }

        // Extract faces
        faces.clear();
        for (const auto& shape : shapes) {
            for (size_t i = 0; i < shape.mesh.indices.size(); i += 3) {
                faces.push_back({
                    shape.mesh.indices[i].vertex_index,
                    shape.mesh.indices[i + 1].vertex_index,
                    shape.mesh.indices[i + 2].vertex_index
                    });
            }
        }

        std::cout << "Loaded " << positions.size() << " vertices and "
            << faces.size() << " faces from " << filename << std::endl;

        return true;
    }

}  // namespace MeshIO