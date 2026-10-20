#pragma once
#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

namespace fsim {

// Splits a closed triangle mesh (its corners, and three indices per
// triangle) into at most `maxPieces` convex pieces that together follow its
// shape, with V-HACD: it fills the mesh with small cubes (voxels), then
// keeps cutting the filled volume where it's least convex. Returns each
// piece's corners (empty if it fails). Runs on the calling thread only, so
// the same mesh always gives the same pieces.
std::vector<std::vector<glm::vec3>> convexPieces(const std::vector<glm::vec3>& corners,
                                                 const std::vector<std::uint32_t>& triangles, int maxPieces);

} // namespace fsim
