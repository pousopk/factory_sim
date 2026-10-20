// V-HACD is one header, its code compiled here only (and without our
// warnings: it isn't our code).
#define ENABLE_VHACD_IMPLEMENTATION 1
#include "sim/convex_pieces.hpp"
#include <VHACD.h>

namespace fsim {

std::vector<std::vector<glm::vec3>> convexPieces(const std::vector<glm::vec3>& corners,
                                                 const std::vector<std::uint32_t>& triangles, int maxPieces) {
    std::vector<float> points;
    points.reserve(corners.size() * 3);
    for (const auto& p : corners) {
        points.insert(points.end(), {p.x, p.y, p.z});
    }
    VHACD::IVHACD::Parameters parameters;
    parameters.m_maxConvexHulls = static_cast<std::uint32_t>(std::max(maxPieces, 1));
    parameters.m_resolution = 100000; // voxels: fine enough for a product, quick enough to wait for
    parameters.m_maxNumVerticesPerCH = 48;
    parameters.m_asyncACD = false; // one thread: the same result every time
    VHACD::IVHACD* vhacd = VHACD::CreateVHACD();
    std::vector<std::vector<glm::vec3>> pieces;
    if (vhacd->Compute(points.data(), static_cast<std::uint32_t>(corners.size()), triangles.data(),
                       static_cast<std::uint32_t>(triangles.size() / 3), parameters)) {
        for (std::uint32_t i = 0; i < vhacd->GetNConvexHulls(); ++i) {
            VHACD::IVHACD::ConvexHull hull;
            if (vhacd->GetConvexHull(i, hull)) {
                std::vector<glm::vec3> piece;
                for (const auto& v : hull.m_points) {
                    piece.emplace_back(static_cast<float>(v.mX), static_cast<float>(v.mY), static_cast<float>(v.mZ));
                }
                pieces.push_back(std::move(piece));
            }
        }
    }
    vhacd->Release();
    return pieces;
}

} // namespace fsim
