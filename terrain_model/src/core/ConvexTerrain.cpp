
#include <terrain_model/core/ConvexTerrain.hpp>

namespace terrain_model
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain::ConvexTerrain(): plane_(), boundary_() {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ConvexTerrain::ConvexTerrain(TerrainPlane plane, std::vector<vector2_t> boundary):
    plane_(std::move(plane)), boundary_(std::move(boundary)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const TerrainPlane& ConvexTerrain::getTerrainPlane() const
  {
    return plane_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
	const std::vector<vector2_t>& ConvexTerrain::getBoundaryPoints() const
  {
    return boundary_;
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
	const vector2_t& ConvexTerrain::getBoundaryPoint(size_t currentPointIndex) const
  {
  	return boundary_[currentPointIndex % boundary_.size()]; // wrap around
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
	const vector2_t& ConvexTerrain::getNextBoundaryPoint(size_t currentPointIndex) const
  {
  	return boundary_[(currentPointIndex + 1) % boundary_.size()]; // next point with wrap around
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::pair<scalar_t, vector2_t> ConvexTerrain::projectToConvex2dPolygonBoundary(
		const vector2_t& queryPoint) const
  {
    vector2_t image = queryPoint;
    scalar_t dist2 = std::numeric_limits<scalar_t>::max();
    auto saveIfCloser = [&queryPoint, &dist2, &image](const vector2_t& q)
    {
        const scalar_t newDist2 = (queryPoint - q).squaredNorm();
        if (newDist2 < dist2)
        {
            dist2 = newDist2;
            image = q;
        }
    };
    // Point is inside of convex polygon if it is on the same side of all edges,
    // this works for clockwise and counter-clockwise order
    bool hasPointOnLeftSide = false;
    bool hasPointOnRightSide = false;
    for (size_t i = 0; i < boundary_.size(); i++)
    {
        const auto& p1 = getBoundaryPoint(i);
        const auto& p2 = getNextBoundaryPoint(i);
        const vector2_t p12 = p2 - p1;
        const vector2_t p1Query = queryPoint - p1;
        const scalar_t edgeSquaredNorm = p12.squaredNorm();
        if (edgeSquaredNorm < std::numeric_limits<scalar_t>::epsilon())
        {
            // Degenerated edge, only its point can be the closest one
            saveIfCloser(p1);
            continue;
        }

        const scalar_t r = p12.dot(p1Query) / edgeSquaredNorm;
        if (r > 1.0)
        {
            saveIfCloser(p2);
        }
        else if (r < 0.0)
        {
            saveIfCloser(p1);
        }
        else
        {
            const vector2_t q = p1 + r * p12;
            saveIfCloser(q);
        }

        const scalar_t crossProduct = p12.x() * p1Query.y() - p12.y() * p1Query.x();
        if (crossProduct > 0.0)
        {
            hasPointOnLeftSide = true;
        }
        else if (crossProduct < 0.0)
        {
            hasPointOnRightSide = true;
        }
    } // end of i loop
    const bool isInside = !(hasPointOnLeftSide && hasPointOnRightSide);
    return {(isInside ? -dist2 : dist2), image};
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector3_t ConvexTerrain::projectToConvex3dPolygon(const vector3_t& queryPoint) const
  {
    const vector3_t local_p = plane_.getPositionInTerrainFrameFromPositionInWorld(queryPoint);
    const vector2_t local_2d_p(local_p.x(), local_p.y());
    const auto distance2ImagePair = projectToConvex2dPolygonBoundary(local_2d_p);
    vector3_t local_q;
    if (distance2ImagePair.first <= 0.0)
    {
        // the 2d local point is inside polygon
        local_q << local_2d_p, 0.0;
    }
    else
    {
        // the 2d local point is outside polygon
        local_q << distance2ImagePair.second, 0.0;
    }
    return plane_.getPositionInWorldFrameFromPositionInTerrain(local_q);
  }
} // namespace terrain_model