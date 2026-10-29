
#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>

#include <stdexcept>

#include <Eigen/LU>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace terrain_model
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane computeTerrainPlane(const vector3_t& position, 
    const vector3_t& eulerAngles, ocs2::scalar_t height)
  {
    const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(eulerAngles);
    const vector3_t planePosition = position - rotationMatrix.col(2) * height; // normal vector in world times height

    return TerrainPlane(planePosition, rotationMatrix.transpose());
  }
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TerrainPlane computeTerrainPlane(const std::vector<vector3_t>& positions)
  {
    size_t pointNum = positions.size();
    matrix3_t rotationMatrix;
    vector3_t meanPosition = vector3_t::Zero();
    const scalar_t tolerance = 1e-9;

    if(pointNum < 3)
    {
      throw std::invalid_argument("At least 3 points are needed for creating plane using "
        "fit method");
    }
    else if (pointNum == 3)
    {

      vector3_t vectorOnPlane1 = positions[1] - positions[0];
      vector3_t vectorOnPlane2 = positions[2] - positions[0];

      vector3_t normalToPlane = (vectorOnPlane1).cross(vectorOnPlane2);
      if(!(normalToPlane.norm() > tolerance))
      {
        throw std::invalid_argument("Points for creating plane can not be collinear");
      }

      for(const auto& position: positions)
      {
        meanPosition += position;
      }
      meanPosition /= 3;

      normalToPlane.normalize();
      vectorOnPlane1.normalize();

      /** X axis is vectorOnPlane1, 
       *  Y axis will be  normalToPlane x vectorOnPlane1
       *  Z asix is normalToPlane */

      rotationMatrix.col(1) = normalToPlane.cross(vectorOnPlane1);

      if(normalToPlane.z() < 0)
      {
        rotationMatrix.col(0) = -vectorOnPlane1;
        rotationMatrix.col(2) = -normalToPlane;
      }
      else
      {
        rotationMatrix.col(0) = vectorOnPlane1;
        rotationMatrix.col(2) = normalToPlane;
      }
    }
    else
    {
      for(const auto& position: positions)
      {
        meanPosition += position;
      }
      meanPosition /= pointNum;

      // Least squares fit of z = a * x + b * y + c, in coordinates centered at the mean position
      // it is a fixed size 2x2 problem for slopes a and b, the fitted plane goes through the mean
      Eigen::Matrix<scalar_t, 2, 2> covarianceXY = Eigen::Matrix<scalar_t, 2, 2>::Zero();
      vector2_t covarianceXYZ = vector2_t::Zero();
      for(const auto& position: positions)
      {
        const vector3_t centeredPosition = position - meanPosition;
        covarianceXY.noalias() += centeredPosition.head<2>() * centeredPosition.head<2>().transpose();
        covarianceXYZ += centeredPosition.head<2>() * centeredPosition.z();
      }

      const scalar_t traceXY = covarianceXY.trace();
      if(!(covarianceXY.determinant() > tolerance * traceXY * traceXY))
      {
        throw std::invalid_argument("Points for creating plane can not be collinear in XY plane");
      }
      const vector2_t slopes = covarianceXY.inverse() * covarianceXYZ;
      const vector3_t result{slopes.x(), slopes.y(),
        meanPosition.z() - slopes.dot(meanPosition.head<2>())};
      vector3_t normalToPlane{-result.x(), -result.y(), 1};

      normalToPlane.normalize();

      /** Get random vector on plane from plane equation */
      vector3_t pointOnPlane1{positions[0].x(), positions[0].y(),
        result.x() * positions[0].x() + result.y() * positions[0].y() + result.z()};

      vector3_t pointOnPlane2{positions[1].x(), positions[1].y(),
        result.x() * positions[1].x() + result.y() * positions[1].y() + result.z()};

      vector3_t vectorOnPlane = pointOnPlane1 - pointOnPlane2;
      if(vectorOnPlane.norm() > tolerance)
      {
        vectorOnPlane.normalize();
      }
      else
      {
        // First two points have the same XY position, take any vector on plane
        vectorOnPlane = TerrainPlane::getTangentialBasisFromSurfaceNormal(normalToPlane).row(0).transpose();
      }

      rotationMatrix.col(0) = vectorOnPlane;
      rotationMatrix.col(1) = normalToPlane.cross(vectorOnPlane);
      rotationMatrix.col(2) = normalToPlane;
    }
    return TerrainPlane(meanPosition, rotationMatrix.transpose());
  }
} // namespace terrain_model
