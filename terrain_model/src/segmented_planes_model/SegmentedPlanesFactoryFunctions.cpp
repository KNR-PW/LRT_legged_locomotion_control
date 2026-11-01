#include <terrain_model/segmented_planes_model/SegmentedPlanesFactoryFunctions.hpp>

namespace terrain_model
{

  using namespace ocs2;
  using namespace grid_map;
  using namespace convex_plane_decomposition;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  convex_plane_decomposition::PlanarTerrain computePlanarTerrain(
    GridMap&& gridMap, const PlaneDecompositionPipeline::Config& pipelineConfig,
    const std::string& elevationLayerName)
  {
    // ====== Run the perception pipeline ========
    convex_plane_decomposition::PlaneDecompositionPipeline 
      planeDecompositionPipeline(pipelineConfig);
    planeDecompositionPipeline.update(std::move(gridMap), elevationLayerName);

    return planeDecompositionPipeline.movePlanarTerrain();
  }
} // namespace terrain_model