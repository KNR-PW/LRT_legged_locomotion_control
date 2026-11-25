#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/Numerics.h>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;

    GaitStaticParameters loadGaitStaticParameters(const std::string& filename,
      const ModelSettings& modelSettings, const std::string& fieldName, bool verbose)
    {
      GaitStaticParameters settings;

      boost::property_tree::ptree pt;
      read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Static Gait Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      settings.endEffectorNumber = modelSettings.endEffectorThreeDofNames.size() + modelSettings.endEffectorSixDofNames.size();
      
      loadData::loadPtreeValue(pt, settings.initPhase, fieldName + ".initPhase", verbose);
      if(settings.initPhase < 0.0 || settings.initPhase >= 1.0)
      {
        throw std::invalid_argument("[GaitStaticParameters]: Inital phase should be between [0.0, 1.0)!");
      }

      loadData::loadPtreeValue(pt, settings.maximumSteppingFrequency, fieldName + ".maximumSteppingFrequency", verbose);
      if(settings.maximumSteppingFrequency < 0 || numerics::almost_eq(settings.maximumSteppingFrequency, 0.0, SCALAR_EPSILON))
      {
        throw std::invalid_argument("[GaitStaticParameters]: Maximum stepping frequency  smaller or equal 0!");
      }

      loadData::loadPtreeValue(pt, settings.minimumSteppingFrequency, fieldName + ".minimumSteppingFrequency", verbose);
      if(settings.minimumSteppingFrequency < 0)
      {
        throw std::invalid_argument("[GaitStaticParameters]: Minimum stepping frequency  smaller or equal 0!");
      }
      if(settings.maximumSteppingFrequency < settings.minimumSteppingFrequency || numerics::almost_eq(settings.minimumSteppingFrequency, settings.maximumSteppingFrequency, SCALAR_EPSILON))
      {
        throw std::invalid_argument("[GaitStaticParameters]: Maximum stepping frequency smaller or equal to minimum stepping frequency!");
      }

      loadData::loadPtreeValue(pt, settings.touchdownWindow, fieldName + ".touchdownWindow", verbose);
      if(settings.touchdownWindow < 0 || numerics::almost_eq(settings.touchdownWindow, 0.0, SCALAR_EPSILON))
      {
        throw std::invalid_argument("[GaitStaticParameters]: Touchdown window smaller or equal 0!");
      }

      // Optional, 0.0 (no limit) when not set. Smaller rates would not change parameters in one step
      // of MAX_PARAMETERS_STEP_DURATION more than GAIT_PARAMETERS_EPS (parameters are equal then)
      const scalar_t minimumRate = Definitions::GAIT_PARAMETERS_EPS / Definitions::MAX_PARAMETERS_STEP_DURATION;

      loadData::loadPtreeValue(pt, settings.maximumSteppingFrequencyRate,
        fieldName + ".maximumSteppingFrequencyRate", verbose);
      if(!std::isfinite(settings.maximumSteppingFrequencyRate) || settings.maximumSteppingFrequencyRate < 0.0 ||
        (settings.maximumSteppingFrequencyRate > 0.0 && settings.maximumSteppingFrequencyRate < minimumRate))
      {
        throw std::invalid_argument("[GaitStaticParameters]: Maximum stepping frequency rate has to be 0.0 (no limit) or at least "
          + std::to_string(minimumRate) + "!");
      }

      loadData::loadPtreeValue(pt, settings.maximumSwingRatioRate,
        fieldName + ".maximumSwingRatioRate", verbose);
      if(!std::isfinite(settings.maximumSwingRatioRate) || settings.maximumSwingRatioRate < 0.0 ||
        (settings.maximumSwingRatioRate > 0.0 && settings.maximumSwingRatioRate < minimumRate))
      {
        throw std::invalid_argument("[GaitStaticParameters]: Maximum swing ratio rate has to be 0.0 (no limit) or at least "
          + std::to_string(minimumRate) + "!");
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return settings;
    }

    GaitDynamicParameters loadGaitDynamicParameters(const std::string& filename,
      const ModelSettings& modelSettings, const GaitStaticParameters& staticParams, 
      const std::string& fieldName, bool verbose)
    {
      GaitDynamicParameters settings;

      boost::property_tree::ptree pt;
      read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Dynamic Gait Settings:";
        std::cerr << "\n #### =============================================================================\n";
      }

      const size_t phasesSize = modelSettings.endEffectorThreeDofNames.size() + modelSettings.endEffectorSixDofNames.size() - 1;

      loadData::loadPtreeValue(pt, settings.steppingFrequency, fieldName + ".steppingFrequency", verbose);
      if(settings.steppingFrequency < 0)
      {
        throw std::invalid_argument("[GaitDynamicParameters]: Stepping frequency  smaller or equal 0!");
      }
      if(settings.steppingFrequency < staticParams.minimumSteppingFrequency)
      {
        throw std::invalid_argument("[GaitDynamicParameters]: Stepping frequency smaller than minimum!");
      }
      if(settings.steppingFrequency > staticParams.maximumSteppingFrequency)
      {
        throw std::invalid_argument("[GaitDynamicParameters]: Stepping frequency bigger than maximum!");
      }

      loadData::loadPtreeValue(pt, settings.swingRatio, fieldName + ".swingRatio", verbose);
      if(settings.swingRatio < 0 || settings.swingRatio > 1.0)
      {
        throw std::invalid_argument("[GaitDynamicParameters]: Swing ratio should be between [0.0, 1.0)!");
      }

      loadData::loadStdVector(filename, fieldName + ".phaseOffsets", settings.phaseOffsets, verbose);
      if(settings.phaseOffsets.size() != phasesSize)
      {
        const std::string message = "[GaitDynamicParameters]: Phase offsets have size " + std::to_string(settings.phaseOffsets.size()) + ", shuld be " + std::to_string(phasesSize) + "!";
        throw std::invalid_argument(message);
      }
      size_t index = 0;
      for(const auto& phase: settings.phaseOffsets)
      {
        if(phase < 0.0 || phase > 1.0)
        {
          const std::string message = "[GaitDynamicParameters]: Phase offset in index " + std::to_string(index) + " should be between [0.0, 1.0)!";
          throw std::invalid_argument(message);
        }
        index++;
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return settings;
    }
  } // namespace locomotion
} // namespace legged_locomotion_mpc
