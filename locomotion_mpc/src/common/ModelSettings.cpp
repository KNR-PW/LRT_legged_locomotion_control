#include <legged_locomotion_mpc/common/ModelSettings.hpp>

#include <unordered_set>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  ModelSettings loadModelSettings(const std::string& filename,
    const std::string& fieldName,
    bool verbose)
  {
    ModelSettings modelSettings;

    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Common Model Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, modelSettings.algorithm, fieldName + ".algorithm", verbose);
    if(modelSettings.algorithm != "SQP" && modelSettings.algorithm != "DDP")
    {
      std::string message = "[ModelSettings]: Algorithm can be from set {SQP, DDP}!";
      throw std::invalid_argument(message);
    }

    loadData::loadPtreeValue(pt, modelSettings.robotName, fieldName + ".robotName", verbose);

    loadData::loadPtreeValue(pt, modelSettings.verbose, fieldName + ".verbose", verbose);

    loadData::loadPtreeValue(pt, modelSettings.recompileLibrariesCppAd, fieldName + ".recompileLibrariesCppAd", verbose);

    loadData::loadPtreeValue(pt, modelSettings.modelFolderCppAd, fieldName + ".modelFolderCppAd", verbose);

    loadData::loadPtreeValue(pt, modelSettings.worldLinkName, fieldName + ".worldLinkName", verbose);
    
    loadData::loadPtreeValue(pt, modelSettings.baseLinkName, fieldName + ".baseLinkName", verbose);

    loadData::loadStdVector(filename, fieldName + ".endEffectorThreeDofNames", modelSettings.endEffectorThreeDofNames, verbose);

    loadData::loadStdVector(filename, fieldName + ".endEffectorSixDofNames", modelSettings.endEffectorSixDofNames, verbose);

    std::unordered_set<std::string> namesSet;

    std::vector<std::string> namesVector = modelSettings.endEffectorThreeDofNames;
    namesVector.insert(namesVector.end(), 
        modelSettings.endEffectorSixDofNames.begin(), modelSettings.endEffectorSixDofNames.end());

    for(const auto& endEffectorName: namesVector)
    {
      if(namesSet.find(endEffectorName) == namesSet.cend())
      {
        namesSet.emplace(endEffectorName);
      }
      else
      {
        std::string message = "[ModelSettings]: End effector " +  endEffectorName + " already used!";
        throw std::invalid_argument(message);
      }
    }

    loadData::loadStdVector(filename, fieldName + ".hipFrameNames", modelSettings.hipFrameNames, verbose);
    if(modelSettings.hipFrameNames.size() != namesSet.size())
    {
      std::string message = "[ModelSettings]: Hip frame vector has wrong size!";
      throw std::invalid_argument(message);
    }
    for(const auto& hipFrameName: modelSettings.hipFrameNames)
    {
      if(namesSet.find(hipFrameName) == namesSet.cend())
      {
        namesSet.emplace(hipFrameName);
      }
      else
      {
        std::string message = "[ModelSettings]: Hip frame " +  hipFrameName + " already used!";
        throw std::invalid_argument(message);
      }
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return modelSettings;
  }
}
