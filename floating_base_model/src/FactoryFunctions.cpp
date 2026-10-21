#include "floating_base_model/FactoryFunctions.hpp"

#include <set>


namespace floating_base_model
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::PinocchioInterface createPinocchioInterface(
    const ::urdf::ModelInterfaceSharedPtr& urdfInterface,
    const std::string& baseLinkName)
  {
    using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

    if (urdfInterface == nullptr) {
      throw std::invalid_argument("URDF model tree is nullptr!");
    }

    // Copy existing urdf interface
    ::urdf::ModelInterfaceSharedPtr urdfTree = std::make_shared<::urdf::ModelInterface>(*urdfInterface);

    // remove extraneous joints from urdf
    std::vector<std::string> jointsToRemove;
    std::vector<std::string> linksToRemove;

    ::urdf::LinkConstSharedPtr baseLink = urdfTree ->getLink(baseLinkName);

    if(baseLink == nullptr)
    {
      throw std::invalid_argument("Could not find base link with name: " + baseLinkName);
    }

    // Keep only Base Link and its descendants (removes parents of Base Link and all other branches)
    std::set<std::string> linksToKeep;
    std::vector<::urdf::LinkConstSharedPtr> linksToVisit = {baseLink};
    while(!linksToVisit.empty())
    {
      ::urdf::LinkConstSharedPtr link = linksToVisit.back();
      linksToVisit.pop_back();
      linksToKeep.insert(link->name);
      linksToVisit.insert(linksToVisit.end(), link->child_links.begin(), link->child_links.end());
    }

    for(auto& link : urdfTree ->links_)
    {
      if(linksToKeep.count(link.first) == 0)
      {
        linksToRemove.push_back(link.first);
      }
    }

    // Get all joints that are not between kept links (including the joint into Base Link)
    for (joint_pair_t& jointPair : urdfTree ->joints_)
    {
      if (linksToKeep.count(jointPair.second->parent_link_name) == 0 || linksToKeep.count(jointPair.second->child_link_name) == 0)
      {
        jointsToRemove.push_back(jointPair.first);
      }
    }

    // Remove parents and their joints from tree
    for(auto& jointToRemoveName : jointsToRemove)
    {
      urdfTree ->joints_.erase(jointToRemoveName);
    }

    for(auto& linkToRemoveName : linksToRemove)
    {
      urdfTree ->links_.erase(linkToRemoveName);
    }

    // Remove child joints and links to prepare tree for initTree()
    for(auto& link : urdfTree ->links_)
    {
      link.second->child_joints.clear();
      link.second->child_links.clear();
    }

    std::map<std::string, std::string> parent_link_tree;
    try
    {
      urdfTree ->initTree(parent_link_tree);
    }
    catch(::urdf::ParseError &e)
    {
      throw;
    }

    try
    {
      urdfTree ->initRoot(parent_link_tree);
    }
    catch(::urdf::ParseError &e)
    {
      throw;
    }

    pinocchio::JointModelFreeFlyer freeFlyerJoint;
    return ocs2::getPinocchioInterfaceFromUrdfModel(urdfTree , freeFlyerJoint);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::PinocchioInterface createPinocchioInterfaceFromXmlString(const std::string& xmlString,
    const std::string& baseLinkName)
  {
    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlString);
    if (urdfTree != nullptr) 
    {
      return createPinocchioInterface(urdfTree, baseLinkName);
    } 
    else 
    {
      throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::PinocchioInterface createPinocchioInterfaceFromUrdfFile(const std::string& urdfFilePath, 
    const std::string& baseLinkName)
  {
    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFilePath);
    if (urdfTree != nullptr) 
    {
      return createPinocchioInterface(urdfTree, baseLinkName);
    } 
    else 
    {
      throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model.");
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  FloatingBaseModelInfo createFloatingBaseModelInfo(const ocs2::PinocchioInterface& interface,
    const std::vector<std::string>& threeDofContactNames,
    const std::vector<std::string>& sixDofContactNames)
  {

    FloatingBaseModelInfo info;
    const pinocchio::Model& model = interface.getModel();

    info.numThreeDofContacts = threeDofContactNames.size();
    info.numSixDofContacts = sixDofContactNames.size();
    info.generalizedCoordinatesNum = model.nq - 1; // pinocchio freeflyer joint has quaterion angles (4), not Euler ZYX (3)
    info.actuatedDofNum = model.nv - 6;
    info.stateDim = info.generalizedCoordinatesNum + model.joints[1].nv(); // pinocchio freeflyer joint has 6 velocity variables
    info.inputDim = info.actuatedDofNum + 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;
    info.robotMass = pinocchio::computeTotalMass(model);

    for (const auto& name : threeDofContactNames) {
      const size_t threeDofContactFrameIndex = model.getFrameId(name);
      if(threeDofContactFrameIndex == model.frames.size())
      {
        std::cout << "FloatingBaseModelInfo error: Could not find contact frame!" << std::endl;
        throw std::invalid_argument("Could not find contact frame with name: " + name);
      }

      const size_t threeDofContactJointIndex = model.frames[threeDofContactFrameIndex].parentJoint;
  
      info.endEffectorFrameIndices.push_back(threeDofContactFrameIndex);
      info.endEffectorJointIndices.push_back(threeDofContactJointIndex);
    }

    for (const auto& name : sixDofContactNames) {
      const size_t sixDofContactFrameIndex = model.getFrameId(name);
      if(sixDofContactFrameIndex == model.frames.size())
      {
        std::cout << "FloatingBaseModelInfo error: Could not find contact frame!" << std::endl;
        throw std::invalid_argument("Could not find contact frame with name: " + name);
      }
      const size_t sixDofContactJointIndex = model.frames[sixDofContactFrameIndex].parentJoint;
  
      info.endEffectorFrameIndices.push_back(sixDofContactFrameIndex);
      info.endEffectorJointIndices.push_back(sixDofContactJointIndex);
    }

    return info;
  }

} // namespace floating_base_model