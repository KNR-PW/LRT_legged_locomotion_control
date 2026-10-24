#include <legged_locomotion_mpc/collision/PinocchioForwardCollisionKinematicsCppAd.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <floating_base_model/QuaterionEulerTransforms.hpp>

namespace legged_locomotion_mpc 
{

  using namespace ocs2;
  using namespace collision;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioForwardCollisionKinematicsCppAd::PinocchioForwardCollisionKinematicsCppAd(
    const PinocchioInterface& pinocchioInterface, const FloatingBaseModelInfo info,
    const CollisionSettings& collisionSettings, const std::string& modelName,
    const std::string& modelFolder, bool recompileLibraries, bool verbose): 
      numEndEffectors_(info.numThreeDofContacts + info.numSixDofContacts), 
      numCollisions_(collisionSettings.collisionLinkNames.size())
  {

    const auto model = pinocchioInterface.getModel();

    for(const auto& frameName : collisionSettings.collisionLinkNames)
    {
      const size_t frameIndex = model.getFrameId(frameName);
      // Pinocchio returns number of frames when frame is not found
      if(frameIndex == model.frames.size())
      {
        std::string message = "[PinocchioForwardCollisionKinematicsCppAd]: There is no frame named " + frameName + "!";
        throw std::invalid_argument(message);
      }
      collisionFrameIndices_.push_back(frameIndex);
    }

    // initialize CppAD interface
    auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
    // mapping
    FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
    mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);

    // position function
    auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) 
    {
      y = getPositionCppAd(pinocchioInterfaceCppAd, mappingCppAd, x);
    };
    positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, info.stateDim, modelName + "_position", modelFolder));

    // orientation function
    auto orientationFunc = [&](const ad_vector_t& x, ad_vector_t& y) 
    {
      y = getOrientationCppAd(pinocchioInterfaceCppAd, mappingCppAd, x);
    };
    orientationCppAdInterfacePtr_.reset(new CppAdInterface(orientationFunc, info.stateDim, modelName + "_orientation", modelFolder));

    if(recompileLibraries) 
    {
      positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      orientationCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    } 
    else 
    {
      positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      orientationCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioForwardCollisionKinematicsCppAd::PinocchioForwardCollisionKinematicsCppAd(
    const PinocchioForwardCollisionKinematicsCppAd& rhs)
    :positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
    orientationCppAdInterfacePtr_(new CppAdInterface(*rhs.orientationCppAdInterfacePtr_)),
    numCollisions_(rhs.numCollisions_), collisionFrameIndices_(rhs.collisionFrameIndices_), 
    numEndEffectors_(rhs.numEndEffectors_){}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioForwardCollisionKinematicsCppAd* PinocchioForwardCollisionKinematicsCppAd::clone() const 
  {
    return new PinocchioForwardCollisionKinematicsCppAd(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t PinocchioForwardCollisionKinematicsCppAd::getPositionCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const FloatingBaseModelPinocchioMappingCppAd& mapping,
    const ad_vector_t& state) 
  {
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ad_vector_t positions(3 * numCollisions_);
    for (size_t i = 0; i < numCollisions_; i++) {
      const size_t frameId = collisionFrameIndices_[i];
      positions.segment<3>(3 * i) = data.oMf[frameId].translation();
    }
    return positions;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto PinocchioForwardCollisionKinematicsCppAd::getPosition(const vector_t& state) const -> std::vector<vector3_t> 
  {
    const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

    std::vector<vector3_t> positions;
    for (size_t i = 0; i < numCollisions_; i++) {
      positions.emplace_back(positionValues.segment<3>(3 * i));
    }
    return positions;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> PinocchioForwardCollisionKinematicsCppAd::getPositionLinearApproximation(
      const vector_t& state) const 
  {
    const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);
    const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state);

    std::vector<VectorFunctionLinearApproximation> positions;
    for (size_t i = 0; i < numCollisions_; i++) {
      VectorFunctionLinearApproximation pos;
      pos.f = positionValues.segment<3>(3 * i);
      pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
      positions.emplace_back(std::move(pos));
    }
    return positions;
  }


  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t PinocchioForwardCollisionKinematicsCppAd::getOrientationCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const FloatingBaseModelPinocchioMappingCppAd& mapping,
    const ad_vector_t& state) 
  {
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ad_vector_t orientations(3 * numCollisions_);
    for (size_t i = 0; i < numCollisions_; i++) 
    {
      const size_t frameId = collisionFrameIndices_[i];
      orientations.segment<3>(3 * i) = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(data.oMf[frameId].rotation());
    }
    return orientations;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto PinocchioForwardCollisionKinematicsCppAd::getOrientation(const vector_t& state) const -> std::vector<vector3_t> 
  {
    const vector_t orientationValues = orientationCppAdInterfacePtr_->getFunctionValue(state);
    std::vector<vector3_t> orientations;
    for (size_t i = 0; i < numCollisions_; i++) 
    {
      orientations.emplace_back(orientationValues.segment<3>(3 * i));
    }
    return orientations;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> PinocchioForwardCollisionKinematicsCppAd::getOrientationLinearApproximation(
    const vector_t& state) const 
  {
    const vector_t orientationValues = orientationCppAdInterfacePtr_->getFunctionValue(state);
    const matrix_t orientationJacobian = orientationCppAdInterfacePtr_->getJacobian(state);

    std::vector<VectorFunctionLinearApproximation> orientations;
    for (size_t i = 0; i < numCollisions_; i++) 
    {
      VectorFunctionLinearApproximation rot;
      rot.f = orientationValues.segment<3>(3 * i);
      rot.dfdx = orientationJacobian.block(3 * i, 0, 3, state.rows());
      orientations.emplace_back(std::move(rot));
    }
    return orientations;
  }

  size_t PinocchioForwardCollisionKinematicsCppAd::getCollisionNumber() const
  {
    return numCollisions_;
  }
}  // namespace legged_locomotion_mpc