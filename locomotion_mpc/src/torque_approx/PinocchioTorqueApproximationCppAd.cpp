#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>

#include <legged_locomotion_mpc/common/ModelHelperFunctions.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/ModelHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <functional>

namespace legged_locomotion_mpc 
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioTorqueApproximationCppAd::PinocchioTorqueApproximationCppAd(
    const PinocchioInterface& pinocchioInterface,
    const FloatingBaseModelInfo info,
    vector_t torqueDynamicsError,
    const std::string& modelName,
    const std::string& modelFolder,
    bool recompileLibraries, bool verbose): 
      info_(info), torqueDynamicsError_(std::move(torqueDynamicsError))
  {
    const auto& model = pinocchioInterface.getModel();
    assert(torqueDynamicsError_.rows() == (info.actuatedDofNum));
    
    // torque approximation function
    auto torqueApproxFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) 
    {
      // initialize CppAD interface
      auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
      // mapping
      FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
      mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);
  
      const ad_vector_t state = x.head(info.stateDim);
      const ad_vector_t input = x.tail(info.inputDim);
      y = getValueCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input);
    };

    torqueApproxCppAdInterfacePtr_.reset(new CppAdInterface(torqueApproxFunc,
      info_.stateDim + info_.inputDim, modelName + "_torque_approx", modelFolder));

    if(recompileLibraries) 
    {
      torqueApproxCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    } 
    else 
    {
      torqueApproxCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioTorqueApproximationCppAd::PinocchioTorqueApproximationCppAd(
    const PinocchioTorqueApproximationCppAd& rhs)
      :torqueApproxCppAdInterfacePtr_(new CppAdInterface(*rhs.torqueApproxCppAdInterfacePtr_)), 
      torqueDynamicsError_(rhs.torqueDynamicsError_), info_(rhs.info_) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioTorqueApproximationCppAd* PinocchioTorqueApproximationCppAd::clone() const 
  {
    return new PinocchioTorqueApproximationCppAd(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t PinocchioTorqueApproximationCppAd::getValue(
    const vector_t& state, const vector_t& input) const
  {
    const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
    return torqueApproxCppAdInterfacePtr_->getFunctionValue(stateInput) + torqueDynamicsError_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation PinocchioTorqueApproximationCppAd::getLinearApproximation(
    const vector_t& state, const vector_t& input) const
  {
    const vector_t stateInput = (vector_t(state.rows() + input.rows()) << state, input).finished();
    VectorFunctionLinearApproximation approx;
    approx.f = torqueApproxCppAdInterfacePtr_->getFunctionValue(stateInput) + torqueDynamicsError_;
    const matrix_t torqueApproxJacobian = torqueApproxCppAdInterfacePtr_->getJacobian(stateInput);
    approx.dfdx = torqueApproxJacobian.leftCols(state.rows());
    approx.dfdu = torqueApproxJacobian.middleCols(state.rows(), input.rows());
    return approx;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t PinocchioTorqueApproximationCppAd::getValueCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    const ad_vector_t& state,
    const ad_vector_t& input)
  {
    const auto& infoCppAd = info_.toCppAd();

    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();

    pinocchio::forwardKinematics(model, data, q);

    using Force = pinocchio::ForceTpl<ad_scalar_t, 0>;
    Force force;
    force.linear() = Eigen::Vector<ad_scalar_t, 3>::Zero();
    force.angular() = Eigen::Vector<ad_scalar_t, 3>::Zero();
    pinocchio::container::aligned_vector<Force> fext(model.njoints, force);

    floating_base_model::model_helper_functions::computeSpatialForces(
      pinocchioInterfaceCppAd, infoCppAd, input, fext);

    return legged_locomotion_mpc::model_helper_functions::
      computeApproxActuatedJointGeneralizedTorques(pinocchioInterfaceCppAd, 
        infoCppAd, q, fext);
  }
} // namespace legged_locomotion_mpc