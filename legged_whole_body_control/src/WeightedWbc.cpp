//
// Created by qiayuan on 22-12-23.
//

// Copyright (c) 2026, Bartłomiej Krajewski
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Modified by: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#include <legged_whole_body_control/WeightedWbc.h>

#include <qpOASES.hpp>

namespace legged_whole_body_control 
{
  using namespace ocs2;
  using namespace floating_base_model;

  WeightedWbc::WeightedWbc(const ocs2::PinocchioInterface& pinocchioInterface, 
    floating_base_model::FloatingBaseModelInfo info, 
    WbcBase::Settings settings, Weights weights):
      WbcBase(pinocchioInterface, info, settings), settings_(std::move(weights));

  void WeightedWbc::calculate(ocs2::scalar_t time) override
  {
    WbcBase::calculate(time);

    Task weighedTask = formulateWeightedTask()
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

    // Solve
    auto qpProblem = qpOASES::QProblem(numberOfDecisionVariables_, numConstraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    options.enableEqualities = qpOASES::BT_TRUE;
    qpProblem.setOptions(options);
    int nWsr = 20;

    qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
    vector_t qpSol(numberOfDecisionVariables_);

    qpProblem.getPrimalSolution(qpSol.data());
    return qpSol;
  }

  Task WeightedWbc::formulateWeightedTask() 
  {
    Task weightedTask;

    if(settings_.useDynamicsTask)
    {
      weightedTask += weights_.weightDynamicsTask * formulateDynamicsTask();
    }
    if(settings_.useBaseTrackingTask)
    {
      weightedTask += weights_.weightBaseTrackingTask * formulateBaseTrackingTask();
    }
    if(settings_.useEndEffectorsTrackingTask)
    {
      weightedTask += weights_.weightEndEffectorsTrackingTask * formulateEndEffectorsTrackingTask();
    }
    if(settings_.useContactForceTrackingTask)
    {
      weightedTask += weights_.weightContactForceTrackingTask * formulateContactForceTrackingTask();
    }
    if(settings_.useTorqueLimitsTask)
    {
      weightedTask += weights_.weightTorqueLimitsTask * formulateTorqueLimitsTask();
    }
    if(settings_.useKinematicContactTask)
    {
      weightedTask += weights_.weightKinematicContactTask * formulateKinematicContactTask();
    }
    if(settings_.useFrictionConeTask)
    {
      weightedTask += weights_.weightFrictionConeTask * formulateFrictionConeTask();
    }

    return weightedTask;
  }
} //  namespace legged_whole_body_control
void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose) {
  WbcBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### WBC weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged
