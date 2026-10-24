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

#include <legged_whole_body_control/WeightedWbc.hpp>

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <unordered_set>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace legged_whole_body_control 
{
  
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace proxsuite;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WeightedWbc::WeightedWbc(const ocs2::PinocchioInterface& pinocchioInterface, 
    floating_base_model::FloatingBaseModelInfo info, 
    WbcBase::Settings settings, Weights weights):
      WbcBase(pinocchioInterface, info, settings), weights_(std::move(weights)) {}
  
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WeightedWbc::calculate(ocs2::scalar_t time)
  {
    WbcBase::calculate(time);

    // Stacked tasks always have (rows x numberOfDecisionVariables_) matrices, as expected by the solver
    const Task weighedTask = formulateWeightedTask();
    const Task constraints = formulateConstraints();

    constexpr double infinity = std::numeric_limits<scalar_t>::infinity();

    // H = a^T a, only the lower triangle is computed
    matrix_t H = matrix_t::Zero(numberOfDecisionVariables_, numberOfDecisionVariables_);
    H.selfadjointView<Eigen::Lower>().rankUpdate(weighedTask.a_.transpose());
    H.triangularView<Eigen::StrictlyUpper>() = H.transpose();

    vector_t g(numberOfDecisionVariables_);
    g.noalias() = -weighedTask.a_.transpose() * weighedTask.b_;

    const size_t equalityConstraintsDimension = constraints.b_.size();
    const size_t inequalityConstraintsDimension = constraints.f_.size();

    // Inequality constraints are formulated as D x <= f
    const vector_t lowerBound = vector_t::Constant(inequalityConstraintsDimension, -infinity);

    const auto& a = constraints.a_;
    const auto& d = constraints.d_;

    if(!H.allFinite() || !g.allFinite() || !a.allFinite() || !constraints.b_.allFinite()
      || !d.allFinite() || !constraints.f_.allFinite())
    {
      std::cerr << "[WeightedWbc]: QP data is not finite, keeping previous result!" << std::endl;
      return;
    }

    // Number of constraints depends on contact flags, solver must be recreated when it changes
    const bool dimensionsChanged = !qpSolver_
      || static_cast<size_t>(qpSolver_->model.n_eq) != equalityConstraintsDimension
      || static_cast<size_t>(qpSolver_->model.n_in) != inequalityConstraintsDimension;

    if(dimensionsChanged)
    {
      qpSolver_ = std::make_unique<proxqp::dense::QP<scalar_t>>(numberOfDecisionVariables_,
        equalityConstraintsDimension, inequalityConstraintsDimension);

      // Warm start with previous result leads to false primal infeasibility detections
      qpSolver_->settings.initial_guess =
        proxqp::InitialGuessStatus::EQUALITY_CONSTRAINED_INITIAL_GUESS;

      qpSolver_->init(H, g, a, constraints.b_, d, lowerBound, constraints.f_);
    }
    else
    {
      // Preconditioner is computed only in init(); recomputing it on every update costs ~80 us
      // and does not reduce the number of iterations, also after contact switches
      constexpr bool updatePreconditioner = false;
      qpSolver_->update(H, g, a, constraints.b_, d, lowerBound, constraints.f_,
        updatePreconditioner);
    }

    qpSolver_->solve();

    if(qpSolver_->results.info.status != proxqp::QPSolverOutput::PROXQP_SOLVED
      || !qpSolver_->results.x.allFinite())
    {
      std::cerr << "[WeightedWbc]: QP not solved, keeping previous result!" << std::endl;
      // Solver is recreated after a failure
      qpSolver_.reset();
      return;
    }

    currentResult_ = qpSolver_->results.x;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WeightedWbc::formulateWeightedTask()
  {
    // Tasks are kept in place (reserved), so references to them stay valid
    std::vector<Task> tasks;
    tasks.reserve(MAX_TASK_NUMBER);

    // Tasks are scaled in place by sqrt(weight), so that cost is weight * ||a x - b||^2
    if(settings_.useBaseTrackingTask)
    {
      tasks.push_back(std::sqrt(weights_.weightBaseTrackingTask) * formulateBaseTrackingTask());
    }

    if(settings_.useEndEffectorsTrackingTask)
    {
      tasks.push_back(std::sqrt(weights_.weightEndEffectorsTrackingTask) * formulateEndEffectorsTrackingTask());
    }

    if(settings_.useContactForceTrackingTask)
    {
      tasks.push_back(std::sqrt(weights_.weightContactForceTrackingTask) * formulateContactForceTrackingTask());
    }

    return Task::stack({tasks.begin(), tasks.end()}, numberOfDecisionVariables_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WeightedWbc::formulateConstraints()
  {
    std::vector<Task> tasks;
    tasks.reserve(MAX_TASK_NUMBER);

    std::vector<std::reference_wrapper<const Task>> constraints;
    constraints.reserve(MAX_TASK_NUMBER);

    if(settings_.useDynamicsTask)
    {
      constraints.emplace_back(tasks.emplace_back(formulateDynamicsTask()));
    }

    if(settings_.useTorqueLimitsTask)
    {
      // Constant task, not copied
      constraints.emplace_back(torqueLimitsTask_);
    }

    if(settings_.useKinematicContactTask)
    {
      constraints.emplace_back(tasks.emplace_back(formulateKinematicContactTask()));
    }

    if(settings_.useFrictionConeTask)
    {
      constraints.emplace_back(tasks.emplace_back(formulateFrictionConeTask()));
    }

    return Task::stack(constraints, numberOfDecisionVariables_);
  }

  WeightedWbc::Weights loadWeightedWbcSettings(const std::string& filename,
    const std::string& fieldName, bool verbose)
  {
    WeightedWbc::Weights weights;

    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if(verbose) 
    {
      std::cerr << "\n #### Legged Weighted Whole Body Controller Weights :";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, weights.weightBaseTrackingTask, fieldName + ".weightBaseTrackingTask", verbose);
    if(weights.weightBaseTrackingTask < 0.0)
    {
      throw std::invalid_argument("[WeightedWbc]: Base tracking task weight smaller than 0!");
    }
    
    loadData::loadPtreeValue(pt, weights.weightEndEffectorsTrackingTask, fieldName + ".weightEndEffectorsTrackingTask", verbose);
    if(weights.weightEndEffectorsTrackingTask < 0.0)
    {
      throw std::invalid_argument("[WeightedWbc]: End effector tracking task weight smaller than 0!");
    }
    
    loadData::loadPtreeValue(pt, weights.weightContactForceTrackingTask, fieldName + ".weightContactForceTrackingTask", verbose);
    if(weights.weightContactForceTrackingTask < 0.0)
    {
      throw std::invalid_argument("[WeightedWbc]: Contact force tracking task weight smaller than 0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return weights;
  }
} // namespace legged_whole_body_control