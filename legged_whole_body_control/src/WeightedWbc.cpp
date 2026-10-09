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

#include <unordered_set>

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

    const Task weighedTask = formulateWeightedTask();
    const Task constraints = formulateConstraints();

    constexpr double infinity = std::numeric_limits<scalar_t>::infinity();

    matrix_t H = weighedTask.a_.transpose() * weighedTask.a_;
    vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

    if(started_)
    {
      started_ = false;

      const size_t problemDimension = weighedTask.b_.size();
      const size_t equalityConstraintsDimension = constraints.b_.size();
      const size_t inequalityConstraintsDimension = constraints.f_.size();

      qpSolver_ = std::make_unique<proxqp::dense::QP<scalar_t>>(numberOfDecisionVariables_, 
        equalityConstraintsDimension, inequalityConstraintsDimension);

      qpSolver_->settings.initial_guess =
        proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;

      qpSolver_->init(H, g, constraints.a_, constraints.b_, constraints.d_, 
        -infinity * vector_t::Ones(constraints.f_.size()), constraints.f_);
    }
    else
    {
      // f (u) is const for all constraints, same with l 
      qpSolver_->update(H, g, constraints.a_, constraints.b_, constraints.d_, 
        std::nullopt, std::nullopt);
    }

    qpSolver_->solve();

    currentResult_ = qpSolver_->results.x;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WeightedWbc::formulateWeightedTask() 
  {
    Task weightedTask;

    if(settings_.useBaseTrackingTask)
    {
      weightedTask += std::move(weights_.weightBaseTrackingTask * formulateBaseTrackingTask());
    }
    if(settings_.useEndEffectorsTrackingTask)
    {
      weightedTask += std::move(weights_.weightEndEffectorsTrackingTask * formulateEndEffectorsTrackingTask());
    }
    if(settings_.useContactForceTrackingTask)
    {
      weightedTask += std::move(weights_.weightContactForceTrackingTask * formulateContactForceTrackingTask());
    }

    return weightedTask;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Task WeightedWbc::formulateConstraints()
  {
    Task constraints;

    if(settings_.useDynamicsTask)
    {
      constraints += formulateDynamicsTask();
    }
    if(settings_.useTorqueLimitsTask)
    {
      constraints += formulateTorqueLimitsTask();
    }
    if(settings_.useKinematicContactTask)
    {
      constraints += formulateKinematicContactTask();
    }
    if(settings_.useFrictionConeTask)
    {
      constraints += formulateFrictionConeTask();
    }

    return constraints;
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