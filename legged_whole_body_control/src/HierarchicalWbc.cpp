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

#include <legged_whole_body_control/HierarchicalWbc.h>
#include <legged_whole_body_control/HoQp.h">

namespace legged_whole_body_control 
{
  using namespace ocs2;
  using namespace floating_base_model;

  HierarchicalWbc::HierarchicalWbc(const PinocchioInterface& pinocchioInterface, 
    FloatingBaseModelInfo info, 
    WbcBase::Settings settings, HierarchicalWbc::Hierarchy hierarchy):
      WbcBase(pinocchioInterface, info, settings), hierarchy_(std::move(hierarchy)) {}
  
  void HierarchicalWbc::calculate(ocs2::scalar_t time)
  {
    WbcBase::calculate(time);

    Task task0 = formulateDynamicsTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateKinematicContactTask();
    Task task1 = formulateBaseTrackingTask() + formulateEndEffectorsTrackingTask();
    Task task2 = formulateContactForceTrackingTask();
    HoQp hoQp(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));

    // std::vector<Task> tasks;
    // tasks.resize(MAX_TASK_NUMBER);

    // for(const auto& [type, priority]: hierarchy_)
    // {
    //   switch(type)
    //   {
    //     case TaskType::dynamicsTask:
    //       tasks[priority] += formulateDynamicsTask();
    //       break;
    //     case TaskType::baseTrackingTask:
    //       tasks[priority] += formulateBaseTrackingTask();
    //       break;
    //     case TaskType::endEffectorsTrackingTask:
    //       tasks[priority] += formulateEndEffectorsTrackingTask();
    //       break;
    //     case TaskType::contactForceTrackingTask:
    //       tasks[priority] += formulateContactForceTrackingTask();
    //       break;
    //     case TaskType::torqueLimitsTask:
    //       tasks[priority] += formulateTorqueLimitsTask();
    //       break;
    //     case TaskType::kinematicContactTask:
    //       tasks[priority] += formulateKinematicContactTask();
    //       break;
    //     case TaskType::frictionConeTask:
    //       tasks[priority] += formulateFrictionConeTask();
    //       break;
    //   }
    // }

    // std::vector<std::shared_ptr<HoQp>> hoQps;

    // std::shared_ptr<HoQp> firsthQp = std::make_shared<HoQp>(std::move(tasks[0]));

    // for(size_t i = 1; i <tasks.size(); ++i)
    // {
    //   if(tasks[i].isActive())
    //   {
    //     const auto& [type, priority] = hierarchy_[i];

    //   }
    // }
    return hoQp.getSolutions();
  }
} // namespace legged_whole_body_control 
