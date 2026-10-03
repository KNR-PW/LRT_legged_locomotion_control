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

#ifndef __LEGGED_WHOLE_BODY_CONTROL_WEIGHTED_WBC__
#define __LEGGED_WHOLE_BODY_CONTROL_WEIGHTED_WBC__

#include <legged_whole_body_control/WbcBase.h>

namespace legged_whole_body_control 
{
  class WeightedWbc: public WbcBase
  {
    public:

      WeightedWbc(const ocs2::PinocchioInterface& pinocchioInterface, 
        floating_base_model::FloatingBaseModelInfo info, 
        WbcBase::Settings settings, Weights weights);

      void calculate(ocs2::scalar_t time) override;

    private:

      Task formulateWeightedTask();

      Weights weights_;
  };
}; // namespace legged_whole_body_control
#endif
