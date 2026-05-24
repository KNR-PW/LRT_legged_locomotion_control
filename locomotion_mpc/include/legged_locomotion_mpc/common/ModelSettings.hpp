/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2025

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __MODEL_SETTINGS_LEGGED_LOCOMOTION_MPC__
#define __MODEL_SETTINGS_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>

namespace legged_locomotion_mpc
{
  struct ModelSettings 
  {
    // Robot name
    std::string robotName;
    
    // Algorithm type
    std::string algorithm;

    // CppAD
    bool verbose = true;
    bool recompileLibrariesCppAd = true;
    std::string modelFolderCppAd = "/tmp/legged_locomotion_mpc";

    // World, base link and contact link frames
    std::string worldLinkName;
    std::string baseLinkName;
    std::vector<std::string> endEffectorThreeDofNames;
    std::vector<std::string> endEffectorSixDofNames;

    // Hip frame of 3D and 6D contact frames (first 3D then 6D)
    std::vector<std::string> hipFrameNames;
  };

  /**
   * Creates MPC ModelSettings 
   * @param [in] filename: file path with model settings.
   * @param [in] fieldName: field where settings are defined
   * @param [in] verbose: verbose flag
   * @return ModelSettings struct
   */
  ModelSettings loadModelSettings(const std::string& filename,
    const std::string& fieldName = "legged_model_settings",
    bool verbose = "true");

} // namespace legged_locomotion_mpc

#endif