/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/kinematics.hpp>

#include <gtest/gtest.h>

#include <ocs2_collision_nextgen/impl/SphereCollisionModel.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"
#include "ocs2_mobile_manipulator/package_path.h"

using namespace ocs2;
using namespace mobile_manipulator;

TEST(NextgenSelfCollision, SphereSoADistancesMatchPinocchioFcl) {
  const std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/franka/task.info";
  const std::string sphereUrdf = ocs2::mobile_manipulator::getPath() + "/config/franka/panda_collision_spheres.urdf";

  const ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);

  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", collisionLinkPairs, true);

  PinocchioInterface pinocchioInterface = createPinocchioInterface(sphereUrdf, modelType, removeJointNames);
  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs);
  const collision_nextgen::impl::SphereCollisionModel sphereModel(pinocchioInterface.getModel(), geometryInterface.getGeometryModel());

  const vector_t q = vector_t::Zero(pinocchioInterface.getModel().nq);
  pinocchio::forwardKinematics(pinocchioInterface.getModel(), pinocchioInterface.getData(), q);

  const auto fclDistances = geometryInterface.computeDistances(pinocchioInterface);

  const vector_t nextgenDistances = sphereModel.getDistances(pinocchioInterface);
  ASSERT_EQ(nextgenDistances.rows(), static_cast<int>(fclDistances.size()));
  ASSERT_GT(nextgenDistances.rows(), 0);
  for (int i = 0; i < nextgenDistances.rows(); ++i) {
    EXPECT_NEAR(nextgenDistances[i], fclDistances[static_cast<size_t>(i)].min_distance, 1e-9);
  }
}
