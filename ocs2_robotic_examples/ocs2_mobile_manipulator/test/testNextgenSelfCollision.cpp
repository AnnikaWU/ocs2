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

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <cmath>
#include <utility>

#include <gtest/gtest.h>

#include <ocs2_collision_nextgen/impl/SphereCollisionModel.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision/SelfCollisionConstraint.h>

#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorNextgenSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/package_path.h"

using namespace ocs2;
using namespace mobile_manipulator;

namespace {

template <typename SCALAR>
class IdentityPinocchioMapping final : public PinocchioStateInputMapping<SCALAR> {
 public:
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  IdentityPinocchioMapping() = default;
  ~IdentityPinocchioMapping() override = default;
  IdentityPinocchioMapping<SCALAR>* clone() const override { return new IdentityPinocchioMapping<SCALAR>(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override { return state; }
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override { return input; }
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    return {Jq, Jv};
  }
};

class TestPinocchioPreComputation final : public PreComputation {
 public:
  explicit TestPinocchioPreComputation(PinocchioInterface pinocchioInterface) : pinocchioInterface_(std::move(pinocchioInterface)) {}

  TestPinocchioPreComputation* clone() const override { return new TestPinocchioPreComputation(*this); }

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override {
    pinocchio::forwardKinematics(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), x);
    pinocchio::updateGlobalPlacements(pinocchioInterface_.getModel(), pinocchioInterface_.getData());
    pinocchio::computeJointJacobians(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), x);
  }

  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 private:
  PinocchioInterface pinocchioInterface_;
};

class TestSelfCollisionConstraint final : public SelfCollisionConstraint {
 public:
  TestSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping, PinocchioGeometryInterface geometryInterface,
                              scalar_t minimumDistance)
      : SelfCollisionConstraint(mapping, std::move(geometryInterface), minimumDistance) {}

  TestSelfCollisionConstraint* clone() const override { return new TestSelfCollisionConstraint(*this); }

 protected:
  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<TestPinocchioPreComputation>(preComputation).getPinocchioInterface();
  }
};

class TestNextgenSelfCollisionConstraint final : public collision_nextgen::NextgenSelfCollisionConstraint {
 public:
  TestNextgenSelfCollisionConstraint(const PinocchioStateInputMapping<scalar_t>& mapping, const pinocchio::Model& model,
                                     const pinocchio::GeometryModel& geometryModel, scalar_t minimumDistance)
      : NextgenSelfCollisionConstraint(mapping, model, geometryModel, minimumDistance) {}

  TestNextgenSelfCollisionConstraint* clone() const override { return new TestNextgenSelfCollisionConstraint(*this); }

 protected:
  const PinocchioInterface& getPinocchioInterface(const PreComputation& preComputation) const override {
    return cast<TestPinocchioPreComputation>(preComputation).getPinocchioInterface();
  }
};

const char* branchSphereUrdf = R"(
<robot name="branch_spheres">
  <link name="base"/>
  <link name="left_link">
    <collision name="left_sphere">
      <origin xyz="1.0 0.0 0.0" rpy="0 0 0"/>
      <geometry><sphere radius="0.4"/></geometry>
    </collision>
  </link>
  <link name="right_link">
    <collision name="right_sphere">
      <origin xyz="1.0 0.0 0.0" rpy="0 0 0"/>
      <geometry><sphere radius="0.4"/></geometry>
    </collision>
  </link>
  <joint name="left_yaw" type="revolute">
    <parent link="base"/>
    <child link="left_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="right_yaw" type="revolute">
    <parent link="base"/>
    <child link="right_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>
)";

void compareLinearApproximationForState(const vector_t& state, scalar_t expectedSignedDistanceLowerBound,
                                        scalar_t expectedSignedDistanceUpperBound) {
  PinocchioInterface pinocchioInterface = getPinocchioInterfaceFromUrdfString(branchSphereUrdf);
  PinocchioGeometryInterface geometryInterface(pinocchioInterface, {{"left_link", "right_link"}});
  TestPinocchioPreComputation preComputation(pinocchioInterface);
  const vector_t input = vector_t::Zero(0);
  preComputation.request(Request::SoftConstraint + Request::Approximation, 0.0, state, input);

  const IdentityPinocchioMapping<scalar_t> mapping;
  TestSelfCollisionConstraint fclConstraint(mapping, PinocchioGeometryInterface(pinocchioInterface, {{"left_link", "right_link"}}), 0.0);
  TestNextgenSelfCollisionConstraint nextgenConstraint(mapping, pinocchioInterface.getModel(), geometryInterface.getGeometryModel(), 0.0);

  const auto fclLinear = fclConstraint.getLinearApproximation(0.0, state, preComputation);
  const auto nextgenLinear = nextgenConstraint.getLinearApproximation(0.0, state, preComputation);

  ASSERT_EQ(fclLinear.f.rows(), 1);
  ASSERT_EQ(nextgenLinear.f.rows(), 1);
  EXPECT_GT(fclLinear.f[0], expectedSignedDistanceLowerBound);
  EXPECT_LT(fclLinear.f[0], expectedSignedDistanceUpperBound);
  EXPECT_TRUE(nextgenLinear.f.isApprox(fclLinear.f, 1e-9)) << "fcl: " << fclLinear.f.transpose()
                                                          << "\nnextgen: " << nextgenLinear.f.transpose();
  EXPECT_TRUE(nextgenLinear.dfdx.isApprox(fclLinear.dfdx, 1e-9)) << "fcl:\n"
                                                                 << fclLinear.dfdx << "\nnextgen:\n"
                                                                 << nextgenLinear.dfdx;
}

}  // namespace

TEST(NextgenSelfCollision, HandBuiltSphereLinearApproximationMatchesPinocchioFclWithoutCollision) {
  vector_t state(2);
  state << 0.0, M_PI;

  compareLinearApproximationForState(state, 1.0, 1.4);
}

TEST(NextgenSelfCollision, HandBuiltSphereLinearApproximationMatchesPinocchioFclWithCollision) {
  vector_t state(2);
  state << 0.0, 0.25;

  compareLinearApproximationForState(state, -0.7, -0.4);
}

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

TEST(NextgenSelfCollision, LinearApproximationMatchesPinocchioFcl) {
  const std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/franka/task.info";
  const std::string sphereUrdf = ocs2::mobile_manipulator::getPath() + "/config/franka/panda_collision_spheres.urdf";

  const ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
  std::vector<std::string> removeJointNames;
  loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);

  std::string baseFrame;
  std::string eeFrame;
  loadData::loadCppDataType(taskFile, "model_information.baseFrame", baseFrame);
  loadData::loadCppDataType(taskFile, "model_information.eeFrame", eeFrame);

  scalar_t minimumDistance = 0.0;
  loadData::loadCppDataType(taskFile, "selfCollision.minimumDistance", minimumDistance);

  std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
  loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", collisionLinkPairs, true);

  PinocchioInterface pinocchioInterface = createPinocchioInterface(sphereUrdf, modelType, removeJointNames);
  const ManipulatorModelInfo modelInfo = createManipulatorModelInfo(pinocchioInterface, modelType, baseFrame, eeFrame);
  PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs);
  MobileManipulatorPreComputation preComputation(pinocchioInterface, modelInfo);

  vector_t state = vector_t::Zero(modelInfo.stateDim);
  state << 0.0, 0.171, 0.114, -1.57, 0.05, 1.57, 0.469;
  const vector_t input = vector_t::Zero(modelInfo.inputDim);
  preComputation.request(Request::SoftConstraint + Request::Approximation, 0.0, state, input);

  MobileManipulatorSelfCollisionConstraint fclConstraint(MobileManipulatorPinocchioMapping(modelInfo),
                                                         PinocchioGeometryInterface(pinocchioInterface, collisionLinkPairs),
                                                         minimumDistance);
  MobileManipulatorNextgenSelfCollisionConstraint nextgenConstraint(MobileManipulatorPinocchioMapping(modelInfo),
                                                                    pinocchioInterface.getModel(),
                                                                    geometryInterface.getGeometryModel(), minimumDistance);

  const auto fclLinear = fclConstraint.getLinearApproximation(0.0, state, preComputation);
  const auto nextgenLinear = nextgenConstraint.getLinearApproximation(0.0, state, preComputation);

  ASSERT_TRUE(nextgenLinear.f.isApprox(fclLinear.f, 1e-9));
  ASSERT_TRUE(nextgenLinear.dfdx.isApprox(fclLinear.dfdx, 1e-9));
}
