/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_collision_nextgen/impl/SphereCollisionModel.h>
#include <ocs2_collision_nextgen/impl/SphereDistanceKernel.h>

#include <limits>
#include <stdexcept>

#include <hpp/fcl/collision_utility.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

namespace {

enum class PrimitiveType { Unsupported, Sphere };

PrimitiveType getPrimitiveType(const pinocchio::GeometryObject& object) {
  if (object.geometry == nullptr) {
    return PrimitiveType::Unsupported;
  }
  if (object.geometry->getNodeType() == hpp::fcl::GEOM_SPHERE) {
    return PrimitiveType::Sphere;
  }
  return PrimitiveType::Unsupported;
}

const char* toString(PrimitiveType type) {
  switch (type) {
    case PrimitiveType::Sphere:
      return "sphere";
    case PrimitiveType::Unsupported:
      return "unsupported";
  }
  return "unsupported";
}

double getSphereRadius(const pinocchio::GeometryObject& object) {
  if (object.geometry == nullptr || object.geometry->getNodeType() != hpp::fcl::GEOM_SPHERE) {
    return 0.0;
  }

  const auto* sphere = dynamic_cast<const hpp::fcl::Sphere*>(object.geometry.get());
  if (sphere == nullptr) {
    return 0.0;
  }
  return sphere->radius;
}

std::string objectLabel(const pinocchio::GeometryObject& object, size_t index) {
  if (!object.name.empty()) {
    return object.name;
  }
  return "#" + std::to_string(index);
}

std::int64_t checkedObjectIndex(size_t index) {
  if (index > static_cast<size_t>(std::numeric_limits<std::int64_t>::max())) {
    throw std::runtime_error("[SphereCollisionModel] Geometry object index does not fit into int64_t for SIMD gather.");
  }
  return static_cast<std::int64_t>(index);
}

}  // namespace

SphereCollisionModel::SphereCollisionModel(const pinocchio::Model& model, const pinocchio::GeometryModel& geometryModel) {
  const size_t numObjects = geometryModel.geometryObjects.size();
  objectNames_.reserve(numObjects);
  parentJoint_.reserve(numObjects);
  localX_.reserve(numObjects);
  localY_.reserve(numObjects);
  localZ_.reserve(numObjects);
  radius_.reserve(numObjects);

  std::vector<PrimitiveType> primitiveTypes;
  primitiveTypes.reserve(numObjects);

  for (size_t i = 0; i < geometryModel.geometryObjects.size(); ++i) {
    const auto& object = geometryModel.geometryObjects[i];
    const auto localPosition = object.placement.translation();

    objectNames_.push_back(objectLabel(object, i));
    parentJoint_.push_back(object.parentJoint);
    localX_.push_back(localPosition.x());
    localY_.push_back(localPosition.y());
    localZ_.push_back(localPosition.z());
    radius_.push_back(getSphereRadius(object));
    primitiveTypes.push_back(getPrimitiveType(object));

    if (object.parentFrame >= model.frames.size()) {
      throw std::runtime_error("[SphereCollisionModel] Geometry object '" + objectNames_.back() +
                               "' references parentFrame outside Pinocchio model frames.");
    }
  }

  firstObject_.reserve(geometryModel.collisionPairs.size());
  secondObject_.reserve(geometryModel.collisionPairs.size());
  pairRadiusSum_.reserve(geometryModel.collisionPairs.size());

  for (size_t pairIndex = 0; pairIndex < geometryModel.collisionPairs.size(); ++pairIndex) {
    const auto& pair = geometryModel.collisionPairs[pairIndex];
    if (pair.first >= geometryModel.geometryObjects.size() || pair.second >= geometryModel.geometryObjects.size()) {
      throw std::runtime_error("[SphereCollisionModel] Collision pair " + std::to_string(pairIndex) +
                               " references a geometry object outside the GeometryModel.");
    }
    if (primitiveTypes[pair.first] != PrimitiveType::Sphere || primitiveTypes[pair.second] != PrimitiveType::Sphere) {
      throw std::runtime_error("[SphereCollisionModel] nextgen currently supports sphere-sphere pairs only. Pair " +
                               std::to_string(pairIndex) + " uses " + objectNames_[pair.first] + " (" +
                               toString(primitiveTypes[pair.first]) + ") and " + objectNames_[pair.second] + " (" +
                               toString(primitiveTypes[pair.second]) + ").");
    }

    firstObject_.push_back(checkedObjectIndex(pair.first));
    secondObject_.push_back(checkedObjectIndex(pair.second));
    pairRadiusSum_.push_back(radius_[pair.first] + radius_[pair.second]);
  }

  worldCenterScratch_.x.resize(numObjects);
  worldCenterScratch_.y.resize(numObjects);
  worldCenterScratch_.z.resize(numObjects);
}

const SphereCollisionModel::WorldCenterScratch& SphereCollisionModel::computeWorldCenters(
    const PinocchioInterface& pinocchioInterface) const {
  const auto& data = pinocchioInterface.getData();

  for (size_t objectIndex = 0; objectIndex < getNumObjects(); ++objectIndex) {
    const size_t joint = parentJoint_[objectIndex];
    if (joint >= data.oMi.size()) {
      throw std::runtime_error("[SphereCollisionModel] Geometry object '" + objectNames_[objectIndex] +
                               "' references parentJoint outside Pinocchio data.oMi.");
    }

    // GeometryData::oMg would store data.oMi[parentJoint] * GeometryObject::placement.
    // A sphere only needs the translated center; its local collision rotation is irrelevant.
    const auto& oMi = data.oMi[joint];
    const vector3_t localCenter(localX_[objectIndex], localY_[objectIndex], localZ_[objectIndex]);
    const vector3_t worldCenter = oMi.rotation() * localCenter + oMi.translation();
    worldCenterScratch_.x[objectIndex] = worldCenter.x();
    worldCenterScratch_.y[objectIndex] = worldCenter.y();
    worldCenterScratch_.z[objectIndex] = worldCenter.z();
  }

  return worldCenterScratch_;
}

void SphereCollisionModel::computeDistances(const WorldCenterScratch& worldCenters, double* distances, double* normalX, double* normalY,
                                            double* normalZ) const {
  computeSpherePairDistances(worldCenters.x.data(), worldCenters.y.data(), worldCenters.z.data(), firstObject_.data(), secondObject_.data(),
                             pairRadiusSum_.data(), getNumPairs(), distances, normalX, normalY, normalZ);
}

vector_t SphereCollisionModel::getDistances(const PinocchioInterface& pinocchioInterface) const {
  vector_t distances(getNumPairs());
  const auto& worldCenters = computeWorldCenters(pinocchioInterface);
  computeDistances(worldCenters, distances.data(), nullptr, nullptr, nullptr);
  return distances;
}

SphereCollisionEvaluation SphereCollisionModel::evaluate(const PinocchioInterface& pinocchioInterface) const {
  SphereCollisionEvaluation evaluation(getNumPairs());
  evaluate(pinocchioInterface, evaluation);
  return evaluation;
}

void SphereCollisionModel::evaluate(const PinocchioInterface& pinocchioInterface, SphereCollisionEvaluation& evaluation) const {
  if (evaluation.distances.size() != static_cast<Eigen::Index>(getNumPairs())) {
    evaluation.resize(getNumPairs());
  }

  const auto& worldCenters = computeWorldCenters(pinocchioInterface);
  computeDistances(worldCenters, evaluation.distances.data(), evaluation.normalX.data(), evaluation.normalY.data(),
                   evaluation.normalZ.data());

  for (size_t pairIndex = 0; pairIndex < getNumPairs(); ++pairIndex) {
    const auto first = static_cast<size_t>(firstObject_[pairIndex]);
    const auto second = static_cast<size_t>(secondObject_[pairIndex]);
    const auto firstJoint = parentJoint_[first];
    const auto secondJoint = parentJoint_[second];
    const auto& firstJointPosition = pinocchioInterface.getData().oMi[firstJoint].translation();
    const auto& secondJointPosition = pinocchioInterface.getData().oMi[secondJoint].translation();
    const double firstOffsetX = worldCenters.x[first] - firstJointPosition.x();
    const double firstOffsetY = worldCenters.y[first] - firstJointPosition.y();
    const double firstOffsetZ = worldCenters.z[first] - firstJointPosition.z();
    const double secondOffsetX = worldCenters.x[second] - secondJointPosition.x();
    const double secondOffsetY = worldCenters.y[second] - secondJointPosition.y();
    const double secondOffsetZ = worldCenters.z[second] - secondJointPosition.z();
    evaluation.firstAngularX[pairIndex] = evaluation.normalY[pairIndex] * firstOffsetZ - evaluation.normalZ[pairIndex] * firstOffsetY;
    evaluation.firstAngularY[pairIndex] = evaluation.normalZ[pairIndex] * firstOffsetX - evaluation.normalX[pairIndex] * firstOffsetZ;
    evaluation.firstAngularZ[pairIndex] = evaluation.normalX[pairIndex] * firstOffsetY - evaluation.normalY[pairIndex] * firstOffsetX;
    evaluation.secondAngularX[pairIndex] =
        evaluation.normalY[pairIndex] * secondOffsetZ - evaluation.normalZ[pairIndex] * secondOffsetY;
    evaluation.secondAngularY[pairIndex] =
        evaluation.normalZ[pairIndex] * secondOffsetX - evaluation.normalX[pairIndex] * secondOffsetZ;
    evaluation.secondAngularZ[pairIndex] =
        evaluation.normalX[pairIndex] * secondOffsetY - evaluation.normalY[pairIndex] * secondOffsetX;
  }
}

size_t SphereCollisionModel::getFirstParentJoint(size_t pairIndex) const {
  return parentJoint_[static_cast<size_t>(firstObject_.at(pairIndex))];
}

size_t SphereCollisionModel::getSecondParentJoint(size_t pairIndex) const {
  return parentJoint_[static_cast<size_t>(secondObject_.at(pairIndex))];
}

}  // namespace impl
}  // namespace collision_nextgen
}  // namespace ocs2
