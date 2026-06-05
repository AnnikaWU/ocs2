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

#include <ocs2_collision_nextgen/CollisionModelCache.h>

#include <hpp/fcl/collision_utility.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2 {
namespace collision_nextgen {

namespace {

GeometryPrimitiveType getPrimitiveType(const pinocchio::GeometryObject& object) {
  if (object.geometry == nullptr) {
    return GeometryPrimitiveType::Unsupported;
  }

  if (object.geometry->getNodeType() == hpp::fcl::GEOM_SPHERE) {
    return GeometryPrimitiveType::Sphere;
  }

  return GeometryPrimitiveType::Unsupported;
}

scalar_t getSphereRadius(const pinocchio::GeometryObject& object) {
  if (object.geometry == nullptr || object.geometry->getNodeType() != hpp::fcl::GEOM_SPHERE) {
    return 0.0;
  }

  const auto* sphere = dynamic_cast<const hpp::fcl::Sphere*>(object.geometry.get());
  if (sphere == nullptr) {
    return 0.0;
  }
  return sphere->radius;
}

}  // namespace

CollisionModelCache buildCollisionModelCache(const pinocchio::Model& model, const pinocchio::GeometryModel& geometryModel) {
  CollisionModelCache cache;
  cache.objects.reserve(geometryModel.geometryObjects.size());
  cache.pairs.reserve(geometryModel.collisionPairs.size());

  for (size_t i = 0; i < geometryModel.geometryObjects.size(); ++i) {
    const auto& object = geometryModel.geometryObjects[i];

    CollisionObjectCache objectCache;
    objectCache.name = object.name;
    objectCache.geometryIndex = i;
    objectCache.parentFrame = object.parentFrame;
    objectCache.parentJoint = object.parentJoint;
    objectCache.primitiveType = getPrimitiveType(object);
    const auto localPosition = object.placement.translation();
    objectCache.localPosition = {localPosition.x(), localPosition.y(), localPosition.z()};
    objectCache.sphereRadius = getSphereRadius(object);

    if (object.parentFrame < model.frames.size()) {
      objectCache.parentFrameName = model.frames[object.parentFrame].name;
    }

    if (objectCache.primitiveType == GeometryPrimitiveType::Sphere) {
      ++cache.numSphereObjects;
    } else {
      ++cache.numUnsupportedObjects;
    }

    cache.objects.push_back(std::move(objectCache));
  }

  for (const auto& pair : geometryModel.collisionPairs) {
    const size_t first = pair.first;
    const size_t second = pair.second;

    CollisionPairCache pairCache;
    pairCache.firstObject = first;
    pairCache.secondObject = second;

    if (first < cache.objects.size()) {
      pairCache.firstParentFrame = cache.objects[first].parentFrame;
      pairCache.firstParentJoint = cache.objects[first].parentJoint;
    }
    if (second < cache.objects.size()) {
      pairCache.secondParentFrame = cache.objects[second].parentFrame;
      pairCache.secondParentJoint = cache.objects[second].parentJoint;
    }

    cache.pairs.push_back(pairCache);
  }

  return cache;
}

std::string toString(GeometryPrimitiveType type) {
  switch (type) {
    case GeometryPrimitiveType::Sphere:
      return "sphere";
    case GeometryPrimitiveType::Unsupported:
      return "unsupported";
  }
  return "unsupported";
}

}  // namespace collision_nextgen
}  // namespace ocs2
