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

#pragma once

#include <array>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <pinocchio/multibody/fwd.hpp>

namespace ocs2 {
namespace collision_nextgen {

enum class GeometryPrimitiveType { Unsupported, Sphere };

struct CollisionObjectCache {
  std::string name;
  std::string parentFrameName;
  size_t geometryIndex = 0;
  size_t parentFrame = 0;
  size_t parentJoint = 0;
  GeometryPrimitiveType primitiveType = GeometryPrimitiveType::Unsupported;
  std::array<scalar_t, 3> localPosition{0.0, 0.0, 0.0};
  scalar_t sphereRadius = 0.0;
};

struct CollisionPairCache {
  size_t firstObject = 0;
  size_t secondObject = 0;
  size_t firstParentFrame = 0;
  size_t secondParentFrame = 0;
  size_t firstParentJoint = 0;
  size_t secondParentJoint = 0;
};

struct CollisionModelCache {
  std::vector<CollisionObjectCache> objects;
  std::vector<CollisionPairCache> pairs;
  size_t numSphereObjects = 0;
  size_t numUnsupportedObjects = 0;

  size_t getNumCollisionPairs() const { return pairs.size(); }
  bool isSphereOnly() const { return numUnsupportedObjects == 0; }
};

/**
 * Cached data extracted after URDF geometry loading and collision-link expansion.
 *
 * The cache intentionally stores state-independent data only: object/frame/joint
 * indices, local sphere parameters, and expanded object pairs. World placements,
 * nearest points, and Jacobian workspaces belong to per-evaluation scratch data.
 */
CollisionModelCache buildCollisionModelCache(const pinocchio::Model& model, const pinocchio::GeometryModel& geometryModel);

std::string toString(GeometryPrimitiveType type);

}  // namespace collision_nextgen
}  // namespace ocs2
