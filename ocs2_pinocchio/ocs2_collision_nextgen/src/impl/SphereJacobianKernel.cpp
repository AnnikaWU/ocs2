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

#include <ocs2_collision_nextgen/impl/SphereJacobianKernel.h>
#include <ocs2_collision_nextgen/impl/simd/Vector.h>

#include <algorithm>
#include <cassert>

namespace ocs2 {
namespace collision_nextgen {
namespace impl {

namespace {

struct KernelData {
  const double *jointJacobians;
  size_t numDofs;
  size_t dofStride;
  const std::int64_t *firstJoint;
  const std::int64_t *secondJoint;
  const double *firstAngularX;
  const double *firstAngularY;
  const double *firstAngularZ;
  const double *secondAngularX;
  const double *secondAngularY;
  const double *secondAngularZ;
  const double *normalX;
  const double *normalY;
  const double *normalZ;
  size_t numPairs;
  double *output;
  const SphereJacobianJointRun *jointRuns;
  size_t numJointRuns;
};

inline double jacobianValue(const KernelData &data, size_t joint, size_t row,
                            size_t dof) {
  return data.jointJacobians[(joint * 6 + row) * data.dofStride + dof];
}

[[maybe_unused]] void computeScalar(const KernelData &data) {
  for (size_t dof = 0; dof < data.numDofs; ++dof) {
    double *outputColumn = data.output + dof * data.numPairs;
    for (size_t pair = 0; pair < data.numPairs; ++pair) {
      const size_t first = static_cast<size_t>(data.firstJoint[pair]);
      const size_t second = static_cast<size_t>(data.secondJoint[pair]);
      const double linear =
          data.normalX[pair] * (jacobianValue(data, second, 0, dof) -
                                jacobianValue(data, first, 0, dof)) +
          data.normalY[pair] * (jacobianValue(data, second, 1, dof) -
                                jacobianValue(data, first, 1, dof)) +
          data.normalZ[pair] * (jacobianValue(data, second, 2, dof) -
                                jacobianValue(data, first, 2, dof));
      const double secondAngular =
          data.secondAngularX[pair] * jacobianValue(data, second, 3, dof) +
          data.secondAngularY[pair] * jacobianValue(data, second, 4, dof) +
          data.secondAngularZ[pair] * jacobianValue(data, second, 5, dof);
      const double firstAngular =
          data.firstAngularX[pair] * jacobianValue(data, first, 3, dof) +
          data.firstAngularY[pair] * jacobianValue(data, first, 4, dof) +
          data.firstAngularZ[pair] * jacobianValue(data, first, 5, dof);
      outputColumn[pair] = linear - secondAngular + firstAngular;
    }
  }
}

template <size_t NumDofs>
void computeScalarPairMajor(const KernelData &data) {
  static_assert(NumDofs == 1 || NumDofs == 2,
                "Low-dimensional scalar specialization supports 1 or 2 dofs");
  for (size_t pair = 0; pair < data.numPairs; ++pair) {
    const size_t first = static_cast<size_t>(data.firstJoint[pair]);
    const size_t second = static_cast<size_t>(data.secondJoint[pair]);
    const double *firstRows =
        data.jointJacobians + first * 6 * data.dofStride;
    const double *secondRows =
        data.jointJacobians + second * 6 * data.dofStride;
    const double normalX = data.normalX[pair];
    const double normalY = data.normalY[pair];
    const double normalZ = data.normalZ[pair];
    const double firstAngularX = data.firstAngularX[pair];
    const double firstAngularY = data.firstAngularY[pair];
    const double firstAngularZ = data.firstAngularZ[pair];
    const double secondAngularX = data.secondAngularX[pair];
    const double secondAngularY = data.secondAngularY[pair];
    const double secondAngularZ = data.secondAngularZ[pair];

    for (size_t dof = 0; dof < NumDofs; ++dof) {
      const double linear =
          normalX * (secondRows[dof] - firstRows[dof]) +
          normalY * (secondRows[data.dofStride + dof] - firstRows[data.dofStride + dof]) +
          normalZ * (secondRows[2 * data.dofStride + dof] - firstRows[2 * data.dofStride + dof]);
      const double second =
          secondAngularX * secondRows[3 * data.dofStride + dof] +
          secondAngularY * secondRows[4 * data.dofStride + dof] +
          secondAngularZ * secondRows[5 * data.dofStride + dof];
      const double first =
          firstAngularX * firstRows[3 * data.dofStride + dof] +
          firstAngularY * firstRows[4 * data.dofStride + dof] +
          firstAngularZ * firstRows[5 * data.dofStride + dof];
      data.output[dof * data.numPairs + pair] = linear - second + first;
    }
  }
}

#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)

using Packet = simd::Vector<double, 4>;

#if defined(_MSC_VER)
#define OCS2_COLLISION_NEXTGEN_FORCE_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#define OCS2_COLLISION_NEXTGEN_FORCE_INLINE inline __attribute__((always_inline))
#else
#define OCS2_COLLISION_NEXTGEN_FORCE_INLINE inline
#endif

/**
 * Builds one four-lane DoF packet for one collision pair. Each lane is the
 * derivative of the same pair-distance residual with respect to one adjacent
 * DoF; no cross-pair transpose happens here.
 *
 *   lane                 0          1          2          3
 *   loaded DoF       dof + 0    dof + 1    dof + 2    dof + 3
 *   returned packet [ g(pair,0) g(pair,1) g(pair,2) g(pair,3) ]
 *
 * Here g(pair,k) abbreviates d distance[pair] / d q[dof + k].
 */
OCS2_COLLISION_NEXTGEN_FORCE_INLINE Packet
computeDofPacket(const KernelData &data, size_t pair, size_t dof) {
  const size_t first = static_cast<size_t>(data.firstJoint[pair]);
  const size_t second = static_cast<size_t>(data.secondJoint[pair]);
  auto row = [&](size_t joint, size_t spatialRow) {
    return Packet::loadAligned(data.jointJacobians +
                               (joint * 6 + spatialRow) * data.dofStride + dof);
  };
  auto coefficient = [](double value) { return Packet::set(value); };

  Packet linear = coefficient(data.normalX[pair]) *
                  (row(second, 0) - row(first, 0));
  linear = linear + coefficient(data.normalY[pair]) *
                        (row(second, 1) - row(first, 1));
  linear = linear + coefficient(data.normalZ[pair]) *
                        (row(second, 2) - row(first, 2));
  Packet secondAngular =
      coefficient(data.secondAngularX[pair]) * row(second, 3);
  secondAngular = secondAngular +
                  coefficient(data.secondAngularY[pair]) * row(second, 4);
  secondAngular = secondAngular +
                  coefficient(data.secondAngularZ[pair]) * row(second, 5);
  Packet firstAngular =
      coefficient(data.firstAngularX[pair]) * row(first, 3);
  firstAngular = firstAngular +
                 coefficient(data.firstAngularY[pair]) * row(first, 4);
  firstAngular = firstAngular +
                 coefficient(data.firstAngularZ[pair]) * row(first, 5);
  return linear - secondAngular + firstAngular;
}

#undef OCS2_COLLISION_NEXTGEN_FORCE_INLINE

/**
 * Transposes four pair-major DoF packets into DoF-major output packets.
 * Every bracket below contains exactly four SIMD lanes.
 *
 * Before transpose4x4(): one input row per collision pair
 *
 *   row0 = [ p0d0 p0d1 p0d2 p0d3 ]
 *   row1 = [ p1d0 p1d1 p1d2 p1d3 ]
 *   row2 = [ p2d0 p2d1 p2d2 p2d3 ]
 *   row3 = [ p3d0 p3d1 p3d2 p3d3 ]
 *
 * After transpose4x4(): one packet per output DoF column
 *
 *   row0 = [ p0d0 p1d0 p2d0 p3d0 ] -> output[dof + 0][pair ... pair + 3]
 *   row1 = [ p0d1 p1d1 p2d1 p3d1 ] -> output[dof + 1][pair ... pair + 3]
 *   row2 = [ p0d2 p1d2 p2d2 p3d2 ] -> output[dof + 2][pair ... pair + 3]
 *   row3 = [ p0d3 p1d3 p2d3 p3d3 ] -> output[dof + 3][pair ... pair + 3]
 *
 * ActiveDofs only controls how many transposed output packets are stored; it
 * does not change the four-lane packet width.
 */
template <size_t ActiveDofs>
inline void storeTransposed(Packet row0, Packet row1, Packet row2,
                            Packet row3, size_t pair, size_t dof,
                            const KernelData &data) {
  static_assert(ActiveDofs >= 1 && ActiveDofs <= 4,
                "A transposed packet stores one to four dofs");
  simd::transpose4x4(row0, row1, row2, row3);
  row0.storeUnaligned(data.output + dof * data.numPairs + pair);
  if constexpr (ActiveDofs >= 2) {
    row1.storeUnaligned(data.output + (dof + 1) * data.numPairs + pair);
  }
  if constexpr (ActiveDofs >= 3) {
    row2.storeUnaligned(data.output + (dof + 2) * data.numPairs + pair);
  }
  if constexpr (ActiveDofs == 4) {
    row3.storeUnaligned(data.output + (dof + 3) * data.numPairs + pair);
  }
}

inline void storeTransposedDynamic(Packet row0, Packet row1, Packet row2,
                                   Packet row3, size_t pair, size_t dof,
                                   const KernelData &data) {
  const size_t activeDofs = std::min<size_t>(4, data.numDofs - dof);
  switch (activeDofs) {
  case 4:
    storeTransposed<4>(row0, row1, row2, row3, pair, dof, data);
    break;
  case 3:
    storeTransposed<3>(row0, row1, row2, row3, pair, dof, data);
    break;
  case 2:
    storeTransposed<2>(row0, row1, row2, row3, pair, dof, data);
    break;
  default:
    storeTransposed<1>(row0, row1, row2, row3, pair, dof, data);
    break;
  }
}

void computeAvx2(const KernelData &data) {
  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    for (size_t dof = 0; dof < data.numDofs; dof += 4) {
      storeTransposedDynamic(computeDofPacket(data, pair, dof),
                             computeDofPacket(data, pair + 1, dof),
                             computeDofPacket(data, pair + 2, dof),
                             computeDofPacket(data, pair + 3, dof), pair, dof,
                             data);
    }
  }

  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    for (size_t dof = 0; dof < data.numDofs; dof += 4) {
      computeDofPacket(data, pair, dof).storeAligned(lanes);
      const size_t activeDofs = std::min<size_t>(4, data.numDofs - dof);
      for (size_t lane = 0; lane < activeDofs; ++lane) {
        data.output[(dof + lane) * data.numPairs + pair] = lanes[lane];
      }
    }
  }
}

/**
 * Specialized two-DoF transpose. Lanes x2 and x3 are padded/ignored DoFs:
 *
 *   row0 = [ p0d0 p0d1  x02  x03 ]
 *   row1 = [ p1d0 p1d1  x12  x13 ]
 *   row2 = [ p2d0 p2d1  x22  x23 ]
 *   row3 = [ p3d0 p3d1  x32  x33 ]
 *
 * Pairwise lane selection:
 *
 *   low01  = [ p0d0 p1d0 x02 x12 ]
 *   high01 = [ p0d1 p1d1 x03 x13 ]
 *   low23  = [ p2d0 p3d0 x22 x32 ]
 *   high23 = [ p2d1 p3d1 x23 x33 ]
 *
 * Final packets written to the two column-major output columns:
 *
 *   dof 0 = [ p0d0 p1d0 p2d0 p3d0 ]
 *   dof 1 = [ p0d1 p1d1 p2d1 p3d1 ]
 */
void computeAvx2Dof2(const KernelData &data) {
  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    const Packet row0 = computeDofPacket(data, pair, 0);
    const Packet row1 = computeDofPacket(data, pair + 1, 0);
    const Packet row2 = computeDofPacket(data, pair + 2, 0);
    const Packet row3 = computeDofPacket(data, pair + 3, 0);
    const Packet low01 = simd::permute<0, 4, 2, 6>(row0, row1);
    const Packet high01 = simd::permute<1, 5, 3, 7>(row0, row1);
    const Packet low23 = simd::permute<0, 4, 2, 6>(row2, row3);
    const Packet high23 = simd::permute<1, 5, 3, 7>(row2, row3);
    simd::permute<0, 1, 4, 5>(low01, low23).storeUnaligned(data.output + pair);
    simd::permute<0, 1, 4, 5>(high01, high23).storeUnaligned(data.output + data.numPairs + pair);
  }

  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    computeDofPacket(data, pair, 0).storeAligned(lanes);
    data.output[pair] = lanes[0];
    data.output[data.numPairs + pair] = lanes[1];
  }
}

/**
 * Seven-DoF specialization. Each four-pair batch uses two four-lane packets:
 *
 *   packet at dof 0 = [ d0 d1 d2 d3 ] -> storeTransposed<4>()
 *   packet at dof 4 = [ d4 d5 d6  x ] -> storeTransposed<3>()
 *
 * The final lane x is stride padding. It participates in the register
 * transpose but is never written to the seven-column output matrix.
 */
void computeAvx2Dof7(const KernelData &data) {
  size_t pair = 0;
  for (; pair + 4 <= data.numPairs; pair += 4) {
    storeTransposed<4>(computeDofPacket(data, pair, 0),
                       computeDofPacket(data, pair + 1, 0),
                       computeDofPacket(data, pair + 2, 0),
                       computeDofPacket(data, pair + 3, 0), pair, 0, data);
    storeTransposed<3>(computeDofPacket(data, pair, 4),
                       computeDofPacket(data, pair + 1, 4),
                       computeDofPacket(data, pair + 2, 4),
                       computeDofPacket(data, pair + 3, 4), pair, 4, data);
  }

  alignas(32) double lanes[4];
  for (; pair < data.numPairs; ++pair) {
    computeDofPacket(data, pair, 0).storeAligned(lanes);
    for (size_t lane = 0; lane < 4; ++lane) {
      data.output[lane * data.numPairs + pair] = lanes[lane];
    }
    computeDofPacket(data, pair, 4).storeAligned(lanes);
    for (size_t lane = 0; lane < 3; ++lane) {
      data.output[(lane + 4) * data.numPairs + pair] = lanes[lane];
    }
  }
}

/**
 * Run-grouped seven-DoF path. A run describes a consecutive pair range whose
 * first and second parent joints are constant:
 *
 *   run [begin, end):
 *     firstJoint  = Jf for every pair
 *     secondJoint = Js for every pair
 *
 * For one DoF d, the joint-Jacobian values are therefore loaded once and
 * broadcast across four consecutive pair lanes:
 *
 *   pair lanes       = [ p0       p1       p2       p3       ]
 *   normalX packet   = [ nx[p0]   nx[p1]   nx[p2]   nx[p3]   ]
 *   Js.linearX(d)    = [ Js.x(d)  Js.x(d)  Js.x(d)  Js.x(d)  ]
 *   Jf.linearX(d)    = [ Jf.x(d)  Jf.x(d)  Jf.x(d)  Jf.x(d)  ]
 *   result packet    = [ g[p0,d]  g[p1,d]  g[p2,d]  g[p3,d]  ]
 *
 * The result packet already has the memory order of one column-major output
 * column, so it is stored directly to output[d][p0 ... p3]. No 4x4 transpose
 * is required. The loop visits d = 0 ... 6; one to three pairs left at the
 * end of a run use the scalar tail with the same preloaded joint values.
 */
void computeAvx2RunGroupedDof7(const KernelData &data) {

  // jointRuns partitions the original pair order without permuting it. Within
  // one run, every pair refers to the same two parent-joint Jacobians:
  //
  //   pair index    begin  begin+1  begin+2  ...  end-1
  //   firstJoint      Jf      Jf       Jf           Jf
  //   secondJoint     Js      Js       Js           Js
  //
  // Jf and Js may therefore be reused until the run boundary at end.
  for (size_t runIndex = 0; runIndex < data.numJointRuns; ++runIndex) {
    const auto &run = data.jointRuns[runIndex];
    const size_t runEnd = run.begin + run.length;

    // The DoF7 dispatch guarantees dofStride == 8. Each joint owns six
    // spatial rows, and each row contains seven active values plus padding:
    //
    //                  one spatial row, eight stored doubles
    //                +-----------------------------------------+
    //   row 0, Lx ->  | d0 | d1 | d2 | d3 | d4 | d5 | d6 | x |
    //   row 1, Ly ->  | d0 | d1 | d2 | d3 | d4 | d5 | d6 | x |
    //        ...
    //   row 5, Az ->  | d0 | d1 | d2 | d3 | d4 | d5 | d6 | x |
    //                +-----------------------------------------+
    //
    //   Jf(r,d) = firstRows[r * dofStride + d]
    //   Js(r,d) = secondRows[r * dofStride + d]
    //
    // L denotes a linear row, A an angular row, and x is unused padding.
    const double *firstRows =
        data.jointJacobians + run.firstJoint * 6 * data.dofStride;
    const double *secondRows =
        data.jointJacobians + run.secondJoint * 6 * data.dofStride;

    // This specialization traverses the output in DoF-major order. It consumes
    // the complete run for one DoF before advancing to the next DoF:
    //
    //                  pair direction within this run --->
    //   dof 0   [ begin ... begin+3 ] [ begin+4 ... begin+7 ] ... [ tail ]
    //   dof 1   [ begin ... begin+3 ] [ begin+4 ... begin+7 ] ... [ tail ]
    //    ...
    //   dof 6   [ begin ... begin+3 ] [ begin+4 ... begin+7 ] ... [ tail ]
    //
    // Consequently, the nine joint values below are invariant for the entire
    // inner pair loop and are read only once per DoF.
    for (size_t dof = 0; dof < 7; ++dof) {
      const double linearX = secondRows[dof] - firstRows[dof];
      const double linearY = secondRows[data.dofStride + dof] -
                             firstRows[data.dofStride + dof];
      const double linearZ = secondRows[2 * data.dofStride + dof] -
                             firstRows[2 * data.dofStride + dof];
      const double firstAngularX = firstRows[3 * data.dofStride + dof];
      const double firstAngularY = firstRows[4 * data.dofStride + dof];
      const double firstAngularZ = firstRows[5 * data.dofStride + dof];
      const double secondAngularX = secondRows[3 * data.dofStride + dof];
      const double secondAngularY = secondRows[4 * data.dofStride + dof];
      const double secondAngularZ = secondRows[5 * data.dofStride + dof];

      // Linears and angulars are broadcast to four-lane packets.
      // g(pair,dof) = dot(n(pair), Js.L(dof) - Jf.L(dof))
      //                        - dot(n(pair), Js.A(dof))
      const Packet linearXPacket = Packet::set(linearX);
      const Packet linearYPacket = Packet::set(linearY);
      const Packet linearZPacket = Packet::set(linearZ);
      const Packet firstAngularXPacket = Packet::set(firstAngularX);
      const Packet firstAngularYPacket = Packet::set(firstAngularY);
      const Packet firstAngularZPacket = Packet::set(firstAngularZ);
      const Packet secondAngularXPacket = Packet::set(secondAngularX);
      const Packet secondAngularYPacket = Packet::set(secondAngularY);
      const Packet secondAngularZPacket = Packet::set(secondAngularZ);

      // output points at column dof of the logical [pair][dof] matrix. A store
      // at pair p is already contiguous and has exactly the required order:
      //
      //   SIMD lane         0         1         2         3
      //   pair              p       p + 1     p + 2     p + 3
      //   packet       [ g(p,d)  g(p+1,d)  g(p+2,d)  g(p+3,d) ]
      //                         | direct store |
      //   memory       output[p ... p + 3]
      //
      //   offset(g(q,d)) = d * numPairs + q
      //
      // No pair/DoF transpose is needed on this path.
      double *output = data.output + dof * data.numPairs;
      size_t pair = run.begin;
      for (; pair + 4 <= runEnd; pair += 4) {

        // Pair-dependent coefficients use the same four-pair lane mapping:
        //
        //   normalX load = [ nx(p)  nx(p+1)  nx(p+2)  nx(p+3) ]
        //   deltaLx      = [   dx      dx       dx       dx    ]
        //   product      = [ nx(p)dx nx(p+1)dx nx(p+2)dx nx(p+3)dx ]
        //
        // For lane k, let q = p + k, with k in {0, 1, 2, 3}. Define
        //
        //   n(q)       = (normalX[q], normalY[q], normalZ[q])
        //   firstA(q)  = (firstAngularX[q], firstAngularY[q],
        //                 firstAngularZ[q])
        //   secondA(q) = (secondAngularX[q], secondAngularY[q],
        //                 secondAngularZ[q])
        //
        // The lane-wise Jacobian formula is
        //
        //   g(q,d) = dot(n(q), Js.L(d) - Jf.L(d))
        //          - dot(secondA(q), Js.A(d))
        //          + dot(firstA(q), Jf.A(d))
        Packet linear = Packet::loadUnaligned(data.normalX + pair) * linearXPacket;
        linear = linear + Packet::loadUnaligned(data.normalY + pair) * linearYPacket;
        linear = linear + Packet::loadUnaligned(data.normalZ + pair) * linearZPacket;

        Packet secondAngular = Packet::loadUnaligned(data.secondAngularX + pair) * secondAngularXPacket;
        secondAngular = secondAngular + Packet::loadUnaligned(data.secondAngularY + pair) * secondAngularYPacket;
        secondAngular = secondAngular + Packet::loadUnaligned(data.secondAngularZ + pair) * secondAngularZPacket;

        Packet firstAngular = Packet::loadUnaligned(data.firstAngularX + pair) * firstAngularXPacket;
        firstAngular = firstAngular + Packet::loadUnaligned(data.firstAngularY + pair) * firstAngularYPacket;
        firstAngular = firstAngular + Packet::loadUnaligned(data.firstAngularZ + pair) * firstAngularZPacket;
        (linear - secondAngular + firstAngular).storeUnaligned(output + pair);
      }

      // A run need not end on a packet boundary. For a run of eleven pairs:
      //
      //   SIMD packet 0       SIMD packet 1          scalar tail
      //   [ b b+1 b+2 b+3 ]   [ b+4 b+5 b+6 b+7 ]   [ b+8 b+9 b+10 ]
      //
      // Here tailLength = 11 % 4 = 3.
      //
      // The tail reuses the same per-DoF scalars but avoids an over-read.
      for (; pair < runEnd; ++pair) {
        const double linear = data.normalX[pair] * linearX + data.normalY[pair] * linearY +
                              data.normalZ[pair] * linearZ;
        const double secondAngular = data.secondAngularX[pair] * secondAngularX +
                                     data.secondAngularY[pair] * secondAngularY +
                                     data.secondAngularZ[pair] * secondAngularZ;
        const double firstAngular = data.firstAngularX[pair] * firstAngularX +
                                    data.firstAngularY[pair] * firstAngularY +
                                    data.firstAngularZ[pair] * firstAngularZ;
        output[pair] = linear - secondAngular + firstAngular;
      }
    }
  }
}

/**
 * Generic run-grouped path for arbitrary DoF counts. It uses the same lane
 * layout as the seven-DoF specialization:
 *
 *   coefficient packet for pairs p0 ... p3
 *
 *     normalX       = [ nx[p0]       nx[p1]       nx[p2]       nx[p3]       ]
 *     firstAngularX = [ firstAx[p0]  firstAx[p1]  firstAx[p2]  firstAx[p3]  ]
 *
 *   one joint-Jacobian scalar for DoF d, broadcast to all pair lanes
 *
 *     Jf.angularX(d) = [ Jf.ax(d) Jf.ax(d) Jf.ax(d) Jf.ax(d) ]
 *
 *   direct column-major result
 *
 *     result(d)      = [ g[p0,d]  g[p1,d]  g[p2,d]  g[p3,d]  ]
 *                       |_____________________________________|
 *                         output[d][p0 ... p3], no transpose
 *
 * Unlike computeAvx2RunGroupedDof7(), this function loads the pair-coefficient
 * packets once per four-pair batch and then iterates over every DoF. The run's
 * fixed Jf/Js row bases are reused throughout. A scalar loop handles the final
 * one to three pairs of each run without changing pair order.
 */
void computeAvx2RunGrouped(const KernelData &data) {

  // As in the DoF7 specialization, each run keeps pair order unchanged and
  // fixes both parent joints over [run.begin, run.begin + run.length).
  for (size_t runIndex = 0; runIndex < data.numJointRuns; ++runIndex) {
    const auto &run = data.jointRuns[runIndex];
    const size_t runEnd = run.begin + run.length;

    // The generic joint block is six rows by dofStride. Padding, if present,
    // follows the active DoFs in every spatial row:
    //
    //                     dofStride stored doubles
    //                  +-----------------------------------+
    //   row 0, Lx  ->  | d0 | d1 | ... | d(N-1) | padding |
    //   row 1, Ly  ->  | d0 | d1 | ... | d(N-1) | padding |
    //        ...
    //   row 5, Az  ->  | d0 | d1 | ... | d(N-1) | padding |
    //                  +-----------------------------------+
    //
    //   Jf(r,d) = firstRows[r * dofStride + d]
    //   Js(r,d) = secondRows[r * dofStride + d]
    const double *firstRows = data.jointJacobians + run.firstJoint * 6 * data.dofStride;
    const double *secondRows = data.jointJacobians + run.secondJoint * 6 * data.dofStride;

    // The generic path reverses the two inner loops used by the DoF7 path. One
    // four-pair batch is completed for every DoF before loading the next batch:
    //
    //   pairs [ begin   ... begin+3 ]:  dof 0 -> dof 1 -> ... -> dof N-1
    //   pairs [ begin+4 ... begin+7 ]:  dof 0 -> dof 1 -> ... -> dof N-1
    //                    ...
    //
    // Thus the nine pair-coefficient packets are loaded once per batch and
    // reused by the complete inner DoF loop.
    size_t pair = run.begin;
    for (; pair + 4 <= runEnd; pair += 4) {
      const Packet normalX = Packet::loadUnaligned(data.normalX + pair);
      const Packet normalY = Packet::loadUnaligned(data.normalY + pair);
      const Packet normalZ = Packet::loadUnaligned(data.normalZ + pair);
      const Packet firstAngularX = Packet::loadUnaligned(data.firstAngularX + pair);
      const Packet firstAngularY = Packet::loadUnaligned(data.firstAngularY + pair);
      const Packet firstAngularZ = Packet::loadUnaligned(data.firstAngularZ + pair);
      const Packet secondAngularX = Packet::loadUnaligned(data.secondAngularX + pair);
      const Packet secondAngularY = Packet::loadUnaligned(data.secondAngularY + pair);
      const Packet secondAngularZ = Packet::loadUnaligned(data.secondAngularZ + pair);

      // For each DoF, a scalar from Jf/Js is broadcast across the fixed pair
      // lanes and produces one output-column packet:
      //
      //                         dof d packet       dof d+1 packet
      //   pair p,   lane 0       g(p,d)              g(p,d+1)
      //   pair p+1, lane 1       g(p+1,d)            g(p+1,d+1)
      //   pair p+2, lane 2       g(p+2,d)            g(p+2,d+1)
      //   pair p+3, lane 3       g(p+3,d)            g(p+3,d+1)
      //                          |                    |
      //   store address     output+d*P+p       output+(d+1)*P+p
      //
      // P is numPairs. Each vertical packet is already contiguous in Eigen's
      // column-major [pair][dof] storage, so this path also needs no transpose.
      // For lane k, q = p + k and k is one of {0, 1, 2, 3}:
      //
      //   deltaLx(d) = Js.Lx(d) - Jf.Lx(d)
      //   deltaLy(d) = Js.Ly(d) - Jf.Ly(d)
      //   deltaLz(d) = Js.Lz(d) - Jf.Lz(d)
      //
      //   linear[k] = normalX[q] * deltaLx(d)
      //             + normalY[q] * deltaLy(d)
      //             + normalZ[q] * deltaLz(d)
      //
      //   second[k] = secondAngularX[q] * Js.Ax(d)
      //             + secondAngularY[q] * Js.Ay(d)
      //             + secondAngularZ[q] * Js.Az(d)
      //
      //   first[k]  = firstAngularX[q] * Jf.Ax(d)
      //             + firstAngularY[q] * Jf.Ay(d)
      //             + firstAngularZ[q] * Jf.Az(d)
      //
      //   g(q,d) = linear[k] - second[k] + first[k]
      //   offset(g(q,d)) = d * P + q
      for (size_t dof = 0; dof < data.numDofs; ++dof) {
        Packet linear = normalX * Packet::set(secondRows[dof] - firstRows[dof]);
        linear = linear + normalY * Packet::set(secondRows[data.dofStride + dof] - firstRows[data.dofStride + dof]);
        linear = linear + normalZ * Packet::set(secondRows[2 * data.dofStride + dof] - firstRows[2 * data.dofStride + dof]);
        Packet second = secondAngularX * Packet::set(secondRows[3 * data.dofStride + dof]);
        second = second + secondAngularY * Packet::set(secondRows[4 * data.dofStride + dof]);
        second = second + secondAngularZ * Packet::set(secondRows[5 * data.dofStride + dof]);
        Packet first = firstAngularX * Packet::set(firstRows[3 * data.dofStride + dof]);
        first = first + firstAngularY * Packet::set(firstRows[4 * data.dofStride + dof]);
        first = first + firstAngularZ * Packet::set(firstRows[5 * data.dofStride + dof]);
        (linear - second + first).storeUnaligned(data.output + dof * data.numPairs + pair);
      }
    }

    // Let r = run.length % 4 and packetEnd = runEnd - r. The split is
    //
    //   [ run.begin ........ packetEnd ) [ packetEnd ........ runEnd )
    //   |<---- groups of four SIMD pairs ---->| |<-- r scalar pairs -->|
    //
    // The scalar loop retains the same pair-major/DoF-inner traversal.
    for (; pair < runEnd; ++pair) {
      for (size_t dof = 0; dof < data.numDofs; ++dof) {
        const double linear =
            data.normalX[pair] * (secondRows[dof] - firstRows[dof]) +
            data.normalY[pair] * (secondRows[data.dofStride + dof] - firstRows[data.dofStride + dof]) +
            data.normalZ[pair] * (secondRows[2 * data.dofStride + dof] - firstRows[2 * data.dofStride + dof]);
        const double second =
            data.secondAngularX[pair] * secondRows[3 * data.dofStride + dof] +
            data.secondAngularY[pair] * secondRows[4 * data.dofStride + dof] +
            data.secondAngularZ[pair] * secondRows[5 * data.dofStride + dof];
        const double first =
            data.firstAngularX[pair] * firstRows[3 * data.dofStride + dof] +
            data.firstAngularY[pair] * firstRows[4 * data.dofStride + dof] +
            data.firstAngularZ[pair] * firstRows[5 * data.dofStride + dof];
        data.output[dof * data.numPairs + pair] = linear - second + first;
      }
    }
  }
}

#endif

} // namespace

void computeSpherePairJacobians(
    const double *jointJacobians, size_t numJoints, size_t numDofs,
    size_t jointJacobianDofStride, const std::int64_t *firstJoint,
    const std::int64_t *secondJoint, const double *firstAngularX,
    const double *firstAngularY, const double *firstAngularZ,
    const double *secondAngularX, const double *secondAngularY,
    const double *secondAngularZ, const double *normalX, const double *normalY,
    const double *normalZ, size_t numPairs, double *output,
    const SphereJacobianJointRun *jointRuns, size_t numJointRuns) {
  assert(reinterpret_cast<std::uintptr_t>(jointJacobians) % 32 == 0);
  assert(jointJacobianDofStride >= numDofs && jointJacobianDofStride % 4 == 0);
#ifndef NDEBUG
  if (numPairs > 0) {
    assert(firstJoint != nullptr && secondJoint != nullptr);
    for (size_t pair = 0; pair < numPairs; ++pair) {
      assert(firstJoint[pair] >= 0);
      assert(secondJoint[pair] >= 0);
      assert(static_cast<size_t>(firstJoint[pair]) < numJoints);
      assert(static_cast<size_t>(secondJoint[pair]) < numJoints);
    }
  }
  if (numJointRuns > 0) {
    assert(jointRuns != nullptr);
    size_t nextPair = 0;
    for (size_t runIndex = 0; runIndex < numJointRuns; ++runIndex) {
      const auto &run = jointRuns[runIndex];
      assert(run.begin == nextPair);
      assert(run.begin <= numPairs);
      assert(run.length > 0 && run.length <= numPairs - run.begin);
      assert(run.firstJoint < numJoints && run.secondJoint < numJoints);
      const size_t runEnd = run.begin + run.length;
      for (size_t pair = run.begin; pair < runEnd; ++pair) {
        assert(firstJoint[pair] == static_cast<std::int64_t>(run.firstJoint));
        assert(secondJoint[pair] ==
               static_cast<std::int64_t>(run.secondJoint));
      }
      nextPair = runEnd;
    }
    assert(nextPair == numPairs);
  }
#endif
  (void)numJoints;
  const KernelData data{jointJacobians, numDofs,        jointJacobianDofStride,
                        firstJoint,     secondJoint,    firstAngularX,
                        firstAngularY,  firstAngularZ,  secondAngularX,
                        secondAngularY, secondAngularZ, normalX,
                        normalY,        normalZ,        numPairs,
                        output,         jointRuns,      numJointRuns};
  if (numDofs == 1) {
    computeScalarPairMajor<1>(data);
    return;
  }
#if defined(OCS2_COLLISION_NEXTGEN_USE_AVX2)
  if (numDofs == 2) {
    computeAvx2Dof2(data);
    return;
  }
  const bool useRunGrouped =
      jointRuns != nullptr && numJointRuns > 0 && numJointRuns <= numPairs / 4;
  if (useRunGrouped) {
    if (numDofs == 7 && jointJacobianDofStride == 8) {
      computeAvx2RunGroupedDof7(data);
    } else {
      computeAvx2RunGrouped(data);
    }
  } else if (numDofs == 7 && jointJacobianDofStride == 8) {
    computeAvx2Dof7(data);
  } else {
    computeAvx2(data);
  }
#else
  if (numDofs == 2) {
    computeScalarPairMajor<2>(data);
  } else {
    computeScalar(data);
  }
#endif
}

} // namespace impl
} // namespace collision_nextgen
} // namespace ocs2
