# Nextgen Sphere Self-Collision Optimization

## Scope

`ocs2_self_collision` remains the mesh/FCL reference implementation. Its
runtime path computes mesh distances through `PinocchioGeometryInterface` and
uses nearest-point geometry to form the constraint Jacobian.

The optimized sphere implementation lives in the adjacent
`ocs2_collision_nextgen` package. This document belongs here because the two
packages implement the same self-collision residual and are compared in the
same MPC workflow. It explains what changed in the sphere backend, why those
changes are safe, and how to compare it with the mesh reference. It does not
change the mesh backend or hide backend selection inside this package.

The preserved residual for a pair of collision objects is:

```text
distance - minimumDistance
```

For a sphere pair, the Jacobian uses the same linear and angular terms as the
mesh reference:

```text
n^T (Jv_second - Jv_first)
  - (n x r_second)^T Jw_second
  + (n x r_first)^T Jw_first
```

Here `n` is the distance normal, `r_first` and `r_second` are center offsets
from their parent-joint origins, and `Jv` / `Jw` are the linear / angular rows
of the LOCAL_WORLD_ALIGNED joint Jacobian.

## Baseline Cost

The mesh path is roughly:

```text
Pinocchio state -> forward kinematics -> FCL distance query
                -> nearest points and normal -> Jacobian assembly
                -> OCS2 state/input mapping
```

In an MPC solve, both value evaluation and linearization are hot paths. The
sphere backend keeps the residual and mapping above, but changes the runtime
representation so the distance and Jacobian work can be batched efficiently.

![The mesh/FCL reference and sphere backend preserve the same collision residual before feeding the MPC workflow.](assets/mesh_sphere_mpc_overview.png)

*Figure 1. The mesh/FCL reference and sphere backend differ in representation,
but both feed the same collision residual and MPC workflow.*

## Optimization Techniques

### 1. Preallocated sphere evaluation storage

`SphereCollisionEvaluation` stores pairwise distance normals and angular
coefficients as aligned scalar arrays. `SphereCollisionModel` owns reusable
world-center scratch arrays. `NextgenSelfCollisionConstraint` owns reusable
evaluation, Jacobian, and state-mapping scratch.

This removes per-call allocation of center vectors, normal vectors, per-joint
matrices, and temporary Jacobian containers from
`getLinearApproximation()`. Only active parent joints are queried from
Pinocchio.

The scratch is mutable, so one constraint/model instance must not be evaluated
concurrently. OCS2's clone-per-worker pattern remains the required ownership
model.

### 2. Jacobian layout changed from gather-oriented to contiguous rows

The earlier layout placed joint index as the innermost dimension. It matched
the pair indices but required expensive gathers for each SIMD lane. The current
cache is 32-byte-aligned and laid out as:

```text
[joint][spatial row][padded DoF]
```

The DoF stride is rounded up to four doubles. A four-DoF AVX2 packet can then
load an aligned contiguous Jacobian row. Four pair results are transposed into
the column-major Eigen output, preserving the public `[pair][DoF]` result
layout.

This is the main Jacobian improvement: the reference 7-DoF path has no gather
instructions in its hot loop.

![Joint-major Jacobian cache with four-wide SIMD lanes, consecutive collision-pair runs, and a transpose into the column-major output matrix.](assets/jacobian_simd_layout.png)

*Figure 2. Joint-major storage turns row reads into contiguous SIMD loads;
consecutive pair runs increase reuse before the four-pair result is transposed
into Eigen's column-major output.*

### 3. Dispatch matches the actual problem shape

The production Jacobian entry point chooses among several paths:

| Input shape | Implementation |
|---|---|
| 1 DoF | scalar pair-major specialization |
| 2 DoF | fixed AVX2 packet and transpose path |
| 7 DoF, stride 8, repeated consecutive joint pairs | run-grouped AVX2 path |
| 7 DoF, stride 8, fragmented pairs | ungrouped four-pair transpose fallback |
| all other supported DoF counts | generic AVX2 packet path with scalar pair tails |

The run-grouped path is enabled only when descriptors form a contiguous pair
partition and `numJointRuns <= numPairs / 4`. It loads the two joint Jacobian
blocks once per consecutive run, broadcasts each coefficient, and vectorizes
across four collision pairs. It never sorts pairs or changes output order.

The fallback is deliberate. Fragmented joint pairs do not have enough reuse to
justify run setup, so they retain the original four-pair transpose strategy.

### 4. Distance kernel stays vectorized without changing edge semantics

The sphere distance kernel gathers indexed object centers in four-pair packets,
computes distance, and optionally writes normalized direction vectors. It keeps
a scalar tail and clamps the normal denominator with `1e-12`, which gives a
finite zero subgradient for coincident centers.

Both distance and Jacobian kernels use the generic SIMD wrapper. Compiler
evidence shows their loads, gathers, broadcasts, arithmetic, `sqrt`, `max`,
transposes, permutations, and stores are inlined. Raw AVX2 intrinsics are
confined to the `simd::Vector` specialization; `simd::transpose4x4` and
compile-time `simd::permute` perform unpacking and cross-lane reordering fully
in registers without runtime lane extraction. The Jacobian packet helper is
forced inline: ordinary `inline` left it out of line and regressed the 2-DoF
path, while forced-inline raw and wrapper variants compile to byte-identical
benchmark binaries.

Runtime `Vector::lane(size_t)` is intentionally excluded from hot paths because
it materializes the full packet. `Mask` support is tested for API completeness,
but no production collision kernel currently needs per-lane mask extraction.

## Correctness Guardrails

- Jacobian pair indices are checked in Debug builds; run descriptors are also
  checked for complete coverage and agreement with the pair arrays.
- Padding is never written to the output. Tests cover a legal non-minimal
  `numDofs=3`, `stride=8` input with output canaries.
- Tests cover 1, 2, 3, 5, 6, 7, 8, and 11 DoFs, vector and scalar tails,
  selected run grouping, and singleton runs that force fallback dispatch.
- The 7-DoF Franka reference has 629 sphere pairs, 8 joints, and 17 consecutive
  runs. This is a benchmark shape, not a requirement imposed on other robots.
- Backend selection and sphere URDF policy remain in robot-specific interfaces;
  `ocs2_self_collision` stays available as the mesh/FCL reference for A/B
  comparison.

## Measured Outcome

On the WSL2 Raptor Lake reference host, with the competing Windows VM stopped,
the strict standalone production comparison for the Franka `629/7/8` Jacobian
measured 13958.886 ns for SCALAR and 1981.072 ns for AVX2: 7.046x. The same
binary algorithm-scalar versus production-SIMD comparison measured 5.778x.

The run-aware path itself was 1.491x faster than the ungrouped fallback in the
focused A/B run. This number is intentionally reported below 1.5x rather than
rounded up. It isolates extra reuse from the broader gain due to the new layout
and AVX2 implementation.

The fresh MPC regression gate used 2,592 aligned post-warmup runs. Observation
times matched exactly, cost-diff growth occurred in 52.5 percent of runs, and
the candidate maximum link-pair minimum-distance difference was 3.976837 mm,
under the 4 mm limit.

Full benchmark hashes, assembly/perf evidence, DoF sweep results, and the
precise MPC gate are recorded in
[Collision Nextgen Kernel Results](../../ocs2_collision_nextgen/docs/performance_results.md).

## Reproduction

Build the adjacent implementation and its downstream consumer with the normal
workspace helper:

```bash
cd /home/asterich/code/cpp/ocs2/src/ocs2
tools/build-current.sh ocs2_collision_nextgen ocs2_mobile_manipulator \
  ocs2_mobile_manipulator_ros
```

Run focused correctness tests after sourcing the workspace:

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/openrobots:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=/opt/openrobots/lib:${LD_LIBRARY_PATH}
source /home/asterich/code/cpp/ocs2/install/setup.bash

ctest --test-dir /home/asterich/code/cpp/ocs2/build/ocs2_collision_nextgen \
  --output-on-failure
ctest --test-dir /home/asterich/code/cpp/ocs2/build/ocs2_mobile_manipulator \
  --output-on-failure -R '^(SelfCollisionTest|NextgenSelfCollisionTest|EndEffectorConstraintTest)$'
```

For attributable performance work, use the isolated candidate/baseline
worktrees and fixed-iteration commands documented by
`ocs2_collision_nextgen/tools/README.md`; do not compare timings from an
overlay build with timings from the main workspace build.
