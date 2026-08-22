# Collision Kernel Measurement Gates

These tools are independent of the kernel implementation and CMake targets. They are intended to compare a clean
`dev/collision_nextgen_debugging` build with a candidate built from the same branch plus the kernel patch.

## Build overlays

Use separate build and install directories. Explicit robotpkg package paths are required; a plain temporary colcon build can silently select the
ROS Pinocchio 3.8 packages instead of `/opt/openrobots` Pinocchio 3.9.

Create a detached baseline worktree before applying candidate changes. The candidate worktree remains
`/tmp/ocs2_collision_nextgen_debugging`.

```bash
git -C /home/asterich/code/cpp/ocs2/src/ocs2 worktree add --detach \
  /tmp/ocs2_collision_nextgen_debugging-baseline a6afc3297
```

Set these three paths to the baseline worktree and then to the candidate worktree, running the same build command for each:

```bash
source /opt/ros/jazzy/setup.bash
export CMAKE_PREFIX_PATH=/opt/openrobots:${CMAKE_PREFIX_PATH}
export LD_LIBRARY_PATH=/opt/openrobots/lib:${LD_LIBRARY_PATH}
source /home/asterich/code/cpp/ocs2/install/setup.bash

SOURCE_ROOT=/tmp/ocs2_collision_nextgen_debugging-baseline
BUILD_ROOT=/tmp/ocs2_collision_nextgen_debugging-baseline-build
INSTALL_ROOT=/tmp/ocs2_collision_nextgen_debugging-baseline-install

# For the candidate build, use these instead:
# SOURCE_ROOT=/tmp/ocs2_collision_nextgen_debugging
# BUILD_ROOT=/tmp/ocs2_collision_nextgen_debugging-candidate-build
# INSTALL_ROOT=/tmp/ocs2_collision_nextgen_debugging-candidate-install

colcon build \
  --base-paths "${SOURCE_ROOT}" \
  --build-base "${BUILD_ROOT}" \
  --install-base "${INSTALL_ROOT}" \
  --packages-select ocs2_collision_nextgen ocs2_mobile_manipulator ocs2_mobile_manipulator_ros \
  --parallel-workers 1 \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DPython3_EXECUTABLE=/home/asterich/code/cpp/ocs2/.venv/bin/python3 \
    -Dpinocchio_DIR=/opt/openrobots/lib/cmake/pinocchio \
    -Dhpp-fcl_DIR=/opt/openrobots/lib/cmake/hpp-fcl \
    -Dcoal_DIR=/opt/openrobots/lib/cmake/coal \
    -DCMAKE_CXX_FLAGS=-fno-omit-frame-pointer
```

Confirm the cache before accepting measurements:

```bash
rg '^(pinocchio|hpp-fcl|coal)_DIR' \
  "${BUILD_ROOT}/ocs2_collision_nextgen/CMakeCache.txt"
```

## MPC rollout gate

The debug branch configures `selfCollision.backend = debug`. It runs the nextgen sphere backend while replaying every accepted MPC trajectory
through the original mesh/FCL backend. The output is `/tmp/ocs2_self_collision_debug_mpc.tsv`.

Run baseline and candidate overlays separately with the same task, CPU set, duration, and generated-library directory. Copy the TSV after each run
because the probe truncates it at startup.

```bash
source /opt/ros/jazzy/setup.bash
source /home/asterich/code/cpp/ocs2/install/setup.bash
source /tmp/ocs2_collision_nextgen_debugging-baseline-install/setup.bash

ROS_LOG_DIR=/tmp/ocs2_collision_nextgen_ros_logs \
timeout --signal=INT 60s \
numactl --physcpubind=0-7 --membind=0 \
ros2 launch ocs2_mobile_manipulator_ros manipulator_franka.launch.py rviz:=false

cp /tmp/ocs2_self_collision_debug_mpc.tsv /tmp/results/baseline.mpc.tsv
```

Repeat with `/tmp/ocs2_collision_nextgen_debugging-candidate-install/setup.bash` sourced last and copy the candidate TSV to
`/tmp/results/candidate.mpc.tsv`, then gate the captured files:

```bash
python3 ocs2_pinocchio/ocs2_collision_nextgen/tools/check_mpc_regression.py \
  /tmp/results/baseline.mpc.tsv \
  /tmp/results/candidate.mpc.tsv \
  --skip-first 10 \
  --minimum-common-runs 100 \
  --max-link-pair-distance 0.004
```

Rows are aligned by `mpc_run`. `total_cost_signed_diff` and `total_cost_abs_diff` fail when the candidate is above baseline outside numerical
tolerance in at least 95% of aligned runs. `max_link_pair_min_abs_diff` is a hard candidate maximum in meters; its default gate is 4 mm. The
link-pair metric is only defined when `collisionObjectPairs` is empty. Any non-finite value or non-`ok` row fails by default.

## Microbenchmark ratios

Calibrate an iteration count once, then use that same fixed count for every
baseline and candidate variant. Pin the benchmark to one logical CPU and
retain repetition aggregates as JSON. The `50000x` below is the calibrated
count for the 629-pair Raptor Lake reference run, not a portable default:

```bash
numactl --physcpubind=0 --membind=0 \
/tmp/ocs2_collision_nextgen_debugging-candidate-build/ocs2_collision_nextgen/SphereKernelsBenchmark \
  --benchmark_min_time=50000x \
  --benchmark_repetitions=9 \
  --benchmark_enable_random_interleaving=true \
  --benchmark_report_aggregates_only=true \
  --benchmark_out=/tmp/results/kernels.json \
  --benchmark_out_format=json

python3 ocs2_pinocchio/ocs2_collision_nextgen/tools/summarize_benchmark.py \
  /tmp/results/kernels.json \
  --minimum-speedup 'JacobianFixture.*/629/7/8=1.5'
```

The `629/7/8` fixture reproduces the Franka sphere-pair order and its
consecutive joint-pair runs. `JacobianFixture/Simd` is the production
run-aware dispatch; `JacobianFixture/Ungrouped` is the production transpose
fallback used for fragmented or low-reuse inputs. Keep both in A/B runs when
changing run descriptors or the dispatch threshold.

On this WSL2 Raptor Lake host, `perf stat` reports hardware events such as cycles and instructions as `not supported`. Software sampling remains
usable for hotspot attribution:

```bash
perf record -e cpu-clock -F 999 -g --call-graph dwarf -o /tmp/results/kernels.perf.data -- \
  /tmp/ocs2_collision_nextgen_debugging-candidate-build/ocs2_collision_nextgen/SphereKernelsBenchmark \
  --benchmark_filter=Jacobian --benchmark_min_time=10s
perf report -i /tmp/results/kernels.perf.data
```

Capture hashes, the benchmark Build ID, symbol tables, and current object
disassembly from the same package build used for timing:

```bash
ocs2_pinocchio/ocs2_collision_nextgen/tools/capture_kernel_evidence.sh \
  /tmp/ocs2_collision_nextgen_debugging-candidate-build/ocs2_collision_nextgen \
  /tmp/results/kernel-evidence
```

The script fails if an out-of-line `simd::Vector` or `simd::Mask` symbol is
present. It also records gather instructions and 128/256-bit stack references
for review; those files are evidence, not automatic performance gates.

Both parsers have dependency-free smoke tests:

```bash
python3 ocs2_pinocchio/ocs2_collision_nextgen/tools/check_mpc_regression.py --self-test
python3 ocs2_pinocchio/ocs2_collision_nextgen/tools/summarize_benchmark.py --self-test
```
