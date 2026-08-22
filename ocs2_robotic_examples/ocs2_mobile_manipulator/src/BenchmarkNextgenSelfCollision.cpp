#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_collision_nextgen/impl/SphereCollisionModel.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorNextgenSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/constraint/MobileManipulatorSelfCollisionConstraint.h"
#include "ocs2_mobile_manipulator/package_path.h"

namespace {

struct Settings {
  std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/franka/task.info";
  std::string collisionUrdfFile = ocs2::mobile_manipulator::getPath() + "/config/franka/panda_collision_spheres.urdf";
  size_t iterations = 10000;
  size_t warmupIterations = 100;
  size_t constraintIterations = 1000;
  size_t constraintWarmupIterations = 20;
};

struct TimedResult {
  std::string name;
  size_t outputs = 0;
  size_t iterations = 0;
  double totalMs = 0.0;
  double averageUs = 0.0;
  double checksum = 0.0;
};

struct ErrorStats {
  std::string quantity;
  size_t count = 0;
  double maxAbs = 0.0;
  double meanAbs = 0.0;
};

ErrorStats compareScalar(const std::string& quantity, double lhs, double rhs) {
  const double absError = std::abs(lhs - rhs);
  return ErrorStats{quantity, 1, absError, absError};
}

void printUsage() {
  std::cerr << "Usage:\n"
            << "  ocs2_benchmark_nextgen_self_collision [options]\n\n"
            << "Options:\n"
            << "  --task <task.info>              Task file. Default: franka/task.info\n"
            << "  --collision-urdf <urdf>         Sphere collision URDF. Default: franka/panda_collision_spheres.urdf\n"
            << "  --iterations <n>                Timed collision-only calls. Default: 10000\n"
            << "  --warmup <n>                    Warmup collision-only calls per mode. Default: 100\n"
            << "  --constraint-iterations <n>     Timed constraint value/linear calls. Default: 1000\n"
            << "  --constraint-warmup <n>         Warmup constraint calls per mode. Default: 20\n";
}

std::string requireValue(int argc, char* argv[], int& index, const std::string& name) {
  if (index + 1 >= argc) {
    throw std::runtime_error("Missing value for " + name);
  }
  return argv[++index];
}

Settings parseArgs(int argc, char* argv[]) {
  Settings settings;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--task") {
      settings.taskFile = requireValue(argc, argv, i, arg);
    } else if (arg == "--collision-urdf") {
      settings.collisionUrdfFile = requireValue(argc, argv, i, arg);
    } else if (arg == "--iterations") {
      settings.iterations = static_cast<size_t>(std::stoul(requireValue(argc, argv, i, arg)));
    } else if (arg == "--warmup") {
      settings.warmupIterations = static_cast<size_t>(std::stoul(requireValue(argc, argv, i, arg)));
    } else if (arg == "--constraint-iterations") {
      settings.constraintIterations = static_cast<size_t>(std::stoul(requireValue(argc, argv, i, arg)));
    } else if (arg == "--constraint-warmup") {
      settings.constraintWarmupIterations = static_cast<size_t>(std::stoul(requireValue(argc, argv, i, arg)));
    } else if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }
  if (settings.iterations == 0 || settings.constraintIterations == 0) {
    throw std::runtime_error("--iterations and --constraint-iterations must be greater than zero");
  }
  return settings;
}

std::string resolvePathRelativeToTaskFile(const std::string& path, const std::string& taskFile) {
  if (path.empty()) {
    return path;
  }
  const std::filesystem::path inputPath(path);
  if (inputPath.is_absolute()) {
    return inputPath.lexically_normal().string();
  }
  return (std::filesystem::path(taskFile).parent_path() / inputPath).lexically_normal().string();
}

ocs2::vector_t loadInitialState(const std::string& taskFile, const ocs2::mobile_manipulator::ManipulatorModelInfo& modelInfo) {
  ocs2::vector_t initialState = ocs2::vector_t::Zero(modelInfo.stateDim);
  const int baseStateDim = static_cast<int>(modelInfo.stateDim - modelInfo.armDim);
  const int armStateDim = static_cast<int>(modelInfo.armDim);

  if (baseStateDim > 0) {
    ocs2::vector_t initialBaseState = ocs2::vector_t::Zero(baseStateDim);
    ocs2::loadData::loadEigenMatrix(taskFile,
                                    "initialState.base." +
                                        ocs2::mobile_manipulator::modelTypeEnumToString(modelInfo.manipulatorModelType),
                                    initialBaseState);
    initialState.head(baseStateDim) = initialBaseState;
  }

  ocs2::vector_t initialArmState = ocs2::vector_t::Zero(armStateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState.arm", initialArmState);
  initialState.tail(armStateDim) = initialArmState;
  return initialState;
}

ocs2::vector_t toDistanceVector(const std::vector<hpp::fcl::DistanceResult>& results) {
  ocs2::vector_t distances(results.size());
  for (size_t i = 0; i < results.size(); ++i) {
    distances[static_cast<int>(i)] = results[i].min_distance;
  }
  return distances;
}

template <typename EigenTypeA, typename EigenTypeB>
ErrorStats compareEigen(const std::string& quantity, const EigenTypeA& lhs, const EigenTypeB& rhs) {
  if (lhs.rows() != rhs.rows() || lhs.cols() != rhs.cols()) {
    throw std::runtime_error("Cannot compare " + quantity + ": dimension mismatch");
  }

  double maxAbs = 0.0;
  double sumAbs = 0.0;
  const auto diff = lhs - rhs;
  for (int col = 0; col < diff.cols(); ++col) {
    for (int row = 0; row < diff.rows(); ++row) {
      const double absError = std::abs(diff(row, col));
      maxAbs = std::max(maxAbs, absError);
      sumAbs += absError;
    }
  }
  const size_t count = static_cast<size_t>(diff.rows() * diff.cols());
  return ErrorStats{quantity, count, maxAbs, count > 0 ? sumAbs / static_cast<double>(count) : 0.0};
}

template <typename Function>
TimedResult timeFunction(const std::string& name, size_t outputs, size_t iterations, size_t warmupIterations, Function&& function) {
  double checksum = 0.0;
  for (size_t i = 0; i < warmupIterations; ++i) {
    checksum += function();
  }

  const auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < iterations; ++i) {
    checksum += function();
  }
  const auto end = std::chrono::steady_clock::now();
  const double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
  return TimedResult{name, outputs, iterations, totalMs, totalMs * 1000.0 / static_cast<double>(iterations), checksum};
}

double quadraticApproximationChecksum(const ocs2::ScalarFunctionQuadraticApproximation& approximation) {
  double checksum = approximation.f;
  checksum += 1e-12 * approximation.dfdx.sum();
  checksum += 1e-18 * approximation.dfdxx.sum();
  checksum += 1e-24 * approximation.dfdu.sum();
  checksum += 1e-30 * approximation.dfdux.sum();
  checksum += 1e-36 * approximation.dfduu.sum();
  return checksum;
}

template <typename LinearApproximationFunction>
void appendQuadraticBreakdownResults(std::vector<TimedResult>& results, const std::string& backendName, size_t outputs,
                                     const ocs2::MultidimensionalPenalty& penalty, size_t iterations, size_t warmupIterations,
                                     LinearApproximationFunction&& linearApproximationFunction) {
  double checksum = 0.0;
  double linearChecksum = 0.0;
  for (size_t i = 0; i < warmupIterations; ++i) {
    const auto linearApproximation = linearApproximationFunction();
    const auto quadraticApproximation = penalty.getQuadraticApproximation(0.0, linearApproximation);
    checksum += quadraticApproximationChecksum(quadraticApproximation);
    linearChecksum += linearApproximation.f.sum() + 1e-12 * linearApproximation.dfdx.sum();
  }

  std::chrono::steady_clock::duration linearDuration = std::chrono::steady_clock::duration::zero();
  std::chrono::steady_clock::duration otherDuration = std::chrono::steady_clock::duration::zero();
  for (size_t i = 0; i < iterations; ++i) {
    const auto linearStart = std::chrono::steady_clock::now();
    const auto linearApproximation = linearApproximationFunction();
    const auto linearEnd = std::chrono::steady_clock::now();
    const auto quadraticApproximation = penalty.getQuadraticApproximation(0.0, linearApproximation);
    const auto otherEnd = std::chrono::steady_clock::now();

    linearDuration += linearEnd - linearStart;
    otherDuration += otherEnd - linearEnd;
    checksum += quadraticApproximationChecksum(quadraticApproximation);
    linearChecksum += linearApproximation.f.sum() + 1e-12 * linearApproximation.dfdx.sum();
  }

  const double linearMs = std::chrono::duration<double, std::milli>(linearDuration).count();
  const double otherMs = std::chrono::duration<double, std::milli>(otherDuration).count();
  const double totalMs = linearMs + otherMs;
  results.push_back(TimedResult{"soft-quadratic-staged-total-" + backendName, outputs, iterations, totalMs,
                                totalMs * 1000.0 / static_cast<double>(iterations), checksum});
  results.push_back(TimedResult{"soft-quadratic-staged-linear-" + backendName, outputs, iterations, linearMs,
                                linearMs * 1000.0 / static_cast<double>(iterations), linearChecksum});
  results.push_back(TimedResult{"soft-quadratic-staged-other-" + backendName, outputs, iterations, otherMs,
                                otherMs * 1000.0 / static_cast<double>(iterations), checksum});
}

TimedResult runNextgenDistanceBenchmark(const ocs2::PinocchioInterface& pinocchioInterface,
                                        const pinocchio::GeometryModel& geometryModel, size_t iterations,
                                        size_t warmupIterations) {
  const ocs2::collision_nextgen::impl::SphereCollisionModel model(pinocchioInterface.getModel(), geometryModel);
  return timeFunction("nextgen-sphere-distances", model.getNumPairs(), iterations, warmupIterations,
                      [&]() { return model.getDistances(pinocchioInterface).sum(); });
}

const TimedResult& findTimedResult(const std::vector<TimedResult>& results, const std::string& name) {
  const auto it = std::find_if(results.begin(), results.end(), [&](const TimedResult& result) { return result.name == name; });
  if (it == results.end()) {
    throw std::runtime_error("Missing timed result: " + name);
  }
  return *it;
}

void printTimedResults(const std::string& title, const std::vector<TimedResult>& results) {
  std::cout << '\n' << title << '\n';
  std::cout << "name\toutputs\titerations\ttotal_ms\tavg_us\tchecksum\n";
  for (const auto& result : results) {
    std::cout << result.name << '\t' << result.outputs << '\t' << result.iterations << '\t' << result.totalMs << '\t'
              << result.averageUs << '\t' << result.checksum << '\n';
  }
}

void printErrorStats(const std::vector<ErrorStats>& stats) {
  std::cout << "\nAccuracy vs ocs2_self_collision/FCL\n";
  std::cout << "quantity\tcount\tmax_abs\tmean_abs\n";
  for (const auto& stat : stats) {
    std::cout << stat.quantity << '\t' << stat.count << '\t' << stat.maxAbs << '\t' << stat.meanAbs << '\n';
  }
}

void printQuadraticBreakdown(const std::vector<TimedResult>& results) {
  std::cout << "\nQuadratic Soft-Constraint Breakdown\n";
  std::cout << "backend\ttotal_avg_us\tlinear_avg_us\tlinear_%\tother_avg_us\tother_%\tindependent_total_avg_us\tindependent_penalty_avg_us\n";

  for (const std::string& backend : {"nextgen", "ocs2-self-collision"}) {
    const auto& total = findTimedResult(results, "soft-quadratic-staged-total-" + backend);
    const auto& linear = findTimedResult(results, "soft-quadratic-staged-linear-" + backend);
    const auto& other = findTimedResult(results, "soft-quadratic-staged-other-" + backend);
    const auto& independentTotal = findTimedResult(results, "soft-quadratic-total-" + backend);
    const auto& penalty = findTimedResult(results, "soft-quadratic-penalty-" + backend);

    const double totalUs = total.averageUs;
    const double linearUs = linear.averageUs;
    const double otherUs = other.averageUs;
    const double penaltyUs = penalty.averageUs;

    const double linearShare = totalUs > 0.0 ? 100.0 * linearUs / totalUs : 0.0;
    const double otherShare = totalUs > 0.0 ? 100.0 * otherUs / totalUs : 0.0;

    std::cout << backend << '\t' << totalUs << '\t' << linearUs << '\t' << linearShare << '\t' << otherUs << '\t' << otherShare
              << '\t' << independentTotal.averageUs << '\t' << penaltyUs << '\n';
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const Settings settings = parseArgs(argc, argv);
    const std::string collisionUrdfFile = resolvePathRelativeToTaskFile(settings.collisionUrdfFile, settings.taskFile);

    const auto modelType =
        ocs2::mobile_manipulator::loadManipulatorType(settings.taskFile, "model_information.manipulatorModelType");
    std::vector<std::string> removeJointNames;
    ocs2::loadData::loadStdVector<std::string>(settings.taskFile, "model_information.removeJoints", removeJointNames, false);

    std::string baseFrame;
    std::string eeFrame;
    ocs2::loadData::loadCppDataType(settings.taskFile, "model_information.baseFrame", baseFrame);
    ocs2::loadData::loadCppDataType(settings.taskFile, "model_information.eeFrame", eeFrame);

    ocs2::scalar_t minimumDistance = 0.0;
    ocs2::scalar_t barrierMu = 1e-2;
    ocs2::scalar_t barrierDelta = 1e-3;
    ocs2::loadData::loadCppDataType(settings.taskFile, "selfCollision.minimumDistance", minimumDistance);
    ocs2::loadData::loadCppDataType(settings.taskFile, "selfCollision.mu", barrierMu);
    ocs2::loadData::loadCppDataType(settings.taskFile, "selfCollision.delta", barrierDelta);

    std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
    ocs2::loadData::loadStdVectorOfPair(settings.taskFile, "selfCollision.collisionLinkPairs", collisionLinkPairs, true);
    std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    ocs2::loadData::loadStdVectorOfPair(settings.taskFile, "selfCollision.collisionObjectPairs", collisionObjectPairs, true);

    ocs2::PinocchioInterface pinocchioInterface =
        ocs2::mobile_manipulator::createPinocchioInterface(collisionUrdfFile, modelType, removeJointNames);
    const ocs2::mobile_manipulator::ManipulatorModelInfo modelInfo =
        ocs2::mobile_manipulator::createManipulatorModelInfo(pinocchioInterface, modelType, baseFrame, eeFrame);

    ocs2::PinocchioGeometryInterface geometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs);
    ocs2::mobile_manipulator::MobileManipulatorPreComputation preComputation(pinocchioInterface, modelInfo);

    const ocs2::vector_t state = loadInitialState(settings.taskFile, modelInfo);
    const ocs2::vector_t input = ocs2::vector_t::Zero(modelInfo.inputDim);
    preComputation.request(ocs2::Request::SoftConstraint + ocs2::Request::Approximation, 0.0, state, input);
    const auto& updatedPinocchioInterface = preComputation.getPinocchioInterface();

    ocs2::mobile_manipulator::MobileManipulatorSelfCollisionConstraint fclConstraint(
        ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping(modelInfo),
        ocs2::PinocchioGeometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs), minimumDistance);
    ocs2::mobile_manipulator::MobileManipulatorNextgenSelfCollisionConstraint nextgenConstraint(
        ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping(modelInfo), pinocchioInterface.getModel(),
        geometryInterface.getGeometryModel(), minimumDistance);

    const ocs2::collision_nextgen::impl::SphereCollisionModel distanceModel(pinocchioInterface.getModel(),
                                                                            geometryInterface.getGeometryModel());
    const ocs2::vector_t nextgenDistances = distanceModel.getDistances(updatedPinocchioInterface);
    const ocs2::vector_t fclDistances = toDistanceVector(geometryInterface.computeDistances(updatedPinocchioInterface));

    const ocs2::vector_t fclValue = fclConstraint.getValue(0.0, state, preComputation);
    const ocs2::vector_t nextgenValue = nextgenConstraint.getValue(0.0, state, preComputation);
    const auto fclLinear = fclConstraint.getLinearApproximation(0.0, state, preComputation);
    const auto nextgenLinear = nextgenConstraint.getLinearApproximation(0.0, state, preComputation);

    std::unique_ptr<ocs2::PenaltyBase> fclPenaltyBase =
        std::make_unique<ocs2::RelaxedBarrierPenalty>(ocs2::RelaxedBarrierPenalty::Config{barrierMu, barrierDelta});
    std::unique_ptr<ocs2::PenaltyBase> nextgenPenaltyBase =
        std::make_unique<ocs2::RelaxedBarrierPenalty>(ocs2::RelaxedBarrierPenalty::Config{barrierMu, barrierDelta});
    ocs2::MultidimensionalPenalty fclPenalty(std::move(fclPenaltyBase));
    ocs2::MultidimensionalPenalty nextgenPenalty(std::move(nextgenPenaltyBase));

    ocs2::StateSoftConstraint fclSoftConstraint(
        std::make_unique<ocs2::mobile_manipulator::MobileManipulatorSelfCollisionConstraint>(
            ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping(modelInfo),
            ocs2::PinocchioGeometryInterface(pinocchioInterface, collisionLinkPairs, collisionObjectPairs), minimumDistance),
        std::make_unique<ocs2::RelaxedBarrierPenalty>(ocs2::RelaxedBarrierPenalty::Config{barrierMu, barrierDelta}));
    ocs2::StateSoftConstraint nextgenSoftConstraint(
        std::make_unique<ocs2::mobile_manipulator::MobileManipulatorNextgenSelfCollisionConstraint>(
            ocs2::mobile_manipulator::MobileManipulatorPinocchioMapping(modelInfo), pinocchioInterface.getModel(),
            geometryInterface.getGeometryModel(), minimumDistance),
        std::make_unique<ocs2::RelaxedBarrierPenalty>(ocs2::RelaxedBarrierPenalty::Config{barrierMu, barrierDelta}));

    const ocs2::TargetTrajectories targetTrajectories;
    const auto fclPenaltyQuadratic = fclPenalty.getQuadraticApproximation(0.0, fclLinear);
    const auto nextgenPenaltyQuadratic = nextgenPenalty.getQuadraticApproximation(0.0, nextgenLinear);
    const auto fclSoftQuadratic = fclSoftConstraint.getQuadraticApproximation(0.0, state, targetTrajectories, preComputation);
    const auto nextgenSoftQuadratic = nextgenSoftConstraint.getQuadraticApproximation(0.0, state, targetTrajectories, preComputation);

    std::vector<ErrorStats> accuracyStats;
    accuracyStats.push_back(compareEigen("collision_distance", nextgenDistances, fclDistances));
    accuracyStats.push_back(compareEigen("constraint_value", nextgenValue, fclValue));
    accuracyStats.push_back(compareEigen("linear_f", nextgenLinear.f, fclLinear.f));
    accuracyStats.push_back(compareEigen("linear_dfdx", nextgenLinear.dfdx, fclLinear.dfdx));
    accuracyStats.push_back(compareScalar("soft_quadratic_cost", nextgenSoftQuadratic.f, fclSoftQuadratic.f));
    accuracyStats.push_back(compareEigen("soft_quadratic_dfdx", nextgenSoftQuadratic.dfdx, fclSoftQuadratic.dfdx));
    accuracyStats.push_back(compareEigen("soft_quadratic_dfdxx", nextgenSoftQuadratic.dfdxx, fclSoftQuadratic.dfdxx));
    accuracyStats.push_back(compareScalar("nextgen_total_vs_penalty_cost", nextgenSoftQuadratic.f, nextgenPenaltyQuadratic.f));
    accuracyStats.push_back(compareEigen("nextgen_total_vs_penalty_dfdx", nextgenSoftQuadratic.dfdx, nextgenPenaltyQuadratic.dfdx));
    accuracyStats.push_back(compareEigen("nextgen_total_vs_penalty_dfdxx", nextgenSoftQuadratic.dfdxx, nextgenPenaltyQuadratic.dfdxx));
    accuracyStats.push_back(compareScalar("fcl_total_vs_penalty_cost", fclSoftQuadratic.f, fclPenaltyQuadratic.f));
    accuracyStats.push_back(compareEigen("fcl_total_vs_penalty_dfdx", fclSoftQuadratic.dfdx, fclPenaltyQuadratic.dfdx));
    accuracyStats.push_back(compareEigen("fcl_total_vs_penalty_dfdxx", fclSoftQuadratic.dfdxx, fclPenaltyQuadratic.dfdxx));

    std::vector<TimedResult> distanceResults;
    distanceResults.reserve(2);
    distanceResults.push_back(runNextgenDistanceBenchmark(updatedPinocchioInterface, geometryInterface.getGeometryModel(),
                                                          settings.iterations, settings.warmupIterations));
    distanceResults.push_back(timeFunction("pinocchio-fcl-computeDistances", geometryInterface.getNumCollisionPairs(), settings.iterations,
                                           settings.warmupIterations, [&]() {
                                             return toDistanceVector(geometryInterface.computeDistances(updatedPinocchioInterface)).sum();
                                           }));

    std::vector<TimedResult> constraintResults;
    constraintResults.push_back(timeFunction("value-nextgen", nextgenConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations,
                                             [&]() { return nextgenConstraint.getValue(0.0, state, preComputation).sum(); }));
    constraintResults.push_back(timeFunction("value-ocs2-self-collision", fclConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations,
                                             [&]() { return fclConstraint.getValue(0.0, state, preComputation).sum(); }));
    constraintResults.push_back(timeFunction("linear-nextgen", nextgenConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations, [&]() {
                                               const auto approximation =
                                                   nextgenConstraint.getLinearApproximation(0.0, state, preComputation);
                                               return approximation.f.sum() + 1e-12 * approximation.dfdx.sum();
                                             }));
    constraintResults.push_back(timeFunction("linear-ocs2-self-collision", fclConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations, [&]() {
                                               const auto approximation = fclConstraint.getLinearApproximation(0.0, state, preComputation);
                                               return approximation.f.sum() + 1e-12 * approximation.dfdx.sum();
                                             }));
    constraintResults.push_back(timeFunction("soft-quadratic-total-nextgen", nextgenConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations, [&]() {
                                               const auto approximation = nextgenSoftConstraint.getQuadraticApproximation(
                                                   0.0, state, targetTrajectories, preComputation);
                                               return quadraticApproximationChecksum(approximation);
                                             }));
    constraintResults.push_back(timeFunction("soft-quadratic-total-ocs2-self-collision", fclConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations, [&]() {
                                               const auto approximation =
                                                   fclSoftConstraint.getQuadraticApproximation(0.0, state, targetTrajectories, preComputation);
                                               return quadraticApproximationChecksum(approximation);
                                             }));
    constraintResults.push_back(timeFunction("soft-quadratic-linear-nextgen", nextgenConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations, [&]() {
                                               const auto approximation =
                                                   nextgenConstraint.getLinearApproximation(0.0, state, preComputation);
                                               return approximation.f.sum() + 1e-12 * approximation.dfdx.sum();
                                             }));
    constraintResults.push_back(timeFunction("soft-quadratic-linear-ocs2-self-collision", fclConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations, [&]() {
                                               const auto approximation = fclConstraint.getLinearApproximation(0.0, state, preComputation);
                                               return approximation.f.sum() + 1e-12 * approximation.dfdx.sum();
                                             }));
    constraintResults.push_back(timeFunction("soft-quadratic-penalty-nextgen", nextgenConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations,
                                             [&]() { return quadraticApproximationChecksum(nextgenPenalty.getQuadraticApproximation(0.0, nextgenLinear)); }));
    constraintResults.push_back(timeFunction("soft-quadratic-penalty-ocs2-self-collision", fclConstraint.getNumConstraints(0.0),
                                             settings.constraintIterations, settings.constraintWarmupIterations,
                                             [&]() { return quadraticApproximationChecksum(fclPenalty.getQuadraticApproximation(0.0, fclLinear)); }));
    appendQuadraticBreakdownResults(constraintResults, "nextgen", nextgenConstraint.getNumConstraints(0.0), nextgenPenalty,
                                    settings.constraintIterations, settings.constraintWarmupIterations,
                                    [&]() { return nextgenConstraint.getLinearApproximation(0.0, state, preComputation); });
    appendQuadraticBreakdownResults(constraintResults, "ocs2-self-collision", fclConstraint.getNumConstraints(0.0), fclPenalty,
                                    settings.constraintIterations, settings.constraintWarmupIterations,
                                    [&]() { return fclConstraint.getLinearApproximation(0.0, state, preComputation); });

    std::cout << "task\t" << settings.taskFile << '\n';
    std::cout << "collision_urdf\t" << collisionUrdfFile << '\n';
    std::cout << "pairs\t" << geometryInterface.getNumCollisionPairs() << '\n';
    std::cout << "state_dim\t" << modelInfo.stateDim << '\n';
    std::cout << "input_dim\t" << modelInfo.inputDim << '\n';
    std::cout << "barrier_mu\t" << barrierMu << '\n';
    std::cout << "barrier_delta\t" << barrierDelta << '\n';
    std::cout << "state\t" << state.transpose() << '\n';
    printTimedResults("Collision Distance Speed", distanceResults);
    printTimedResults("Constraint Speed", constraintResults);
    printQuadraticBreakdown(constraintResults);
    printErrorStats(accuracyStats);

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}
