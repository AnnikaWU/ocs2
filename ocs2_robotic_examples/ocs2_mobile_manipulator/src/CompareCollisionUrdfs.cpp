#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

namespace {

struct Settings {
  std::string meshUrdf;
  std::string sphereUrdf;
  std::string pairsFile;
  size_t samples = 20;
  unsigned seed = 0;
  double tolerance = 0.05;
};

void printUsage() {
  std::cerr << "Usage:\n"
            << "  ocs2_compare_collision_urdfs --mesh-urdf <mesh.urdf> --sphere-urdf <sphere.urdf> --pairs <pairs.info> [options]\n\n"
            << "Options:\n"
            << "  --samples <n>     Number of random configurations in addition to zero. Default: 20\n"
            << "  --seed <n>        Random seed. Default: 0\n"
            << "  --tolerance <m>   Maximum allowed absolute distance error in meters. Default: 0.05\n";
}

Settings parseArgs(int argc, char* argv[]) {
  Settings settings;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto requireValue = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for " + name);
      }
      return argv[++i];
    };

    if (arg == "--mesh-urdf") {
      settings.meshUrdf = requireValue(arg);
    } else if (arg == "--sphere-urdf") {
      settings.sphereUrdf = requireValue(arg);
    } else if (arg == "--pairs") {
      settings.pairsFile = requireValue(arg);
    } else if (arg == "--samples") {
      settings.samples = static_cast<size_t>(std::stoul(requireValue(arg)));
    } else if (arg == "--seed") {
      settings.seed = static_cast<unsigned>(std::stoul(requireValue(arg)));
    } else if (arg == "--tolerance") {
      settings.tolerance = std::stod(requireValue(arg));
    } else if (arg == "--help" || arg == "-h") {
      printUsage();
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (settings.meshUrdf.empty() || settings.sphereUrdf.empty() || settings.pairsFile.empty()) {
    printUsage();
    throw std::runtime_error("--mesh-urdf, --sphere-urdf, and --pairs are required");
  }
  return settings;
}

std::vector<std::pair<std::string, std::string>> loadPairs(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Could not open pairs file: " + path);
  }
  const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const std::regex pairRegex(R"PAIR(\[[^\]]+\]\s+"([^",]+)\s*,\s*([^"]+)")PAIR");
  std::vector<std::pair<std::string, std::string>> pairs;
  for (std::sregex_iterator it(text.begin(), text.end(), pairRegex), end; it != end; ++it) {
    pairs.emplace_back((*it)[1].str(), (*it)[2].str());
  }
  if (pairs.empty()) {
    throw std::runtime_error("No collisionLinkPairs found in: " + path);
  }
  return pairs;
}

bool hasUriScheme(const std::string& path) {
  return path.find("://") != std::string::npos;
}

std::string loadUrdfWithAbsoluteMeshPaths(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Could not open URDF: " + path);
  }
  const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  const std::filesystem::path baseDir = std::filesystem::absolute(std::filesystem::path(path)).parent_path();
  const std::regex filenameRegex(R"FILE(filename\s*=\s*"([^"]+)")FILE");

  std::string output;
  std::sregex_iterator begin(text.begin(), text.end(), filenameRegex), end;
  size_t cursor = 0;
  for (auto it = begin; it != end; ++it) {
    const auto& match = *it;
    output.append(text, cursor, static_cast<size_t>(match.position()) - cursor);
    const std::string filename = match[1].str();
    if (filename.empty() || filename.front() == '/' || hasUriScheme(filename)) {
      output += match.str();
    } else {
      const std::filesystem::path absolutePath = (baseDir / filename).lexically_normal();
      output += "filename=\"" + absolutePath.string() + "\"";
    }
    cursor = static_cast<size_t>(match.position() + match.length());
  }
  output.append(text, cursor, std::string::npos);
  return output;
}

ocs2::vector_t randomConfiguration(const pinocchio::Model& model, std::mt19937& rng) {
  ocs2::vector_t q(model.nq);
  for (pinocchio::Model::ConfigVectorType::Index i = 0; i < model.nq; ++i) {
    const double lower = model.lowerPositionLimit[i];
    const double upper = model.upperPositionLimit[i];
    if (std::isfinite(lower) && std::isfinite(upper) && lower < upper) {
      std::uniform_real_distribution<double> distribution(lower, upper);
      q[i] = distribution(rng);
    } else {
      std::uniform_real_distribution<double> distribution(-1.0, 1.0);
      q[i] = distribution(rng);
    }
  }
  return q;
}

double minDistance(const ocs2::PinocchioGeometryInterface& geometryInterface, const ocs2::PinocchioInterface& pinocchioInterface) {
  const auto results = geometryInterface.computeDistances(pinocchioInterface);
  if (results.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double distance = std::numeric_limits<double>::infinity();
  for (const auto& result : results) {
    distance = std::min(distance, result.min_distance);
  }
  return distance;
}

void updateKinematics(ocs2::PinocchioInterface& interface, const ocs2::vector_t& q) {
  pinocchio::forwardKinematics(interface.getModel(), interface.getData(), q);
}

}  // namespace

int main(int argc, char* argv[]) {
  try {
    const Settings settings = parseArgs(argc, argv);
    const auto pairs = loadPairs(settings.pairsFile);

    auto meshInterface = ocs2::getPinocchioInterfaceFromUrdfString(loadUrdfWithAbsoluteMeshPaths(settings.meshUrdf));
    auto sphereInterface = ocs2::getPinocchioInterfaceFromUrdfString(loadUrdfWithAbsoluteMeshPaths(settings.sphereUrdf));
    const auto& meshModel = meshInterface.getModel();
    const auto& sphereModel = sphereInterface.getModel();
    if (meshModel.nq != sphereModel.nq) {
      throw std::runtime_error("URDF models have different nq values");
    }
    if (meshModel.names != sphereModel.names) {
      throw std::runtime_error("URDF models have different joint name sequences");
    }

    std::vector<ocs2::PinocchioGeometryInterface> meshGeometry;
    std::vector<ocs2::PinocchioGeometryInterface> sphereGeometry;
    meshGeometry.reserve(pairs.size());
    sphereGeometry.reserve(pairs.size());
    for (const auto& pair : pairs) {
      const std::vector<std::pair<std::string, std::string>> onePair{pair};
      meshGeometry.emplace_back(meshInterface, onePair);
      sphereGeometry.emplace_back(sphereInterface, onePair);
      if (meshGeometry.back().getNumCollisionPairs() == 0 || sphereGeometry.back().getNumCollisionPairs() == 0) {
        throw std::runtime_error("Pair [" + pair.first + ", " + pair.second + "] has no collision geometry in one URDF");
      }
    }

    std::mt19937 rng(settings.seed);
    double maxAbsError = 0.0;
    double sumAbsError = 0.0;
    size_t comparisons = 0;
    size_t worstSample = 0;
    size_t worstPair = 0;

    for (size_t sample = 0; sample <= settings.samples; ++sample) {
      const ocs2::vector_t q = (sample == 0) ? ocs2::vector_t::Zero(meshModel.nq) : randomConfiguration(meshModel, rng);
      updateKinematics(meshInterface, q);
      updateKinematics(sphereInterface, q);

      for (size_t pairIndex = 0; pairIndex < pairs.size(); ++pairIndex) {
        const double meshDistance = minDistance(meshGeometry[pairIndex], meshInterface);
        const double sphereDistance = minDistance(sphereGeometry[pairIndex], sphereInterface);
        const double absError = std::abs(meshDistance - sphereDistance);
        sumAbsError += absError;
        ++comparisons;
        if (absError > maxAbsError) {
          maxAbsError = absError;
          worstSample = sample;
          worstPair = pairIndex;
        }
      }
    }

    const double meanAbsError = comparisons > 0 ? sumAbsError / static_cast<double>(comparisons) : 0.0;
    std::cout << "Compared collision URDFs through OCS2 PinocchioGeometryInterface\n";
    std::cout << "Pairs: " << pairs.size() << '\n';
    std::cout << "Samples including zero: " << (settings.samples + 1) << '\n';
    std::cout << "Comparisons: " << comparisons << '\n';
    std::cout << "Mean abs distance error: " << meanAbsError << '\n';
    std::cout << "Max abs distance error: " << maxAbsError << '\n';
    std::cout << "Worst pair: [" << pairs[worstPair].first << ", " << pairs[worstPair].second << "] at sample " << worstSample << '\n';

    if (maxAbsError > settings.tolerance) {
      std::cerr << "error: max abs distance error exceeds tolerance " << settings.tolerance << '\n';
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << '\n';
    return 1;
  }
}
