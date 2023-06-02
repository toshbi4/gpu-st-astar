#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include <st_a_star.hpp>
#include <valarray>
#include <algorithm>

int main(int argc, char* argv[]) {

    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    bool lifelong;
    uint8_t randSeed = 1;
    desc.add_options()
          ("help", "produce help message")
          ("input,i", po::value<std::string>(&inputFile)->required(),
           "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)")
          ("lifelong", po::bool_switch(&lifelong), "TurnOn LifeLong mode.")
          ("seed", po::value<uint8_t>(&randSeed), "The seed for recreating radom variables.");

    try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
    } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
    }

    YAML::Node config = YAML::LoadFile(inputFile);

    std::unordered_set<Location> obstacles;
    std::vector<Location> goals;
    std::vector<Location> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(Location(start[0].as<int>(), start[1].as<int>()));
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  // sanity check: no identical start states
  std::unordered_set<Location> startStatesSet;
  for (const auto& s : startStates) {
    if (startStatesSet.find(s) != startStatesSet.end()) {
      std::cout << "Identical start states detected -> no solution!" << std::endl;
      return 0;
    }
    startStatesSet.insert(s);
  }

  Environment env(dimx, dimy, obstacles);
  StAStar stAStar{env, lifelong, randSeed, goals, startStates};

  Timer timer;
  bool success = stAStar.search(startStates, dimx, dimy);
  timer.stop();
  std::cout << "  runtime: " << timer.elapsedSeconds() << std::endl;

  if (success) {

    std::vector<std::vector<Location>> solution = stAStar.getSolution();

    std::cout << "Planning successful! " << std::endl;
    //int cost = 0;
    int makespan = 0;
//    for (const auto& s : solution) {
//      cost += s.cost;
//      makespan = std::max<int>(makespan, s.cost);
//    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << 0 << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << 0 << std::endl;
    out << "  lowLevelExpanded: " << 0 << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      out << "  agent" << a << ":" << std::endl;
      for (size_t t = 0; t < solution[a].size(); ++t) {
        out << "    - x: " << solution[a][t].x << std::endl
            << "      y: " << solution[a][t].y << std::endl
            << "      t: " << t << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
