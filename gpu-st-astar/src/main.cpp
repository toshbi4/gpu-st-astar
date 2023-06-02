#include "Graph.h"
#include "astar.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <random>
#include <vector>
#include <boost/program_options.hpp>

namespace compute = boost::compute;

// Little helper for validation
static float costs(const std::vector<Node> &path) {
    if (path.empty())
        return 0.0f;

    auto &graph = path.front().graph();
    auto  node = path.begin();
    float costs = 0.0f;

    for (auto pred = node++; node != path.end(); pred = node++)
        costs += graph.pathCost(*pred, *node);

    return costs;
}

// Run parallel GA*
static void runGAStar(const compute::device &clDevice, std::string inputFile, std::string outputFile) {
    // Generate graph and obstacles
    Graph graph(inputFile); // should be big

    std::cout << "Graph" << std::endl;

    std::vector<Position> sourceVec = graph.sourceVec;
    std::vector<Position> destVec = graph.destVec;

    std::cout << graph.sourceVec.back().x << std::endl;

    // CPU reference run
    std::cout << " ----- CPU reference run..." << std::endl;
    const auto cpuStart = std::chrono::high_resolution_clock::now();
    const auto cpuPaths = cpuAStar(graph, sourceVec, destVec);
    const auto cpuStop = std::chrono::high_resolution_clock::now();

    // Print cpu timing
    std::cout << "CPU time for graph (" << graph.width() << ", " << graph.height()
              << "): " << std::chrono::duration<double>(cpuStop - cpuStart).count() << " seconds"
              << std::endl;

    try {
        // GPU GA* run
        std::cout << " ----- GPU GA* run..." << std::endl;

        const auto gpuStart = std::chrono::high_resolution_clock::now();
        const std::vector<std::vector<Node>> gpuPaths = gpuGAStar(graph, sourceVec, destVec, clDevice);
        const auto gpuStop = std::chrono::high_resolution_clock::now();

        // Print gpu timing
        std::cout << "GPU time for graph (" << graph.width() << ", " << graph.height()
              << "): " << std::chrono::duration<double>(gpuStop - gpuStart).count() << " seconds"
              << std::endl;

        std::cout << "CPU path:" << std::endl;
        for (std::vector<Node> path : cpuPaths) {
            for (Node node : path) {
                std::cout << node.position().x << " " << node.position().y << ", ";
            }
        }
        std::cout << std::endl;

        std::ofstream outCPU("output/CPU" + outputFile);
        outCPU << "statistics:" << std::endl;
        outCPU << "  cost: " << 0 << std::endl;
        outCPU << "  makespan: " << 0 << std::endl;
        outCPU << "  runtime: " << std::chrono::duration<double>(cpuStop - cpuStart).count() << std::endl;
        outCPU << "  highLevelExpanded: " << 0 << std::endl;
        outCPU << "  lowLevelExpanded: " << 0 << std::endl;
        outCPU << "schedule:" << std::endl;
        for (size_t a = 0; a < cpuPaths.size(); ++a) {
          outCPU << "  agent" << a << ":" << std::endl;
          for (size_t t = 0; t < cpuPaths[a].size(); ++t) {
            outCPU << "    - x: " << cpuPaths[a][t].position().x << std::endl
                   << "      y: " << cpuPaths[a][t].position().y << std::endl
                   << "      t: " << t << std::endl;
          }
        }

        std::cout << "GPU path:" << std::endl;
        for (std::vector<Node> path : gpuPaths) {
            for (Node node : path) {
                std::cout << node.position().x << " " << node.position().y << ", ";
            }
        }
        std::cout << std::endl;

        std::ofstream out("output/GPU" + outputFile);
        out << "statistics:" << std::endl;
        out << "  cost: " << 0 << std::endl;
        out << "  makespan: " << 0 << std::endl;
        out << "  runtime: " << std::chrono::duration<double>(gpuStop - gpuStart).count() << std::endl;
        out << "  highLevelExpanded: " << 0 << std::endl;
        out << "  lowLevelExpanded: " << 0 << std::endl;
        out << "schedule:" << std::endl;
        for (size_t a = 0; a < gpuPaths.size(); ++a) {
          out << "  agent" << a << ":" << std::endl;
          for (size_t t = 0; t < gpuPaths[a].size(); ++t) {
            out << "    - x: " << gpuPaths[a][t].position().x << std::endl
                << "      y: " << gpuPaths[a][t].position().y << std::endl
                << "      t: " << t << std::endl;
          }
        }

    } catch (std::exception &e) {
        std::cerr << "GA* execution failed:\n" << e.what() << std::endl;
    }

}

int main(int argc, char* argv[]) {

// Parse provided parameters
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    desc.add_options()
          ("help", "produce help message")
          ("input,i", po::value<std::string>(&inputFile)->required(),
           "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)");

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


// Choose OpenCL device
#if 1
    // Select default OpenCL device
    compute::device dev = compute::system::default_device();
#else
    // Workaround for testing on broken AMD system: Use CPU instead.
	compute::device dev = []() {
        for (auto d : compute::system::devices())
            if (d.type() == compute::device::cpu)
                return d;
        return compute::system::default_device();
    }();
#endif
	std::cout << "OpenCL device: " << dev.name() << std::endl;

    // Run multi-agent A*
    // runAStar(dev);

    // Run parallel GA*
    runGAStar(dev, inputFile, outputFile);

#ifdef _WIN32
    std::cout << "\nPress ENTER to continue..." << std::flush;
    std::cin.ignore();
#endif
    return 0;
}
