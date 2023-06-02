#include "Graph.h"

#include "Node.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <random>
#include <iostream>
#include <sstream>

Graph::Graph(int width, int height, int time) : m_width(width), m_height(height), m_time(time) {
    // Set default cost for each node to 1.0f
    m_costs.resize(width * height * time, 1.0f);
}

void Graph::generateObstacles(std::string inputFile, int amount) {

    std::ifstream file(inputFile); // открыть файл для чтения
    std::string line;
    int           stage = 0;
    while (std::getline(file, line)) { // читать файл построчно

        if (line.find("agents:") == 0) {
            // Получаем количество агентов
            int agent_count = std::stoi(line.substr(7));
            stage = 1;

        } else if (line.find("map:") == 0) {
            // Получаем размер карты
            std::sscanf(line.c_str(), "map: %d %d", &m_width, &m_height);
            //std::cout << "m_width :" << m_width << " m_height :" << m_height << std::endl;
            m_costs.resize(m_width * m_height * m_time, 1.0f);
            stage = 2;
            
        } else {
            if (stage == 1) {
                // Считываем координаты старта и финиша каждого агента
                int start_x, start_y, goal_x, goal_y;
                std::istringstream iss(line);
                iss >> start_x >> start_y >> goal_x >> goal_y;
                //std::sscanf(line.c_str(), "%d %d %d %d", &start_x, &start_y, &goal_x, &goal_y);
                sourceVec.push_back(Position{start_x, start_y, 0});
                destVec.push_back(Position{goal_x, goal_y, 0});
                // std::cout << "start_x :" << start_x << " start_y :" << start_y << std::endl;
                
            }
            else if (stage == 2) {
                // Считываем координаты препятствий
                int x, y;
                std::istringstream iss(line); // создать поток для чтения строки
                iss >> x >> y; // извлечь координаты из строки
                for (int t = 0; t < m_time; ++t) {
                    m_costs[y * m_width + x + t * m_width * m_height] += 2000;
                    /*std::cout << "Costs: (" << x << ", " << y << ", " << t
                                 << "): " << m_costs[y * m_width + x + t * m_width * m_height]
                                 << std::endl; */
                }
            }
        }
    }
    file.close(); // закрыть файл

}

float Graph::pathCost(const Node &source, const Node &destination) const {
    assert(source.inBounds());
    assert(destination.inBounds());

    const auto &src = source.position();
    const auto &dst = destination.position();

// This function is only legal for neighbors!
#ifndef GRAPH_DIAGONAL_MOVEMENT
    // Orthogonal only

//    std::cout << "src.x: " << src.x << std::endl;
//    std::cout << "src.y: " << src.y << std::endl;
//    std::cout << "dst.x: " << dst.x << std::endl;
//    std::cout << "dst.y: " << dst.y << std::endl;

    assert((std::abs(src.x - dst.x) == 1 && src.y == dst.y) ||
           (std::abs(src.y - dst.y) == 1 && src.x == dst.x));

    return std::max(m_costs[src.y * m_width + src.x + src.t * m_width * m_height],
                    m_costs[dst.y * m_width + dst.x + dst.t * m_width * m_height]);
#else
    // Diagonal connections as well
    assert(std::abs(src.x - dst.x) <= 1 && std::abs(src.y - dst.y) <= 1);

    const bool diagonal = src.x != dst.x && src.y != dst.y;
    const auto      cost = std::max(m_costs[src.y * m_width + src.x + src.t * m_width * m_height],
                                    m_costs[dst.y * m_width + dst.x + dst.t * m_width * m_height]);
    constexpr float sqrt2 = 1.41421356237f;
    return diagonal ? sqrt2 * cost : cost;
#endif
}
