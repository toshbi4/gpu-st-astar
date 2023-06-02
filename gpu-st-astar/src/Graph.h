#pragma once

// #define GRAPH_DIAGONAL_MOVEMENT
#undef GRAPH_DIAGONAL_MOVEMENT

#include <string>
#include <vector>
#include "Position.h"

class Node;

class Graph {
public:
    Graph(int width, int height, int time);
    Graph(std::string inputFile) : m_time{10} { 
        generateObstacles(inputFile);
    }

    void generateObstacles(std::string inputFile, int amount = 10);

    float pathCost(const Node &source, const Node &destination) const;

    int width() const { return m_width; }
    int height() const { return m_height; }
    int time() const { return m_time; }
    int size() const { return m_width * m_height * m_time; }
    std::vector<Position> sourceVec;
    std::vector<Position> destVec;

private:
    int                m_width;
    int                m_height;
    int                m_time;
    std::vector<float> m_costs;
};
