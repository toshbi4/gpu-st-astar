#include "Node.h"
#include <iostream>

Node::Node(const Graph &graph, Position position)
    : m_graph(&graph), m_position(std::move(position)) {}

Node::Node(const Graph &graph, int x, int y, int t) : m_graph(&graph), m_position{x, y, t}, cost{0} {}

std::vector<std::pair<Node, float>> Node::neighbors() const {
    std::vector<std::pair<Node, float>> neighbors;

#ifndef GRAPH_DIAGONAL_MOVEMENT
    neighbors.reserve(4);

    // clockwise
    if (m_position.y > 0) {
        // top neighbor
        //std::cout << "top neighbor" << std::endl;
        const Node neighbor(*m_graph, m_position.x, m_position.y - 1, 0);
        const auto cost = m_graph->pathCost(*this, neighbor);
        neighbors.emplace_back(neighbor, cost);
    }
    if (m_position.x < m_graph->width() - 1) {
        // right neighbor
        //std::cout << "right neighbor" << std::endl;
        const Node neighbor(*m_graph, m_position.x + 1, m_position.y, 0);
        const auto cost = m_graph->pathCost(*this, neighbor);
        neighbors.emplace_back(neighbor, cost);
    }
    if (m_position.y < m_graph->height() - 1) {
        // bottom neighbor
        //std::cout << "bottom neighbor" << std::endl;
        const Node neighbor(*m_graph, m_position.x, m_position.y + 1, 0);
        const auto cost = m_graph->pathCost(*this, neighbor);
        neighbors.emplace_back(neighbor, cost);
    }
    if (m_position.x > 0) {
        // left neighbor
        //std::cout << "left neighbor" << std::endl;
        const Node neighbor(*m_graph, m_position.x - 1, m_position.y, 0);
        const auto cost = m_graph->pathCost(*this, neighbor);
        neighbors.emplace_back(neighbor, cost);
    }
#else
    neighbors.reserve(8);

    for (int y = m_position.y - 1; y <= m_position.y + 1; ++y) {
        for (int x = m_position.x - 1; x <= m_position.x + 1; ++x) {
            const Node neighbor(*m_graph, x, y, 0);
            if (neighbor != *this && neighbor.inBounds())
                neighbors.emplace_back(neighbor, m_graph->pathCost(*this, neighbor));
        }
    }
#endif

    return neighbors;
}
