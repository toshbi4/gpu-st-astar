#include "astar.h"

#include "PriorityQueue.h"
#include <map>

namespace {
struct NodeCost {
    NodeCost(Node _node, float _totalCost, float _heuristic, Node _predecessor)
        : node(std::move(_node)), totalCost(_totalCost), heuristic(_heuristic),
          predecessor(std::move(_predecessor)) {}

    Node  node;
    float totalCost;   // g-value
    float heuristic;   // h-value
    Node  predecessor; // to recreate path
};

// Comparator to put cheapest nodes first.
struct Compare {
    bool operator()(const NodeCost &a, const NodeCost &b) {
        return a.totalCost + a.heuristic > b.totalCost + b.heuristic;
    }
};
} // namespace

struct TimeRange{
    TimeRange(float aStart, float aEnd, uint16_t aAgentId=0):
        start {aStart},
        end {aEnd},
        agentId {aAgentId}
    {
    }

    bool inRange(float aStart, float aEnd){
        return ((aStart >= start) && (aStart <= end)) ||
               ((aEnd >= start) && (aEnd <= end)) ||
               ((aEnd >= end) && (aStart <= start));
    }

    float start;
    float end;
    uint16_t agentId;
};

// Implementation like in https://de.wikipedia.org/wiki/A*-Algorithmus#Funktionsweise
std::vector<std::vector<Node>>  cpuAStar(const Graph &graph, std::vector<Position> sourceVec,
                                         std::vector<Position> destVec) {
    
    std::vector<std::vector<std::vector<TimeRange>>> TOT{graph.width(), std::vector<std::vector<TimeRange>>(graph.height())};
    std::vector<std::vector<Node>> paths;

    for (const Position source : sourceVec) {

    const Position destination = destVec.back();
    destVec.pop_back();

    if (source == destination) {
        paths.push_back(std::vector<Node>{Node(graph, source)});
        continue;
    }

    // Open and closed list
    PriorityQueue<NodeCost, Compare> open;
    // (Tree) map seems to perform _much_ better than the unordered hash map!
    std::map<Node, Node> closed; // store predecessor as value to recreate path

    // Begin at source
    const Node sourceNode(graph, source);
    open.emplace(sourceNode, 0.0f, 0.0f, sourceNode);

    while (!open.empty()) {
        auto current = open.top();
        open.pop();

        current.node.setCost(current.totalCost);
        // std::cout << current.node.position().x << " " << current.node.position().y << " " << current.node.position().t << std::endl;
        // Reached destination! Restore path and return.
        if (current.node.position() == destination) {
            std::vector<Node> result = {current.node, current.predecessor};

            for (auto it = closed.find(result.back()); it->second != result.back();
                 it = closed.find(result.back())) {
                result.push_back(it->second);
            }

            std::reverse(result.begin(), result.end());
            
            for (auto node : result) {
                
                TOT[node.position().x][node.position().y].push_back(TimeRange(node.getCost(), node.getCost() + 1));
                // std::cout << node.position().x << " " << node.position().y << "(cost): " << node.getCost() << std::endl;
            }

            paths.push_back(result);
            break;
        }

        closed.emplace(current.node, current.predecessor);

        // Expand node
        for (const auto &neighbor : current.node.neighbors()) {
            Node nbNode = neighbor.first;
            auto &nbStepCost = neighbor.second;

            // Already visited (cycle)
            if (closed.count(nbNode) != 0)
                continue;

            auto nbTotalCost = current.totalCost + nbStepCost;
            // Check TOT
            for (auto & elem : TOT[nbNode.position().x][nbNode.position().y]){
                if (elem.inRange(nbTotalCost, nbTotalCost + 1)){
                    Node newNode(graph, Position{current.node.position().x, current.node.position().y, current.node.position().t + 1}); 
                    nbNode = newNode;
                    nbTotalCost = current.totalCost + 1;
                }
            }

            const auto nbIndex =
                open.find_if([&](const NodeCost &nc) { return nc.node == nbNode; });

            // Node already queued for visiting and other path cost is equal or better
            if (nbIndex < open.size() && open[nbIndex].totalCost <= nbTotalCost)
                continue;

            const auto nbHeuristic = (destination - nbNode.position()).length();

            if (nbIndex < open.size())
                open.update(nbIndex, {nbNode, nbTotalCost, nbHeuristic, current.node});
            else
                open.emplace(nbNode, nbTotalCost, nbHeuristic, current.node);
        }
    }

    }
    // No path found
    return paths;
}
