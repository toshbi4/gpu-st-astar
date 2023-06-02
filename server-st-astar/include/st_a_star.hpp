#pragma once

#include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include <unordered_map>
#include <unordered_set>

#include <vector>
#include "timer.hpp"
#include <random>

#define DEBUG_OUPUT

struct Node {
    Node():
    visited {false},
    g {100000.0f},
    f {100000.0f},
    i_prev {0},
    j_prev {0},
    k_prev {0}
    {

    }

    bool visited;
    float g;
    float f;
    uint16_t i_prev;
    uint16_t j_prev;
    uint16_t k_prev;
};

struct Location {
  Location(){}
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct Solution {
    Solution (Location aPos, uint16_t aTime) :
        pos (aPos),
        time (aTime){

    }

    Location pos;
    uint16_t time;
};

class Environment {
public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles):
    m_dimx(dimx),
    m_dimy(dimy),
    m_obstacles(std::move(obstacles)) {
}

    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    bool checkObstacles(int x, int y) {
    return x >= 0 && x < m_dimx && y >= 0 && y < m_dimy &&
           m_obstacles.find(Location(x, y)) == m_obstacles.end();
    }

public:
    int m_dimx;
    int m_dimy;
private:
    std::unordered_set<Location> m_obstacles;
};

class StAStar
{

struct TimeRange{
    TimeRange(float aStart, float aEnd, uint16_t aAgentId):
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

public:
    StAStar(Environment& environment, bool aLifeLong, uint8_t aSeed,
            std::vector<Location> aGoals, std::vector<Location> aStartPositions);
    ~StAStar();


private:
    Environment& m_env;
    std::vector<std::vector<std::vector<TimeRange>>> TOT;

public:
    bool search(std::vector<Location> startStatesSet, int dimx, int dimy);
    std::vector<std::vector<Location>> getSolution();
    std::vector<Location> freeAgent(int x, int y,
                                    float time, bool hasCargo,
                                    int dimx, int dimy);

private:
    std::vector<Location> singleAgentSolution(uint16_t xi, uint16_t yi,
                                              uint16_t xf, uint16_t yf,
                                              uint32_t dimx, uint32_t dimy,
                                              float cost0, uint16_t agentId,
                                              bool considerTOT=true);

    std::vector<std::vector<Location>> solutions;
    bool lifelong;
    std::default_random_engine rng;
    std::vector<Location> goals;
    std::vector<Location> goalsTemp;
    std::vector<Location> startPositions;
    std::vector<Location> startPositionsTemp;
};
