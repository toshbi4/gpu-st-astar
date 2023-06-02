#include <fstream>
#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include <st_a_star.hpp>
#include <valarray>
#include <algorithm>

struct SolutionExt{

    SolutionExt(std::vector<Location> aSolution, uint16_t aAgentNumber):
        solution {aSolution},
        agentNumber {aAgentNumber}
    {

    }

    std::vector<Location> solution;
    uint16_t agentNumber;

};

bool compareByLength(const SolutionExt &a, const SolutionExt &b)
{
    return a.solution.size() < b.solution.size();
}

StAStar::StAStar(Environment& environment, bool aLifeLong, uint8_t aSeed,
                 std::vector<Location> aGoals, std::vector<Location> aStartPositions):
    m_env(environment),
    TOT{environment.m_dimx, std::vector<std::vector<TimeRange>>(environment.m_dimy)},
    lifelong{aLifeLong},
    rng{aSeed},
    goals{aGoals},
    goalsTemp{aGoals},
    startPositions{aStartPositions},
    startPositionsTemp{aStartPositions}
{
    if (lifelong){
        std::shuffle(std::begin(goals), std::end(goals), rng);
    }
}

StAStar::~StAStar(){
    TOT.clear();
}

std::vector<Location> StAStar::freeAgent(int x, int y,
                                         float time, bool hasCargo,
                                         int dimx, int dimy){

    TOT[x][y].clear();
    for(int i = 0; i < dimx; i++) {
        for(int j = 0; j < dimy; j++) {
            TOT[i][j].erase(std::remove_if(TOT[i][j].begin(), TOT[i][j].end(), [time](TimeRange x) {
                                return (x.end < time); }), TOT[i][j].end());
        }
    }

    if (goalsTemp.empty()){
        goalsTemp = goals;
        std::shuffle(std::begin(goalsTemp), std::end(goalsTemp), rng);
    }
    if (startPositionsTemp.empty()){
        startPositionsTemp = startPositions;
        std::shuffle(std::begin(startPositionsTemp), std::end(startPositionsTemp), rng);
    }

    Location start = Location(x, y);
    Location goal{};
    if (!hasCargo){
        goal = goalsTemp.back();
        goalsTemp.pop_back();
    } else {
        goal = startPositionsTemp.back();
        startPositionsTemp.pop_back();
    }

    std::cout << " startPositionsTemp " << std::endl;
    for (uint i = 0; i < startPositionsTemp.size(); i++) {
       std::cout << startPositionsTemp[i] << " ";
     }
    std::cout << " goalsTemp " << std::endl;
    for (uint i = 0; i < goalsTemp.size(); i++) {
       std::cout << goalsTemp[i] << " ";
     }


    std::cout << "Free agent, hasCargo: " << hasCargo << std::endl;
    std::cout << "Free agent, newGoal: " << goal.x << " " << goal.y << std::endl;

    std::vector<Location> solution = singleAgentSolution(start.x,
                                                         start.y,
                                                         goal.x,
                                                         goal.y,
                                                         dimx, dimy,
                                                         time, 0);
    return solution;
}

bool StAStar::search(std::vector<Location> startStates, int dimx, int dimy){

//    std::vector<SolutionExt> solutionExtVec;

//    for (uint16_t it = 0; it < startStates.size(); ++it) {
//        solutionExtVec.push_back(SolutionExt(singleAgentSolution(startStates[it].x,
//                                                                 startStates[it].y,
//                                                                 goals[it].x,
//                                                                 goals[it].y,
//                                                                 considerTOT), it));
//    }
    //std::sort(solutionExtVec.begin(), solutionExtVec.end(), compareByLength);


    //std::cout << startStates.size() << std::endl;
    for (uint16_t it = 0; it < startStates.size(); ++it) {
//        uint16_t agentId = solutionExtVec[it].agentNumber;
//        std::cout << it << std::endl;
//        std::cout << goals[it].x << " " << goals[it].y << std::endl;
        solutions.push_back(singleAgentSolution(startStates[it].x,
                            startStates[it].y,
                            goals[it].x,
                            goals[it].y,
                            dimx, dimy,
                            0.0f, it));
    }

    return true;
}

std::vector<std::vector<Location>> StAStar::getSolution(){
    return solutions;
}

std::vector<Location> StAStar::singleAgentSolution(uint16_t xi, uint16_t yi,
                                                   uint16_t xf, uint16_t yf,
                                                   uint32_t dimx, uint32_t dimy,
                                                   float cost0, uint16_t agentId,
                                                   bool considerTOT){

#ifdef DEBUG_OUPUT
    std::cout << "start: " << xi << " " << yi << std::endl;
    std::cout << "finish: " << xf << " " << yf << std::endl;
#endif

    float g_current = 0;
    float h_current = 0;
    float f_current = 0;

    uint16_t i_current = 0;
    uint16_t j_current = 0;
    uint16_t k_current = 0;

    uint8_t waiting_time=40;

    uint8_t maximum_number_at_the_same_node = 20;

    if (!considerTOT) {
        maximum_number_at_the_same_node = 1;
    }

    float time_to_move_to_next=40.0f;
    uint16_t start[2] {xi, yi};
    uint16_t goal[2] {xf, yf};

    std::vector<std::vector<std::vector<Node>>> unvisited_list(dimx,
                                                               std::vector<std::vector<Node>>(dimy,
                                                                                              std::vector<Node>(maximum_number_at_the_same_node)));
    std::vector<std::vector<std::vector<Node>>> visited_list(dimx,
                                                             std::vector<std::vector<Node>>(dimy,
                                                                                            std::vector<Node>(maximum_number_at_the_same_node)));

    g_current = cost0;
    h_current = (std::abs(goal[0]-start[0]) + std::abs(goal[1]-start[1])) * time_to_move_to_next;
    f_current = g_current + h_current;

    unvisited_list[start[0]][start[1]][0].g = g_current;
    unvisited_list[start[0]][start[1]][0].f = f_current;

    std::cout << unvisited_list[start[0]][start[1]][0].f << std::endl;

    bool end_of_reseach {false};


    while(!end_of_reseach) {

        // Finding cell with the minimum f-value
        float min_f = 100000.0f;
        for (uint32_t idx1=0; idx1<dimx; idx1++){
            for (uint32_t idx2=0; idx2<dimy; idx2++){
                for (uint32_t idx3=0; idx3<maximum_number_at_the_same_node; idx3++){
                    if (unvisited_list[idx1][idx2][idx3].f < min_f){
                        min_f = unvisited_list[idx1][idx2][idx3].f;
                        i_current = idx1;
                        j_current = idx2;
                        k_current = idx3;
                    }
                }
            }
        }

#ifdef DEBUG_OUPUT
        std::cout << "Currents: " << i_current << " " << j_current << " " << k_current << std::endl;
        std::cout << "min_f: " << min_f << std::endl;
#endif

        // If we don't have cells with minimum f-value
        if (!(min_f < 100000.0f)){
            std::cout << " break " << std::endl;
            break;
        }

        for (int16_t j=j_current-1;j<=j_current+1;j+=2) {

            if ((j >= static_cast<uint16_t>(dimy)) || (j < 0)){
                continue;
            }

#ifdef DEBUG_OUPUT
            std::cout << "Inspect-J-neigh: " << i_current << " " << j << " AgentXI: " << xi << std::endl;
#endif

            // Here we need to check the availabilty of the waiting in the
            // node (max(unvisited_list(i,j,:,6))==1) there still a wait
            // chance

            if (m_env.checkObstacles(i_current,j) && !unvisited_list[i_current][j][0].visited) {

                float new_g = unvisited_list[i_current][j_current][k_current].g + time_to_move_to_next;
                if (unvisited_list[i_current][j][0].g > new_g) {

                    bool collision1 = false;
                    bool collision2 = false;
                    if (considerTOT)
                    {

                        for (auto & elem : TOT[i_current][j]){
#ifdef DEBUG_OUPUT
                            std::cout << i_current << " " << j << k_current << std::endl;
                            std::cout << "TOT Range: " << new_g - time_to_move_to_next / 2.0f << " "
                                      << new_g + waiting_time / 2.0f << std::endl;
                            std::cout << "New Range: " << elem.start << " " << elem.end << std::endl;
#endif
                            if (elem.inRange(new_g - time_to_move_to_next / 2.0f, new_g + waiting_time / 2.0f)){
                                collision1 = true;
                                break;
                            }
                        }

                        // TODO: rewrite end condition
                        for (auto & elem : TOT[i_current][j]){
#ifdef DEBUG_OUPUT
                            std::cout << i_current << " " << j << k_current << std::endl;
                            std::cout << "TOT Range: " << new_g - time_to_move_to_next / 2.0f << " " << 100000.0f << std::endl;
                            std::cout << "New Range: " << elem.start << " " << elem.end << std::endl;
#endif
                            if (elem.inRange(new_g - time_to_move_to_next / 2.0f, 100000.0f)){
                                collision2 = true;
                                break;
                            }
                        }

                        bool endPoint = (i_current==xf) && (j==yf);
                        collision2 = endPoint && collision2;
                    }

                    bool cond = ((!collision1) && (!collision2));

                    if (cond) {

                        unvisited_list[i_current][j][0].g = new_g;
                        unvisited_list[i_current][j][0].f = unvisited_list[i_current][j][0].g +
                                (std::abs(i_current - goal[0]) + std::abs(j-goal[1])) * time_to_move_to_next;
                        unvisited_list[i_current][j][0].i_prev = i_current;
                        unvisited_list[i_current][j][0].j_prev = j_current;
                        unvisited_list[i_current][j][0].k_prev = k_current;

                        std::cout << "Added" << std::endl;
                        std::cout << "G-value " << unvisited_list[i_current][j][0].g << std::endl;
                        std::cout << "H-value " << (std::abs(i_current - goal[0]) + std::abs(j-goal[1])) * time_to_move_to_next << std::endl;
                        std::cout << "F-value " << unvisited_list[i_current][j][0].f << std::endl;

                    }
                }
            }
        }

        for (int16_t i=i_current-1;i<=i_current+1;i+=2) {

            if ((i >= static_cast<int16_t>(dimx)) || (i < 0)){
                continue;
            }

            // Here we need to check the availabilty of the waiting in the
            // node (max(unvisited_list(i,j,:,6))==1) there still a wait
            // chance

#ifdef DEBUG_OUPUT
            std::cout << "Inspect-I-neigh: " << i << " " << j_current << " AgentXI: " << xi << std::endl;
#endif
            if (m_env.checkObstacles(i,j_current) && !unvisited_list[i][j_current][0].visited) {

                float new_g = unvisited_list[i_current][j_current][k_current].g + time_to_move_to_next;

                if (unvisited_list[i][j_current][0].g > new_g) {

                    bool collision1 = false;
                    bool collision2 = false;
                    if (considerTOT)
                    {

                        for (auto & elem : TOT[i][j_current]){
#ifdef DEBUG_OUPUT
                            std::cout << i << " " << j_current << k_current << std::endl;
                            std::cout << "TOT Range: " << elem.start << " " << elem.end << std::endl;
                            std::cout << "New Range: " << new_g - time_to_move_to_next / 2.0f << " "
                                      << new_g + waiting_time / 2.0f << std::endl;
#endif
                            if (elem.inRange(new_g - time_to_move_to_next / 2.0f, new_g + waiting_time / 2.0f)){
                                collision1 = true;
                                break;
                            }
                        }

                        for (auto & elem : TOT[i][j_current]){
#ifdef DEBUG_OUPUT
                            std::cout << i << " " << j_current << std::endl;
                            std::cout << "TOT Range: " << elem.start << " " << elem.end << std::endl;
                            std::cout << "New Range: " << new_g - time_to_move_to_next / 2.0f << " "
                                      << 100000.0f << std::endl;
#endif
                            if (elem.inRange(new_g - time_to_move_to_next / 2.0f, 100000.0f)){
                                collision2 = true;
                                break;
                            }
                        }

                        collision2 = (i==xf) && (j_current==yf) && collision2;
                    }

                    if (!collision1 && !collision2) {

                        unvisited_list[i][j_current][0].g = new_g;
                        unvisited_list[i][j_current][0].f = unvisited_list[i][j_current][0].g +
                                (std::abs(i - goal[0]) + std::abs(j_current-goal[1])) * time_to_move_to_next;
                        unvisited_list[i][j_current][0].i_prev = i_current;
                        unvisited_list[i][j_current][0].j_prev = j_current;
                        unvisited_list[i][j_current][0].k_prev = k_current;
#ifdef DEBUG_OUPUT
                        std::cout << "Added-J-neigh" << std::endl;
                        std::cout << "G-value " << unvisited_list[i][j_current][0].g << std::endl;
                        std::cout << "H-value " << (std::abs(i - goal[0]) + std::abs(j_current-goal[1])) * time_to_move_to_next << std::endl;
                        std::cout << "F-value " << unvisited_list[i][j_current][0].f << std::endl;
#endif
                    }
                }
            }
        }

        if (k_current < maximum_number_at_the_same_node) {

            uint16_t k = k_current + 1;

#ifdef DEBUG_OUPUT
            std::cout << "Inspect-k-neigh: " << i_current << " " << j_current << " " << k << " AgentXI: " << xi << std::endl;
#endif

            if (k >= maximum_number_at_the_same_node){
#ifdef DEBUG_OUPUT
            std::cout << "Can't find solution. Agents will stay for 5 waitings" << std::endl;
#endif
                break;
            }

            float new_g = unvisited_list[i_current][j_current][k_current].g + waiting_time;
            if ((unvisited_list[i_current][j_current][k].g > new_g)
                    && !unvisited_list[i_current][j_current][k].visited) {

                bool collision1 = false;
                for (auto & elem : TOT[i_current][j_current]){
                    std::cout << i_current << " " << j_current << k_current << std::endl;
                    std::cout << "TOT Range: " << elem.start << " " << elem.end << std::endl;
                    std::cout << "New Range: " << new_g << " "
                              << new_g + waiting_time << std::endl;
                    if (elem.inRange(new_g, new_g + waiting_time)){
                        collision1 = true;
                        break;
                    }
                }

                if (!collision1) {

                    unvisited_list[i_current][j_current][k].g = new_g;
                    unvisited_list[i_current][j_current][k].f = unvisited_list[i_current][j_current][k].g +
                            (std::abs(i_current - goal[0]) + std::abs(j_current-goal[1])) * time_to_move_to_next;
                    unvisited_list[i_current][j_current][k].i_prev = i_current;
                    unvisited_list[i_current][j_current][k].j_prev = j_current;
                    unvisited_list[i_current][j_current][k].k_prev = k_current;
#ifdef DEBUG_OUPUT
                    std::cout << "Added-K-neigh" << std::endl;
                    std::cout << "F-value" << unvisited_list[i_current][j_current][k].f << std::endl;
#endif
                }
            }
        }

        visited_list[i_current][j_current][k_current] = unvisited_list[i_current][j_current][k_current];
        unvisited_list[i_current][j_current][k_current].visited = true;
        unvisited_list[i_current][j_current][k_current].f = 100000.0f;

        if ((i_current==goal[0]) && (j_current==goal[1]))
        {
            end_of_reseach=true;
        }
    }

    if (end_of_reseach) {

#ifdef DEBUG_OUPUT
        std::cout << "End of research" << std::endl;
#endif

    }
    else {

#ifdef DEBUG_OUPUT
        std::cout << "There is no valid solution for one of the agents" << std::endl;
#endif
        unvisited_list.clear();
        visited_list.clear();

        std::vector<Location> solution;
        std::vector<float> g;
        for (uint8_t i=1; i<=5 ;++i){
            solution.push_back(Location(start[0], start[1]));
        }
        float initialCost = visited_list[start[0]][start[1]][0].g;
        TOT[start[0]][start[1]].push_back(TimeRange(initialCost, initialCost + 5*waiting_time, agentId));
        // Return only waiting solution solution
        return solution;
    }

    std::vector<Location> solution;
    std::vector<float> g;
    solution.push_back(Location(goal[0], goal[1]));
    g.push_back(visited_list[xf][yf][k_current].g);

    //std::cout << "G: " << visited_list[xf][yf][k_current].g << std::endl;

    uint16_t k=k_current;
    uint16_t i=1;

    while (!((solution[i-1].x==start[0]) &&
            (solution[i-1].y==start[1]) &&
             (k==0)))
    {
        solution.push_back(Location(visited_list[solution[i-1].x][solution[i-1].y][k].i_prev,
                                    visited_list[solution[i-1].x][solution[i-1].y][k].j_prev));
        k = visited_list[solution[i-1].x][solution[i-1].y][k].k_prev;
        g.push_back(visited_list[solution[i].x][solution[i].y][k].g);

        //std::cout << "G: " << visited_list[solution[i].x][solution[i].y][k].g << std::endl;

        i=i+1;
    }

    std::reverse(solution.begin(), solution.end());
    std::reverse(g.begin(), g.end());

    for (uint16_t i=1; i < g.size(); i++) {
        if ((solution[i].x == solution[i-1].x) && (solution[i].y == solution[i-1].y)) {
            std::cout << "Add to TOT: (x,y) " << solution[i].x << " " << solution[i].y << " range " <<
                     g[i-1] << " " << g[i] << std::endl;
            TOT[solution[i].x][solution[i].y].push_back(TimeRange(g[i-1], g[i], agentId));
        }
        else {
            float mid = g[i] - time_to_move_to_next / 2.0f;
            std::cout << "Add to TOT: (x,y) " << solution[i].x << " " << solution[i].y << " range " <<
                     mid << " " << g[i] << std::endl;
            TOT[solution[i].x][solution[i].y].push_back(TimeRange(mid, g[i], agentId));
            std::cout << "Add to TOT: (x,y) " << solution[i-1].x << " " << solution[i-1].y << " range " <<
                     g[i-1] << " " << mid << std::endl;
            TOT[solution[i-1].x][solution[i-1].y].push_back(TimeRange(g[i-1], mid, agentId));
        }
    }

    float cost = visited_list[xf][yf][k_current].g;
    std::cout << "Final cost: " << cost + waiting_time << std::endl;
    std::cout << "Add to TOT: (x,y) " << xf << " " << yf << " range " <<
             cost << " " << cost + waiting_time << std::endl;
    TOT[xf][yf].push_back(TimeRange(cost, cost + 2*waiting_time, agentId)); // cost + 120.0f

    unvisited_list.clear();
    visited_list.clear();

    return solution;
}
