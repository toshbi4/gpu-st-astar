#include <iostream>

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

#include "st_a_star.hpp"
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <valarray>

using namespace std;

const int PORT = 12345;
const int BACKLOG = 5;
const int BUFFER_SIZE = 1024;

queue<int> g_connQueue;
mutex g_connQueueMutex;
condition_variable g_connQueueCV;

void processConnection(int connfd, char* message) {
    // Обмен данными с клиентом
    char buffer[BUFFER_SIZE];
    int bytesRead = recv(connfd, buffer, BUFFER_SIZE, 0);
    if (bytesRead < 0) {
        cerr << "Ошибка при чтении из сокета" << endl;
        return;
    }
    buffer[bytesRead] = '\0';
    cout << "Получено сообщение от клиента: " << buffer << endl;

    // Отправка данных клиенту
    //const char* message = "Привет, клиент!";
    int bytesSent = send(connfd, message, strlen(message), 0);
    if (bytesSent < 0) {
        cerr << "Ошибка при отправке данных клиенту" << endl;
    } else {
        cout << "Данные успешно отправлены клиенту" << endl;
    }

    close(connfd);
}

void serverLoop() {
    // Создание сокета
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        cerr << "Ошибка при создании сокета" << endl;
        exit(EXIT_FAILURE);
    }

    // Настройка адреса и порта
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);

    // Привязка сокета к адресу и порту
    if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        cerr << "Ошибка при привязке сокета к адресу и порту" << endl;
        exit(EXIT_FAILURE);
    }

    // Ожидание подключений
    if (listen(sockfd, BACKLOG) < 0) {
        cerr << "Ошибка при ожидании подключений" << endl;
        exit(EXIT_FAILURE);
    }

    while (true) {
        // Принятие соединения
        int connfd = accept(sockfd, nullptr, nullptr);
        if (connfd < 0) {
            cerr << "Ошибка при принятии соединения" << endl;
            continue;
        }

        unique_lock<mutex> lock(g_connQueueMutex);
        g_connQueue.push(connfd);
        lock.unlock();

        // Уведомление обработчиков о новом соединении
        g_connQueueCV.notify_one();
    }
}

void workerLoop(int argc, char* argv[]) {


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
      return;
    }
    } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return;
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
      return;
    }
    startStatesSet.insert(s);
    }

    Environment env(dimx, dimy, obstacles);
    StAStar stAStar{env, lifelong, randSeed, goals, startStates};

    Timer timer;
    bool success = stAStar.search(startStates, dimx, dimy);
    timer.stop();
    std::cout << "  runtime: " << timer.elapsedSeconds() << std::endl;

    string str;

    if (success) {

      std::vector<std::vector<Location>> solution = stAStar.getSolution();

      std::cout << "Planning successful! " << std::endl;
      //int cost = 0;
      int makespan = 0;
  //    for (const auto& s : solution) {
  //      cost += s.cost;
  //      makespan = std::max<int>(makespan, s.cost);
  //    }

      stringstream out;
      out << "statistics:" << std::endl;
      out << "  cost: " << 0 << std::endl;
      out << "  makespan: " << makespan << std::endl;
      out << "  runtime: " << timer.elapsedSeconds() << std::endl;
      out << "  highLevelExpanded: " << 0 << std::endl;
      out << "  lowLevelExpanded: " << 0 << std::endl;
      out << "schedule:" << std::endl;

      size_t a = 0;
      //for (size_t a = 0; a < solution.size(); ++a) {
        out << "  agent" << a << ":" << std::endl;
        for (size_t t = 0; t < solution[a].size(); ++t) {
          out << "    - x: " << solution[a][t].x << std::endl
              << "      y: " << solution[a][t].y << std::endl
              << "      t: " << t << std::endl;
        }
      //}

    str = out.str();

    } else {
      str = "Planning NOT successful!";
    }

    while (true) {
        // Извлечение соединения из очереди
        int connfd;
        unique_lock<mutex> lock(g_connQueueMutex);
        g_connQueueCV.wait(lock, [] {
            return !g_connQueue.empty();
        });
        connfd = g_connQueue.front();
        g_connQueue.pop();
        lock.unlock();

        // char* message = "Привет, клиент!";

        char* message = const_cast<char*>(str.c_str());
        // Обработка соединения

        // Обмен данными с клиентом
        char buffer[BUFFER_SIZE];
        int bytesRead = recv(connfd, buffer, BUFFER_SIZE, 0);
        if (bytesRead < 0) {
            cerr << "Ошибка при чтении из сокета" << endl;
            return;
        }
        buffer[bytesRead] = '\0';
        cout << "Получено сообщение от клиента: " << buffer << endl;

        std::stringstream ss(buffer);

        int x;
        int y;
        float time;
        bool hasCargo;

        ss >> x >> y >> time >> hasCargo;

        std::vector<Location> oneSolution = stAStar.freeAgent(x, y, time, hasCargo, dimx, dimy);

        stringstream ssSol;
        for (size_t t = 0; t < oneSolution.size(); ++t) {
          ssSol << "    - x: " << oneSolution[t].x << std::endl
                << "      y: " << oneSolution[t].y << std::endl
                << "      t: " << t << std::endl;
        }
        str = ssSol.str();
        message = const_cast<char*>(str.c_str());

#ifdef DEBUG_OUPUT
        cout << "Отправленное сообщение:" << endl;
        cout << message << endl;
#endif

        int bytesSent = send(connfd, message, strlen(message), 0);
        if (bytesSent < 0) {
            cerr << "Ошибка при отправке данных клиенту" << endl;
        } else {
            cout << "Данные успешно отправлены клиенту" << endl;
        }

        close(connfd);

    }
}

int main(int argc, char* argv[]) {

    // Запуск сервера в отдельном потоке
    thread serverThread(serverLoop);

    // Запуск нескольких потоков обработчиков соединений
//    const int NUM_WORKERS = 4;
//    thread workerThreads[NUM_WORKERS];

//    for (int i = 0; i < NUM_WORKERS; i++) {
//        workerThreads[i] = thread(workerLoop, argc, argv);
//    }

    workerLoop(argc, argv);

    // Ожидание завершения сервера и потоков обработчиков
//    serverThread.join();
//    for (int i = 0; i < NUM_WORKERS; i++) {
//        workerThreads[i].join();
//    }

    return 0;
}
