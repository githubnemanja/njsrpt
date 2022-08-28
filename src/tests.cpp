#include <stdio.h>
#include <stdlib.h>
#include "time.h"
#include <random>
#include <climits>
#include <tuple>
#include <iomanip>
#include "graph.hpp"
#include "algorithm.hpp"
#include "tests.hpp"

// Ako je PRINT_TEST flag jednak 1 stampa se rezultat svakog testa, ako je PRINT_TEST jednak 0 stampa se samo ukupan rezultat
#define PRINT_TEST 0

// -----------------------------------------------------------------------------------------------------------------------
// Definicije lokalnih funkcija
// -----------------------------------------------------------------------------------------------------------------------

// Bira 2 razlicita cvora iz grafa velicine size i smesta ih u src i dest
void generateVertices(int size, int& src, int& dest){
    std::default_random_engine generator(std::random_device{}());
    std::uniform_int_distribution<int> vertices_distribution(0, size - 1);

    src = vertices_distribution(generator);
    dest = vertices_distribution(generator);
    while(src == dest){
        dest = vertices_distribution(generator);
    }
}

// Generise grupu uzorka group
void generateGroup(Graph& g, int group){
    if(group == 1){
        generateEdges(g, g.size());
    }
    else if(group == 2){
        generateEdges(g, g.size() * log2(g.size()));
    }
    else if(group == 3){
        generateEdges(g, g.size() * (g.size() - 1) / 3);
    }
}

// Dodaje O(m) random grana u graf g i postavlja directed polje
void generateEdges(Graph& g, int m){
    std::default_random_engine generator(std::random_device{}());
    std::uniform_int_distribution<int> weights_distribution(INT_MIN + 1, INT_MAX - 1);

    int upper_bound = g.size() * (g.size() - 1) / 2;
    if(m > upper_bound){
        m = upper_bound;
    }

    int edges_num = 0;
    while(edges_num < m){
        int u, v, weight;
        generateVertices(g.size(), u, v);
        weight = weights_distribution(generator);
        if(g.addEdge(u, v, weight)){
            ++edges_num;
        }
    }
}

void generateConstantO1Edges(Graph& g){
    if(g.size() < 5){
        return;
    }
    g.addEdge(0, 1, 1);
    g.addEdge(0, 2, 2);
    g.addEdge(2, 1, 2);
    g.addEdge(1, 0, 2);
    g.addEdge(2, 4, 200);
    g.addEdge(4, 3, 200);
    g.addEdge(3, 1, 200);
}

void generateConstantOVEdges(Graph& g){
    for(int i = 2; i < g.size(); ++i){
        g.addEdge(i, i - 1, 10);
    }
    for(int i = 2; i < g.size(); ++i){
        g.addEdge(0, i, i);
    }
}

// Funkcija check_result proverava da li je rezultat algoritma ispravan
bool check_result(const Graph& g, int src, int dest, const Path& path, int& bottleneck, bool first_test){
    if(path.empty()){
        if(g.connected(src, dest)){
            std::cout << "[ERROR] Alg. returned null path, but path between vertices exists! " << std::endl;
            return false;
        }
        else{
            return true;
        }
    }

    int prevbottleneck = bottleneck;
    if(g.getMinEdge(path, bottleneck) == false){
        std::cout << "[ERROR] Path that alg. returned does not exist in graph! " << std::endl;
        return false;
    }

    if(!first_test && prevbottleneck != bottleneck){
        std::cout << "[ERROR] Different path output! " << 
        "previous alg. returned path with bottleneck " << prevbottleneck << 
        ", this alg. returned path with bottleneck " << bottleneck << std::endl;
        return false;
    }
    return true;
}

// Ispisuje na standardni izlaz naziv funkcije name, kao i vreme izvrsavanja
// time_s predstavlja vreme pocetka izvrsavanja funkcije
// time_e predstavlja vreme zavrsetka izvrsavanja funkcije
double printTimeSpecs(std::string name, struct timespec time_s, struct timespec time_e){
    double duration = (double)(time_e.tv_nsec - time_s.tv_nsec) / 1000000.0 +
                      (double)(time_e.tv_sec - time_s.tv_sec) * 1000.0;
#if PRINT_TEST
    std::cout << "[" << name << "]" << " [ms]:" << duration << " ";
#endif
    return duration;
}

// Funkcija runTests izvrsava testove tj. widestPath(g, src, dest)
// za sve razlicite implementacije algoritma widestPath.
// Alg. widestPathBruteForce je ukljucen u testove akko je flag incbf true.
// Vremena izvrsavanja algoritama se vracaju kroz times
bool runTests(Graph& g, int src, int dest, std::vector<double>& times, bool incbf){
    std::vector<std::pair<Path, int>> ps;
    Path path;
    struct timespec time_s, time_e;
    double duration;
    int bottleneck;
    int prevbottleneck;
    bool result = true;

#if PRINT_TEST
    std::cout << "--------------------------------" << std::endl;
    std::cout << "[runTests] src=" << src << ", dest=" << dest << std::endl;
    std::cout << "--------------------------------" << std::endl;
#endif

    if(incbf){
        clock_gettime(CLOCK_MONOTONIC, &time_s);
        path = widestPathBruteForce(g, src, dest);
        clock_gettime(CLOCK_MONOTONIC, &time_e);
        duration = printTimeSpecs("widestPathBruteForce", time_s, time_e);
        times.push_back(duration);
        printPath(path);
        result = check_result(g, src, dest, path, bottleneck, true) ? result : false;
    }
    else{
#if PRINT_TEST
        std::cout << "[widestPathBruteForce] [not executed] " << std::endl;
#endif
        times.push_back(0);
    }

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathDijkstra(g, src, dest);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    duration = printTimeSpecs("widestPathDijkstra", time_s, time_e);
    times.push_back(duration);
    printPath(path);
    result = check_result(g, src, dest, path, bottleneck, incbf ? false : true) ? result : false;

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathMedianEdgeWeight(g, src, dest);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    duration = printTimeSpecs("widestPathMedianEdgeWeight", time_s, time_e);
    times.push_back(duration);
    printPath(path);
    result = check_result(g, src, dest, path, bottleneck, true) ? result : false;

    if(g.isDirected() == false){
        clock_gettime(CLOCK_MONOTONIC, &time_s);
        path = widestPathInUndirectedGraph(g, src, dest);
        clock_gettime(CLOCK_MONOTONIC, &time_e);
        duration = printTimeSpecs("widestPathInUndirectedGraph", time_s, time_e);
        times.push_back(duration);
        printPath(path);
        result = check_result(g, src, dest, path, bottleneck, true) ? result : false;
    }
    else{
#if PRINT_TEST
        std::cout << "[widestPathInUndirectedGraph] [not executed] " << std::endl;
#endif
        times.push_back(0);
    }

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathEdgesOrdering(g, src, dest);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    duration = printTimeSpecs("widestPathEdgesOrdering", time_s, time_e);
    times.push_back(duration);
    printPath(path);
    result = check_result(g, src, dest, path, bottleneck, false) ? result : false;

#if PRINT_TEST    
    if(result){
        std::cout << "[success]" << std::endl;
    }
    else{
        std::cout << "[ERROR][TEST FAILED]" << std::endl;
    }
    std::cout << "--------------------------------" << std::endl;
#endif

    return result;
}

// Funkcija updateAvgs se koristi za racunanje prosecnog vremena izvravanja testova
void updateAvgs(std::vector<std::pair<std::string, std::tuple<double, double, double>>>& avgs, std::vector<double> times, int divident){
    for(int i = 0; i < avgs.size(); ++i){
        if(std::get<0>(avgs[i].second) > times[i]){
            std::get<0>(avgs[i].second) = times[i];
        }

        std::get<1>(avgs[i].second) += times[i] / divident;

        if(std::get<2>(avgs[i].second) < times[i]){
            std::get<2>(avgs[i].second) = times[i];
        }
    }
}

// Funkcija printAvgs stampa na standardni izlaz prosecno vreme izvravanja testiranig algoritama
void printAvgs(std::vector<std::pair<std::string, std::tuple<double, double, double>>> avgs){
    std::cout << "--------------------------------" << std::endl;
    std::cout << "[AVERAGE EXECUTION TIME]" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    std::cout << std::setw(28) << std::left << "format: [alg. name" << "][ms]: " << std::setw(15) << std::left << "t_min(ms) " << std::setw(15) << std::left << "t_avg(ms) " << std::setw(15) << std::left << "t_max(ms) " << std::endl;
    std::cout << "--------------------------------" << std::endl;
    for(int i = 0; i < avgs.size(); ++i){
        std::cout << "["<< std::setw(27) << std::left << avgs[i].first << "][ms]: "
                        << std::setw(14) << std::left << std::get<0>(avgs[i].second) << " "
                        << std::setw(14) << std::left << std::get<1>(avgs[i].second) << " "
                        << std::setw(14) << std::left << std::get<2>(avgs[i].second) << " "
                        << std::endl;
    }
}

// Funkcija printResult stampa na standardni izlaz rezultat testova,
// pri cemu je succcess broj testova koji su se zavrsili uspesno,
// dok je total ukupa broj testova
void printResult(int success, int total){
    std::cout << "--------------------------------" << std::endl;
    if(success == total){
        std::cout << "All tests passed!" << std::endl;
    }
    else{
        std::cout << "Tests results: " << "[" << success << "/" << total << "] tests passed." << std::endl;
    }
    std::cout << "--------------------------------" << std::endl;
}
