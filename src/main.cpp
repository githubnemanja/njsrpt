#include <stdio.h>
#include <stdlib.h>
#include "time.h"
#include <fstream>

#include "graph.hpp"
#include "algorithm.hpp"

// flag za debagovanje
#define DEBUG 0

void populateGraph1(Graph& g){
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

void populateGraph2(Graph& g){
    for(int i = 2; i < g.size(); ++i){
        g.addEdge(i, i - 1, 10);
    }
    for(int i = 2; i < g.size(); ++i){
        g.addEdge(0, i, i);
    }
}

void populateGraph3(Graph& g){
    srand(time(0));

    for(int i = 0; i < g.size(); ++i){
        for(int j = 0; j < g.size(); ++j){
            if(i != j){
                g.addEdge(i, j, rand());
            }
        }
    }
}

void populateGraph4(Graph& g){
    srand(time(0));

    for(int i = 0; i < g.size(); ++i){
        for(int j = i + 1; j < g.size(); ++j){
            if(i != j){
                int weight = rand();
                g.addEdge(i, j, weight);
                g.addEdge(j, i, weight);
            }
        }
    }
}

void check_result(const Graph& g, const Path& path){
    int bottleneck;
    bool result;

    result = g.getMinEdge(path, bottleneck);
    std::cout << "[check_result] ";
    if(result){
        std::cout << "bottleneck=" << bottleneck;
    }
    else{
        std::cout << "[ERROR] Path does not exist!";
    }
    std::cout << std::endl;
}

// Ispisuje na standardni izlaz naziv funkcije name, kao i vreme izvrsavanje
// time_s predstavlja vreme pocetka izvrsavanja funkcije
// time_e predstavlja vreme zavrsetka izvrsavanja funkcije
void printTimeSpecs(std::string name, struct timespec time_s, struct timespec time_e){
    std::cout << "[" << name << "]" << " [ms]:" <<
    (double)(time_e.tv_nsec - time_s.tv_nsec) / 1000000.0 +
    (double)(time_e.tv_sec - time_s.tv_sec) * 1000.0 << " ";
}

void runTests(Graph& g, int src, int dest){
    std::vector<std::pair<Path, int>> ps;
    Path path;
    struct timespec time_s, time_e;

    std::cout << "--------------------------------" << std::endl;
    std::cout << "[runTests] src=" << src << ", dest=" << dest << std::endl;
    std::cout << "--------------------------------" << std::endl;

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    //path = widestPathBruteForce(g, src, dest);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printTimeSpecs("widestPathBruteForce", time_s, time_e);
    printPath(path);
    check_result(g, path);

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathDijkstra(g, src, dest);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printTimeSpecs("widestPathDijkstra", time_s, time_e);
    printPath(path);
    check_result(g, path);

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathMedianEdgeWeight(g, 0, 1);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printTimeSpecs("widestPathMedianEdgeWeight", time_s, time_e);
    printPath(path);
    check_result(g, path);

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathInUndirectedGraph(g, 0, 1);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printTimeSpecs("widestPathInUndirectedGraph", time_s, time_e);
    printPath(path);
    check_result(g, path);

    std::cout << "--------------------------------" << std::endl;
}

int main(){
    Graph g(30);

    #if DEBUG == 1
    std::ofstream out("out.txt");
    std::streambuf *coutbuf = std::cout.rdbuf();
    std::cout.rdbuf(out.rdbuf());
    #endif

    populateGraph4(g);
    runTests(g, 0, 1);

    #if DEBUG == 1
    std::cout.rdbuf(coutbuf);
    #endif
    return 0;
}