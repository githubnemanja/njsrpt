#include <stdio.h>
#include <stdlib.h>
#include "time.h"

#include "graph.hpp"
#include "algorithm.hpp"

void populateGraph1(Graph& g, int size){
    if(size < 5){
        return;
    }
    addEdge(g, 0, 1, 1);
    addEdge(g, 0, 2, 2);
    addEdge(g, 2, 1, 2);
    addEdge(g, 1, 0, 2);
    addEdge(g, 2, 4, 200);
    addEdge(g, 4, 3, 200);
    addEdge(g, 3, 1, 200);
}

void populateGraph2(Graph& g, int size){
    for(int i = 2; i < size; ++i){
        addEdge(g, i, i - 1, 10);
    }
    for(int i = 2; i < size; ++i){
        addEdge(g, 0, i, i);
    }
}

void populateGraph3(Graph& g, int size){
    srand(time(0));

    for(int i = 0; i < size; ++i){
        for(int j = 0; j < size; ++j){
            if(i != j){
                addEdge(g, i, j, rand());
            }
        }
    }
}

void runTests(Graph& g, int src, int dest){
    std::deque<std::pair<Path, int>> ps;
    Path path;
    struct timespec time_s, time_e;

    printf("--------------------------------\n");
    printf("[runTests] src=%d, dest=%d\n", src, dest);
    //printGraph(g);
    printf("--------------------------------\n");

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    findPaths(g, src, dest, ps);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printf("[findPaths] [ms]:%f\n",(double)(time_e.tv_nsec - time_s.tv_nsec) / 1000000.0 + (double)(time_e.tv_sec - time_s.tv_sec) * 1000.0);
    //printPaths(ps);S

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathBruteForce(g, src, dest);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printf("[widestPathBruteForce] [ms]:%f [path]:",(double)(time_e.tv_nsec - time_s.tv_nsec) / 1000000.0 + (double)(time_e.tv_sec - time_s.tv_sec) * 1000.0);
    printPath(path);

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathDijkstra(g, src, dest);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printf("[widestPathDijkstra] [ms]:%f [path]:",(double)(time_e.tv_nsec - time_s.tv_nsec) / 1000000.0 + (double)(time_e.tv_sec - time_s.tv_sec) * 1000.0);
    printPath(path);

    clock_gettime(CLOCK_MONOTONIC, &time_s);
    path = widestPathMedianEdgeWeight(g, 0, 1);
    clock_gettime(CLOCK_MONOTONIC, &time_e);
    printf("[widestPathMedianEdgeWeight] [ms]:%f [path]:",(double)(time_e.tv_nsec - time_s.tv_nsec) / 1000000.0 + (double)(time_e.tv_sec - time_s.tv_sec) * 1000.0);
    printPath(path);

    printf("--------------------------------\n");
}

int main(){
    int size = 11;
    Graph g = std::vector<std::vector<Edge>>(size);

    populateGraph3(g, size);

    runTests(g, 0, 1);

    return 0;
}