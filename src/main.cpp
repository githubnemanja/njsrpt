#include <stdio.h>
#include <stdlib.h>
#include "time.h"

#include "graph.hpp"
#include "algorithm.hpp"

Graph * generateGraph1(){
    Graph * g = newGraph(5);

    addEdge(g, 0, 1, 1);
    addEdge(g, 0, 2, 2);
    addEdge(g, 2, 1, 2);
    addEdge(g, 1, 0, 2);
    addEdge(g, 2, 4, 200);
    addEdge(g, 4, 3, 200);
    addEdge(g, 3, 1, 200);

    return g;
}

Graph * generateGraph2(int size){
    int i = 0;
    Graph * g = NULL;

    g = newGraph(size);

    for(i = 2; i < size; ++i){
        addEdge(g, i, i - 1, 10);
    }
    for(i = 2; i < size; ++i){
        addEdge(g, 0, i, i);
    }

    return g;
}

Graph * generateGraph3(int size){
    int i = 0;
    int j = 0;
    Graph * g = NULL;

    g = newGraph(size);

    srand(time(0));

    for(i = 0; i < size; ++i){
        for(j = 0; j < size; ++j){
            if(i != j){
                addEdge(g, i, j, rand());
            }
        }
    }

    return g;
}

void runTests(Graph * g, int src, int dest){
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
    //printPaths(ps);

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
    Graph * g = NULL;

    g = generateGraph3(11);

    runTests(g, 0, 1);

    freeGraph(g);
    return 0;
}