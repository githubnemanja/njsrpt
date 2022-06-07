#ifndef __ALGORITHM_HPP__
#define __ALGORITHM_HPP__

#include "graph.hpp"
#include <utility>
#include <deque>

void printPath(Path p);

void findPaths(Graph * g, int src, int dest, std::deque<std::pair<Path, int>>& paths);

Path widestPathBruteForce(Graph * g, int src, int dest);

Path widestPathDijkstra(Graph * g, int src, int dest);

Path widestPathKnowingBottleneck(Graph * g, int src, int dest, int bottleneck);

Path widestPathMedianEdgeWeight(Graph * g, int src, int dest);

int median_of_medians(int* a, int n, int k);

#endif
