#ifndef __ALGORITHM_HPP__
#define __ALGORITHM_HPP__

#include "graph.hpp"
#include <utility>
#include <deque>

void findPaths(const Graph& g, int src, int dest, std::deque<std::pair<Path, int>>& paths);

Path widestPathBruteForce(const Graph& g, int src, int dest);

Path widestPathDijkstra(const Graph& g, int src, int dest);

Path widestPathKnowingBottleneck(const Graph& g, int src, int dest, int bottleneck);

Path widestPathMedianEdgeWeight(const Graph& g, int src, int dest);

int median_of_medians(int* a, int n, int k);

#endif
