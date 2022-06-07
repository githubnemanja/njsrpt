#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <deque>
#include <vector>

struct Edge{
    int dest;
    int weight;

    Edge(int dest, int weight): dest(dest), weight(weight) {}
};

typedef std::vector<std::vector<Edge>> Graph;

typedef  std::deque<int> Queue;

typedef  std::deque<int> Path;

void addEdge(Graph& g, int src, int dest, int weight);

void printGraph(const Graph& g);

void DFS(const Graph& g, int s, int * visited);

void BFS(const Graph& g, int src);

#endif