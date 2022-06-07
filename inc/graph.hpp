#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <deque>
#include <vector>

struct Edge{
    int dest;
    int weight;

    Edge(int dest, int weight): dest(dest), weight(weight) {}
};

class Graph{
    public:
    Graph(int size);
    int size() const;
    std::vector<Edge>& operator[] (int index);
    const std::vector<Edge>& operator[] (int index) const;
    void addEdge(int src, int dest, int weight);
    void printGraph() const;

    std::vector<std::vector<Edge>> adj;
};

typedef  std::deque<int> Queue;

typedef  std::deque<int> Path;

void DFS(const Graph& g, int src, int * visited);

void BFS(const Graph& g, int src);

void printPath(const Path& p);

void printPaths(const std::deque<std::pair<Path, int>>& paths);

#endif