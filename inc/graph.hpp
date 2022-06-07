#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <deque>
#include <vector>

typedef  std::deque<int> Queue;

typedef  std::deque<int> Path;

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
    void findPaths(int src, int dest, std::deque<std::pair<Path, int>>& paths) const;
    int connected_components(std::vector<int>& com) const;

    std::vector<std::vector<Edge>> adj;

    private:
    void findPaths_dfs(int src, int dest, std::vector<bool>& visited, Path p, std::deque<std::pair<Path, int>>& paths, int minEdge) const;
    void connected_components_dfs(int src, std::vector<bool>& visited, int comp_id, std::vector<int>& comp) const;
};

void DFS(const Graph& g, int src, int * visited);

void BFS(const Graph& g, int src);

void printPath(const Path& p);

void printPaths(const std::deque<std::pair<Path, int>>& paths);

#endif