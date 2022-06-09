#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <deque>
#include <vector>
#include <string>
#include <iostream>

typedef  std::deque<int> Queue;

typedef  std::deque<int> Path;

// Koristi se u listi susedstava cvora grafa
struct Edge{
    int dest;
    int weight;

    Edge() {}
    Edge(int dest, int weight): dest(dest), weight(weight) {}
};

// Kompletan identifikator grane grafa
struct EdgeId{
    int src;
    int dest;
    int weight;

    EdgeId(int src, int dest, int weight)
        : src(src), dest(dest), weight(weight)
        {}
};

// Koristi se za uporedjivanje objekata EdgeId
// Koristi se u std::sort
struct CompareEdgeIds{
    bool operator()(const EdgeId& e1, const EdgeId& e2) const{
        return e1.weight < e2.weight;
    }
};

class Graph{
    public:
    Graph(int size);
    Graph(const Graph& g);
    int size() const;
    std::vector<Edge>& operator[] (int index);
    const std::vector<Edge>& operator[] (int index) const;
    void addEdge(int src, int dest, int weight);
    void deleteEdges(int bottleneck);
    void getEdgeIds(std::vector<EdgeId>& edges) const;
    std::string toString() const;
    void printGraph() const;
    void findPath(int src, int dest, Path& path) const;
    void findPath(int src, int dest, int bottleneck, Path& path) const;
    void findPaths(int src, int dest, std::vector<std::pair<Path, int>>& paths) const;
    int connected_components(std::vector<int>& com) const;
    int connected_components(int bottleneck, std::vector<int>& com) const;
    void shrink(const std::vector<int>& comp, int comp_size);
    bool getMinEdge(const Path& path, int& min_edge) const;
    bool isConnected(int src, int dest) const;
    bool isConnected(int src, int dest, int bottleneck) const;

    std::vector<std::vector<Edge>> adj;

    private:
    void findPaths_dfs(int src, int dest, std::vector<bool>& visited, Path p, std::vector<std::pair<Path, int>>& paths, int minEdge) const;
    void connected_components_dfs(int src, std::vector<bool>& visited, int comp_id, std::vector<int>& comp) const;
    void connected_components_dfs(int src, std::vector<bool>& visited, int bottleneck, int comp_id, std::vector<int>& comp) const;
};

void DFS(const Graph& g, int src, int * visited);

void BFS(const Graph& g, int src);

void printPath(const Path& p);

void printPaths(const std::vector<std::pair<Path, int>>& paths);

void swap(Edge& e1, Edge& e2);

void swap(EdgeId& e1, EdgeId& e2);

std::ostream& operator<< (std::ostream& os, const Graph& g);

#endif