#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <deque>
#include <vector>
#include <string>
#include <iostream>

typedef  std::deque<int> Queue;

typedef  std::deque<int> Path;

// Koristi se u listi susedstava cvora grafa
// Uredjeni par (src, dest, weigh) oznacava granu od cvora src do cvora dest, tezine weight
// Cvor src se dobija kao indeks vektora susedstava adj
// Polje order se koristi u slucaju kada su grane sortirane po tezini da oznaci redni broj grane
struct Edge{
    int dest;
    int weight;
    int order;

    Edge() {}
    Edge(int dest, int weight): dest(dest), weight(weight), order(0) {}

    std::string toString() const;
};

// Kompletan identifikator grane grafa
// Uredjeni par (src, dest, weigh) oznacava granu od cvora src do cvora dest, tezine weight
// Polje adrr se koristi kao pokazivac na granu grafa kada je to potrebno
// Polje addr se koristi u algoritmu widestPathEdgesOrdering
struct EdgeId{
    int src;
    int dest;
    int weight;
    Edge* addr;

    EdgeId(int src, int dest, int weight)
        : src(src), dest(dest), weight(weight), addr(nullptr)
        {}

    // Koristi se u algoritmu widestPathEdgesOrdering
    EdgeId(int src, int dest, int weight, Edge* addr)
        : src(src), dest(dest), weight(weight), addr(addr)
        {}
    
    std::string toString() const;
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
    bool isDirected() const;
    void setDirected(bool directed);
    std::vector<Edge>& operator[] (int index);
    const std::vector<Edge>& operator[] (int index) const;

    void addEdge(int src, int dest, int weight);
    void getEdgeIds(std::vector<EdgeId>& edges) const;
    void getEdges(int src, std::vector<EdgeId>& edges, int& minEdge, int& maxEdge) const;
    int deleteEdges(int bottleneck);

    bool getMinEdge(const Path& path, int& min_edge) const;

    bool connected(int src, int dest) const;
    bool connected(int src, int dest, int bottleneck) const;
    void findPath(int src, int dest, Path& path) const;
    void findPath(int src, int dest, int bottleneck, Path& path) const;
    int connected_components(std::vector<int>& com) const;
    int connected_components(int bottleneck, std::vector<int>& com) const;
    void shrink(const std::vector<int>& comp, int comp_size);

    std::string toString() const;
    void printGraph() const;

    private:
    bool connectedDFS(int src, int dest, std::vector<bool>& visited) const;
    bool connectedDFS(int src, int dest, std::vector<bool>& visited, int bottleneck) const;
    void findPathDFS(int src, std::vector<bool>& visited, std::vector<int>& pred) const;
    void findPathDFS(int src, std::vector<bool>& visited, int bottleneck, std::vector<int>& pred) const;
    void connected_components_dfs(int src, std::vector<bool>& visited, int comp_id, std::vector<int>& comp) const;
    void connected_components_dfs(int src, std::vector<bool>& visited, int bottleneck, int comp_id, std::vector<int>& comp) const;
    void getEdgesDFS(int src, std::vector<bool>& visited , std::vector<EdgeId>& edges, int& minEdge, int& maxEdge) const;

    // Lista susedstava grafa
    // Susedni cvorovi cvora v, nalaze se u adj[v]
    std::vector<std::vector<Edge>> adj;
    // Da li je graf usmeren. Koristi se da obustavi testiranje algoritama koji su impl samo za neusmerene grafove
    bool directed = true;
};

void swap(Edge& e1, Edge& e2);
void swap(EdgeId& e1, EdgeId& e2);

void formPath(int src, int dest, const std::vector<int>& pred, Path& path);
void printPath(const Path& p);
std::ostream& operator<< (std::ostream& os, const Graph& g);
std::string toString(std::vector<EdgeId> vec);
std::string toString(std::vector<int> vec);

#endif