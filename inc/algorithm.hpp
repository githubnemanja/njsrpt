#ifndef __ALGORITHM_HPP__
#define __ALGORITHM_HPP__

#include "graph.hpp"

// Koristi se za deklaraciju boost::heap::fibonacci_heap
struct HeapNode{
    int key;
    int val;

    HeapNode(int key, int val)
      : key(key), val(val)
    { }
};

// Koristi se za deklaraciju boost::heap::fibonacci_heap
// U korenu hipa se cuva maksimum
struct CompareHeapNode{
    bool operator()(const HeapNode& n1, const HeapNode& n2) const{
        return n1.key < n2.key;
    }
};

Path widestPathBruteForce(const Graph& g, int src, int dest);

Path widestPathDijkstra(const Graph& g, int src, int dest);

Path widestPathKnowingBottleneck(const Graph& g, int src, int dest, int bottleneck);

Path widestPathMedianEdgeWeight(const Graph& g, int src, int dest);

int median_of_medians(int* a, int n, int k);

Path widestPathInUndirectedGraph(const Graph& g, int src, int dest);

#endif
