#ifndef __ALGORITHM_HPP__
#define __ALGORITHM_HPP__

#include "graph.hpp"

// -----------------------------------------------------------------------------------------------------------------------
// Definicije lokalnih tipova
// -----------------------------------------------------------------------------------------------------------------------

// Koristi se za deklaraciju boost::heap::fibonacci_heap
struct HeapNode{
    int key;
    int val;

    HeapNode(int key, int val)
      : key(key), val(val)
    { }
};

// Koristi se za uporedjivanje objekata HeapNode
// Koristi se za deklaraciju boost::heap::fibonacci_heap
// U korenu hipa se cuva maksimum
struct CompareHeapNode{
    bool operator()(const HeapNode& n1, const HeapNode& n2) const{
        return n1.key < n2.key;
    }
};

// -----------------------------------------------------------------------------------------------------------------------
// Deklaracije funkcija za pronalazenje najsireg puta
// -----------------------------------------------------------------------------------------------------------------------

Path widestPathBruteForce(const Graph& g, int src, int dest);
Path widestPathDijkstra(const Graph& g, int src, int dest);
Path widestPathMedianEdgeWeight(const Graph& g, int src, int dest);
Path widestPathInUndirectedGraph(const Graph& g, int src, int dest);
Path widestPathEdgesOrdering(const Graph& g, int src, int dest);

#endif
