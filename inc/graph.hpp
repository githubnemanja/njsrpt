#ifndef __GRAPH_HPP__
#define __GRAPH_HPP__

#include <deque>

typedef struct AdjListNode{
    int dest;
    int weight;
    struct AdjListNode * next;
}AdjListNode;

typedef struct AdjList{
    AdjListNode * head;
}AdjList;

typedef struct Graph{
    int size;
    AdjList * adj;
}Graph;

typedef  std::deque<int> Queue;

typedef  std::deque<int> Path;

AdjListNode * newAdjListNode(int dest, int weight);

void freeAdjList(AdjList list);

Graph * newGraph(int size);

void addEdge(Graph * g, int src, int dest, int weight);

void printGraph(Graph * g);

void freeGraph(Graph * g);

void DFS(Graph * g, int s, int * visited);

void BFS(Graph * g, int src);

#endif