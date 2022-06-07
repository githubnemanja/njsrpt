#include <stdio.h>
#include <stdlib.h>
#include "graph.hpp"

AdjListNode * newAdjListNode(int dest, int weight){
    AdjListNode * _new = NULL;

    _new = (AdjListNode*)malloc(sizeof(AdjListNode));
    _new->dest = dest;
    _new->weight = weight;
    _new->next = NULL;

    return _new;
}

void freeAdjList(AdjList list){
    AdjListNode * prev = NULL;
    AdjListNode * cur  = NULL;

    prev = list.head;
    while(prev != NULL){
        cur = prev->next;
        free(prev);
        prev = cur;
    }
}

Graph * newGraph(int size){
    int i = 0;
    Graph * g = NULL;

    g = (Graph*)malloc(sizeof(Graph));
    g->size = size;
    g->adj = (AdjList*)malloc(size*sizeof(AdjList));

    for(i = 0; i < size; ++i){
        g->adj[i].head = NULL;
    }

    return g;
}

void addEdge(Graph * g, int src, int dest, int weight){
    AdjListNode * _new = NULL;

    if(g == NULL){
        return;
    }
    
    _new = newAdjListNode(dest, weight);
    _new->next = g->adj[src].head;
    g->adj[src].head = _new;
}

void printGraph(Graph * g){
    int i = 0;
    AdjListNode * cur = NULL;

    if(g == NULL){
        printf("null\n");
        return;
    }

    printf("GRAPH edges: (src, dest) = weight\n");

    for(i=0; i<g->size; ++i){
        for(cur = g->adj[i].head; cur != NULL; cur = cur->next){
            printf("(%d, %d) = %d\t", i, cur->dest, cur->weight);
        }
    }

    printf("\n--------------------------------\n");
}

void freeGraph(Graph * g){
    int i = 0;

    if(g == NULL){
        return;
    }

    for(i = 0; i < g->size; ++i){
        freeAdjList(g->adj[i]);
    }
    free(g->adj);
    free(g);
}

void DFS(Graph * g, int s, int * visited){
    int i = 0;
    AdjListNode * cur = NULL;

    if(g == NULL || visited == NULL){
        return;
    }

    visited[s] = 1;
    printf("%d\n", s);

    for(cur = g->adj[s].head; cur != NULL; cur = cur->next){
        if(visited[cur->dest] == 0){
            DFS(g, cur->dest, visited);
            printf("(%d, %d)\n", s, cur->dest);
        }
    }
}

void BFS(Graph * g, int src){
    int v = 0;
    int * visited = NULL;
    AdjListNode * cur = NULL;
    Queue q;

    if(g == NULL){
        return;
    }

    visited = (int*)malloc(g->size*sizeof(int));
    for(v = 0; v < g->size; ++v){
        visited[v] = 0;
    }
    visited[src] = 1;

    q.push_back(src);

    while(q.empty()){
        v = q.front();
        q.pop_front();
        printf("%d\n", v);
        for(cur = g->adj[v].head; cur != NULL; cur = cur->next){
            if(visited[cur->dest] == 0){
                visited[cur->dest] = 1;
                printf("(%d,%d)\n", v, cur->dest);
                q.push_back(cur->dest);
            }
        }
    }
}
