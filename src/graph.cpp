#include <stdio.h>
#include <stdlib.h>
#include "graph.hpp"

void addEdge(Graph& g, int src, int dest, int weight){
    g[src].push_back({dest, weight});
}

void printGraph(const Graph& g){
    if(g.size() == 0){
        printf("null\n");
        return;
    }

    printf("GRAPH edges: (src, dest) = weight\n");

    for(int i = 0; i < g.size(); ++i){
        for(auto & edge : g[i]){
            printf("(%d, %d) = %d\t", i, edge.dest, edge.weight);
        }
    }

    printf("\n--------------------------------\n");
}

void DFS(const Graph& g, int src, int * visited){
    if(visited == NULL){
        return;
    }

    visited[src] = 1;
    printf("%d\n", src);

    for(auto & cur : g[src]){
        if(visited[cur.dest] == 0){
            DFS(g, cur.dest, visited);
            printf("(%d, %d)\n", src, cur.dest);
        }
    }
}

void BFS(const Graph& g, int src){
    int v = 0;
    Queue q;
    bool visited[g.size()] = {false};

    visited[src] = true;

    q.push_back(src);

    while(q.empty()){
        v = q.front();
        q.pop_front();
        printf("%d\n", v);
        for(auto & cur : g[v]){
            if(visited[cur.dest] == false){
                visited[cur.dest] = true;
                printf("(%d,%d)\n", v, cur.dest);
                q.push_back(cur.dest);
            }
        }
    }
}
