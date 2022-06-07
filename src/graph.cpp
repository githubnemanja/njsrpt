#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "graph.hpp"

Graph::Graph(int size){
    adj.resize(size);
}

int Graph::size() const{
    return adj.size();
}

std::vector<Edge>& Graph::operator[] (int index){
    return adj[index];
}

const std::vector<Edge>& Graph::operator[] (int index) const{
    return adj[index];
}

void Graph::addEdge(int src, int dest, int weight){
    adj[src].push_back({dest, weight});
}

void Graph::printGraph() const{
    if(size() == 0){
        printf("null\n");
        return;
    }

    printf("GRAPH edges: (src, dest) = weight\n");

    for(int i = 0; i < size(); ++i){
        for(auto & edge : adj[i]){
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

void printPath(const Path& p){
    bool fst = true;
    for(auto i = p.begin(); i != p.end(); ++i){
        if(fst){
            fst = false;
        }
        else{
            std::cout << "->";
        }
        std::cout << *i;
    }
    std::cout << std::endl;
}

void printPaths(const std::deque<std::pair<Path, int>>& paths){
    for(auto i = paths.cbegin(); i != paths.cend(); ++i){
        std::cout << "[minEdge]:" << i->second << ", [path]:";
        printPath(i->first);
    }
}