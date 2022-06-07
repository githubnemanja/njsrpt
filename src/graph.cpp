#include <stdio.h>
#include <iostream>
#include <climits>
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


void Graph::findPaths_dfs(int src, int dest, std::vector<bool>& visited, Path p, std::vector<std::pair<Path, int>>& paths, int minEdge) const{
    visited[src] = true;

    if(src == dest){
        paths.push_back(std::make_pair(p, minEdge));
    }
    else{
        for(auto & cur : adj[src]){
            if(visited[cur.dest] == false){
                p.push_back(cur.dest);
                findPaths_dfs(cur.dest, dest, visited, p, paths, 
                            minEdge < cur.weight ? minEdge : cur.weight);
                p.pop_back();
            }
        }
    }

    visited[src] = 0;
}

// Funkcija findPaths pronalazi sve puteve u grafu izmedju cvorova src i dest. Od src do dest.
// Putevi se smestaju u strukturu paths.
void Graph::findPaths(int src, int dest, std::vector<std::pair<Path, int>>& paths) const{
    Path p;
    std::vector<bool> visited(size(), false);

    p.push_back(src);

    findPaths_dfs(src, dest, visited, p, paths, INT_MAX);
}


void Graph::connected_components_dfs(int src, std::vector<bool>& visited, int comp_id, std::vector<int>& comp) const{
    visited[src] = true;
    comp[src] = comp_id;

    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false){
            connected_components_dfs(cur.dest, visited, comp_id, comp);
        }
    }
}

// Funkcija connected_components() pronalazi povezane komponente u neusmerenom grafu
// koristeci DFS pristup. Povezana komponenta je skup cvorova koji su svi dostizni
// medju sobom. Rezultat algoritma se cuva u strukturi comp, koja predstavlja niz
// u kome je id komponente dodeljen indeksu cvora. Funkcija connected_components
// vraca ukupan broj komponenti kao return.
int Graph::connected_components(std::vector<int>& comp) const{
    int comp_id = 0;
    std::vector<bool> visited(size(), false);

    for(int i = 0; i < size(); ++i){
        if(!visited[i]){
            connected_components_dfs(i, visited, comp_id, comp);
            comp_id++;
        }
    }

    return comp_id;
}


void DFS(const Graph& g, int src, std::vector<bool>& visited){
    visited[src] = true;
    printf("%d\n", src);

    for(auto & cur : g[src]){
        if(visited[cur.dest] == false){
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

void printPaths(const std::vector<std::pair<Path, int>>& paths){
    for(auto i = paths.cbegin(); i != paths.cend(); ++i){
        std::cout << "[minEdge]:" << i->second << ", [path]:";
        printPath(i->first);
    }
}