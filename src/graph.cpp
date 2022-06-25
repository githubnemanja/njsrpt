#include <stdio.h>
#include <iostream>
#include <climits>
#include <iterator>
#include "graph.hpp"

Graph::Graph(int size){
    adj.resize(size);
}

Graph::Graph(const Graph& g){
    adj = g.adj;
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

// Obrisati sve grane cija je tezina manja od bottleneck i vratiti broj obrisanih grana
int Graph::deleteEdges(int bottleneck){
    int num_deleted = 0;

    for(int i = 0; i < size(); ++i){
        int new_size = adj[i].size();
        int j = 0;
        while(j < new_size){
            if(adj[i][j].weight < bottleneck){
                // trenutna grana je za brisanje
                // swapuj trenutnu i poslednju granu u adj[i]
                swap(adj[i][j], adj[i][new_size - 1]);
                // resize parametar koji ce doprineti da se kasnije obrise poslednja grana
                new_size--;
                // cuvaj broj obrisanih grana
                num_deleted++;
            }
            else{
                ++j;
            }
        }
        adj[i].resize(new_size);
    }

    return num_deleted;
}


// Funkcija getEdgeIds vraca sve grane grafa. Cuva ih u strukturi edges.
void Graph::getEdgeIds(std::vector<EdgeId>& edges) const{
    for(int i = 0; i < adj.size(); ++i){
        for(auto edge : adj[i]){
            edges.push_back({i, edge.dest, edge.weight});
        }
    }
}

std::string Graph::toString() const{
    std::string str = "[graph] size=" + std::to_string(size()) + " edges={";

    for(int i = 0; i < size(); ++i){
        for(auto & edge : adj[i]){
            str += " (" + std::to_string(i) + "," + std::to_string(edge.dest) + "," + std::to_string(edge.weight) + "," + std::to_string(edge.order) + ")";
        }
    }

    str += " }";

    return str;
}

void Graph::printGraph() const{
    std::cout << *this << std::endl;
}

// Funkcija findPath vraca put, ako postoji, od cvora src do cvora dest.
// Put se smesta u strukturu path.
void Graph::findPath(int src, int dest, Path& path) const{
    int v;
    std::vector<int> pred(size());
    std::vector<bool> visited(size(), false);
    Queue q;

    visited[src] = true;
    q.push_back(src);

    while(!q.empty()){
        v = q.front();
        q.pop_front();
        for(auto & cur : adj[v]){
            if(visited[cur.dest] == false){
                visited[cur.dest] = true;
                pred[cur.dest] = v;
                q.push_back(cur.dest);
            }
        }
    }

    // Ako put ne postoji
    if(visited[dest] != true){
        return;
    }

    path.push_front(dest);
    v = dest;
    while(v != src){
        v = pred[v];
        path.push_front(v);
    }
}

// Funkcija findPath vraca put, ako postoji, od cvora src do cvora dest,
// s tim sto nijedna grana na putu ne sme biti manja od bottleneck.
// Put se smesta u strukturu path.
void Graph::findPath(int src, int dest, int bottleneck, Path& path) const{
    int v;
    std::vector<int> pred(size());
    std::vector<bool> visited(size(), false);
    Queue q;

    path = {};

    visited[src] = true;
    q.push_back(src);

    while(!q.empty()){
        v = q.front();
        q.pop_front();
        for(auto & cur : adj[v]){
            if(visited[cur.dest] == false && cur.weight >= bottleneck){
                visited[cur.dest] = true; 
                pred[cur.dest] = v;
                q.push_back(cur.dest);
            }
        }
    }

    if(visited[dest] != true){
        path = {};
        return;
    }

    path.push_front(dest);
    v = dest;
    while(v != src){
        v = pred[v];
        path.push_front(v);
    }
}

// Funkcija connected_components_dfs se koristi kao pomocna za connected_components
void Graph::connected_components_dfs(int src, std::vector<bool>& visited, int comp_id, std::vector<int>& comp) const{
    visited[src] = true;
    comp[src] = comp_id;

    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false){
            connected_components_dfs(cur.dest, visited, comp_id, comp);
        }
    }
}

// Funkcija connected_components_dfs se koristi kao pomocna za connected_components
// Algoritam ignorise grane manje od bottleneck
void Graph::connected_components_dfs(int src, std::vector<bool>& visited, int bottleneck, int comp_id, std::vector<int>& comp) const{
    visited[src] = true;
    comp[src] = comp_id;

    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false && cur.weight >= bottleneck){
            connected_components_dfs(cur.dest, visited, bottleneck, comp_id, comp);
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

// Funkcija connected_components() pronalazi povezane komponente u neusmerenom grafu
// koristeci DFS pristup. Povezana komponenta je skup cvorova koji su svi dostizni
// medju sobom. Rezultat algoritma se cuva u strukturi comp, koja predstavlja niz
// u kome je id komponente dodeljen indeksu cvora. Funkcija connected_components
// vraca ukupan broj komponenti kao return.
// Algoritam ignorise grane manje od bottleneck
int Graph::connected_components(int bottleneck, std::vector<int>& comp) const{
    int comp_id = 0;
    std::vector<bool> visited(size(), false);

    for(int i = 0; i < size(); ++i){
        if(!visited[i]){
            connected_components_dfs(i, visited, bottleneck, comp_id, comp);
            comp_id++;
        }
    }

    return comp_id;
}

// Sazima graf tako da dva cvora u i v ukoliko su deo iste povezujuce komponente,
// tj. ako je comp[u] == comp[v], sada predstavljaju jedan cvor u novonastalom 
// grafu, ciji je id comp[u].
// Novonastali graf sadrzi grane jedino izmedju povezujucih komponenti.
// Cvorovi novonastalog grafa su zapravo povezujuce komponente.
// Postoji grana izmedju dve komponente novonastalog grafa ako postoji
// cvor u prvoj komponenti i cvor u drugoj komponente i grana izmedju ta
// dva cvora u inicijalnom grafu.
// Tezina grane izmedju dve komponente tj. dva cvora u novonastalom grafu je
// jednaka maksimumu tezina grana izmedju cvorova tih komponenti u inicijalnom grafu.
void Graph::shrink(const std::vector<int>& comp, int comp_size){
    // nova lista susedstava grafa nakon operacije shrink
    std::vector<std::vector<EdgeId*>> new_adj(comp_size);
    // pomocni vektor koji cuva pokazivac na vec dodatu granu u new_adj
    std::vector<EdgeId*> helper(comp_size);

    for(int i = 0; i < size(); ++i){
        for(auto j = adj[i].begin(); j != adj[i].end(); ++j){
            int u = comp[i];
            int v = comp[j->dest];
            // ako su cvorovi deo razlicite komponente dodati granu u new_adj
            if(u != v){
                // provera da li je grana vec dodata
                if(helper[v] == nullptr || helper[v]->src != u){
                    // ako nije dodaj granu
                    EdgeId * edge = new EdgeId(u, v, j->weight);
                    new_adj[u].push_back(edge);
                    // sacuvati pokazivac na granu
                    helper[v] = edge;
                }
                else{
                    // ako jeste postaraj se da je weight maksimalan
                    if(helper[v]->weight < j->weight){
                        helper[v]->weight = j->weight;
                    }
                }
            }
        }
    }

    // kopiraj new_adj u adj, formiraj novu listu suseda grafa G
    adj.clear();
    adj.resize(comp_size);

    for(int i = 0; i < new_adj.size(); ++i){
        for(int j = 0; j < new_adj[i].size(); ++j){
            EdgeId * edge = new_adj[i][j];
            adj[i].push_back({edge->dest, edge->weight});
            free(edge);
        }
    }
}

// Funkcija getMinEdge racuna minimalnu granu na putu path
// Ako put postoji return je true i vrednost najmanje grane se cuva u promenljivoj min_edge
// Ako put ne postoji return je false
bool Graph::getMinEdge(const Path& path, int& min_edge) const{
    int src = path.front();
    int dest;
    int min = INT_MAX;

    if(path.size() < 2){
        return false;
    }

    for(auto i = path.begin() + 1; i != path.end(); ++i){
        bool edge_exists = false;
        dest = *i;
        for(auto edge : adj[src]){
            if(dest == edge.dest){
                edge_exists = true;
                if(min > edge.weight){
                    min = edge.weight;
                }
            }
        }
        if(!edge_exists){
            return false;
        }
        src = dest;
    }

    min_edge = min;
    return true;
}

// Funkcija isConnected proverava da li postoji put u grafu od cvora src do cvora dest.
// Ako put postoji funkcija vraca true. Inace vraca false.
// Kada je src == dest funkcija vraca false.
bool Graph::isConnected(int src, int dest) const{
    int v;
    Queue q;

    if(src == dest){
        // Kada je src == dest funkcija vraca false.
        return false;
    }

    std::vector<bool> visited(size(), false);

    visited[src] = true;
    q.push_back(src);

    while(!q.empty()){
        v = q.front();
        q.pop_front();
        for(auto & cur : adj[v]){
            if(cur.dest == dest){
                return true;
            }

            if(visited[cur.dest] == false){
                visited[cur.dest] = true;
                q.push_back(cur.dest);
            }
        }
    }

    return visited[dest];
}

// Funkcija isConnected proverava da li postoji put u grafu od cvora src do cvora dest,
// pri cemu nijedna grana tog puta ne sme biti manja od bottleneck.
// Ako put postoji funkcija vraca true. Inace vraca false.
// Kada je src == dest funkcija vraca false.
bool Graph::isConnected(int src, int dest, int bottleneck) const{
    int v;
    Queue q;

    if(src == dest){
        // Kada je src == dest funkcija vraca false.
        return false;
    }

    std::vector<bool> visited(size(), false);

    visited[src] = true;
    q.push_back(src);

    while(!q.empty()){
        v = q.front();
        q.pop_front();
        for(auto & cur : adj[v]){
            // ignorisi grane manje od bottleneck
            if(cur.weight >= bottleneck){
                if(cur.dest == dest){
                    return true;
                }

                if(visited[cur.dest] == false){
                    visited[cur.dest] = true;
                    q.push_back(cur.dest);
                }
            }
        }
    }

    return visited[dest];
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
    std::vector<bool> visited(g.size(), false);

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
    std::cout << "[path]:";
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

void swap(Edge& e1, Edge& e2){
    int tmp;

    tmp = e1.dest;
    e1.dest = e2.dest;
    e2.dest = tmp;

    tmp = e1.weight;
    e1.weight = e2.weight;
    e2.weight = tmp;
}

void swap(EdgeId& e1, EdgeId& e2){
    int tmp;

    tmp = e1.src;
    e1.src = e2.src;
    e2.src = tmp;

    tmp = e1.dest;
    e1.dest = e2.dest;
    e2.dest = tmp;

    tmp = e1.weight;
    e1.weight = e2.weight;
    e2.weight = tmp;

    Edge* t = e1.addr;
    e1.addr = e2.addr;
    e2.addr = t;
}

std::string Edge::toString() const{
    return "(" + std:: to_string(dest) + "," + std::to_string(weight) + ")";
}

std::string EdgeId::toString() const{
    return "(" + std:: to_string(src) + "," + std:: to_string(dest) + "," + std::to_string(weight) + ")";
}

std::ostream& operator<< (std::ostream& os, const Graph& g){
    os << g.toString();
    return os;
}

std::string toString(std::vector<EdgeId> vec){
    std::string str = "[edges] size=" + std::to_string(vec.size()) + " edges={";

    for(auto e : vec){
        str += " " + e.toString();
    }

    str += " }";

    return str;
}

std::string toString(std::vector<int> vec){
    bool fst = true;
    std::string str = "[";

    for(auto e : vec){
        if(fst){
            fst = false;
        }
        else{
            str += ",";
        }
        str +=  std::to_string(e);
    }

    str += "]";

    return str;
}
