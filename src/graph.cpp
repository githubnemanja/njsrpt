#include <stdio.h>
#include <iostream>
#include <climits>
#include <iterator>
#include "graph.hpp"

// -----------------------------------------------------------------------------------------------------------------------
// Definicije osnovnih funkcionalnosti klase Graph
// -----------------------------------------------------------------------------------------------------------------------

Graph::Graph(int size){
    adj.resize(size);
}

Graph::Graph(const Graph& g){
    adj = g.adj;
    directed = g.isDirected();
}

int Graph::size() const{
    return adj.size();
}

bool Graph::isDirected() const{
    return directed;
}

void Graph::setDirected(bool directed){
    this->directed = directed;
}

std::vector<Edge>& Graph::operator[] (int index){
    return adj[index];
}

const std::vector<Edge>& Graph::operator[] (int index) const{
    return adj[index];
}

// Dodaj granu (src, dest) tezine weight u graf
void Graph::addEdge(int src, int dest, int weight){
    if(src < size()){
        for(auto & cur : adj[src]){
            if(cur.dest == dest){
                //grana vec postoji
                return;
            }
        }
        adj[src].push_back({dest, weight});
    }
}

// Obrisati sve grane u grafu cija je tezina manja od bottleneck i vratiti broj obrisanih grana
int Graph::deleteEdges(int bottleneck){
    int num_deleted = 0;

    for(int i = 0; i < size(); ++i){
        int new_size = adj[i].size();
        int j = 0;
        while(j < new_size){
            if(adj[i][j].weight < bottleneck){
                // trenutna grana je za brisanje
                // swap-uj trenutnu i poslednju granu u adj[i]
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

// Funkcija getEdgeIds vraca sve grane grafa. Rezultat cuva u strukturi edges.
void Graph::getEdgeIds(std::vector<EdgeId>& edges) const{
    for(int i = 0; i < adj.size(); ++i){
        for(auto edge : adj[i]){
            edges.push_back({i, edge.dest, edge.weight});
        }
    }
}

// -----------------------------------------------------------------------------------------------------------------------
// Definicije Osnovnih grafovskih algoritama
// -----------------------------------------------------------------------------------------------------------------------

// @thesis formirajPutOdNizaPrethodnika
// Formira put path od src do dest koristeci niz prethodnika pred
// Pretpostavka je da je pred korektno popunjen
void formPath(int src, int dest, const std::vector<int>& pred, Path& path){
    path = {};
    if(src == dest){
        return;
    }
    while(dest != src){
        path.push_front(dest);
        dest = pred[dest];
    }
    path.push_front(src);
}

// @thesis postojiPutDFS
// Funkcija connectedDFS se koristi kao pomocna za connected i predstavlja obilazak grafa u dubinu
bool Graph::connectedDFS(int src, int dest, std::vector<bool>& visited) const{
    visited[src] = true;

    if(src == dest){
        return true;
    }

    // obrada neoznacenih suseda cvora src
    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false){
            if(connectedDFS(cur.dest, dest, visited)){
                return true;
            }
        }
    }

    return false;
}

// @thesis postojiPut
// Funkcija connected proverava da li postoji put u grafu od cvora src do cvora dest.
// Ako put postoji funkcija vraca true. Inace vraca false.
// Kada je src == dest funkcija vraca false.
bool Graph::connected(int src, int dest) const{
    if(src == dest){
        return false;
    }
    std::vector<bool> visited(size(), false);
    return connectedDFS(src, dest, visited);
}

// Funkcija connectedDFS se koristi kao pomocna za connected i predstavlja obilazak grafa u dubinu
// Algoritam ignorise grane manje od bottleneck
bool Graph::connectedDFS(int src, int dest, std::vector<bool>& visited, int bottleneck) const{
    visited[src] = true;

    if(src == dest){
        return true;
    }

    // obrada neoznacenih suseda cvora src
    // ignorisi grane manje od bottleneck
    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false && cur.weight >= bottleneck){
            if(connectedDFS(cur.dest, dest, visited, bottleneck)){
                return true;
            }
        }
    }

    return false;
}

// Funkcija connected proverava da li postoji put u grafu od cvora src do cvora dest.
// Ako put postoji funkcija vraca true. Inace vraca false.
// Kada je src == dest funkcija vraca false.
// Prilikom trayenja puta algoritam ignorise grane manje od bottleneck.
bool Graph::connected(int src, int dest, int bottleneck) const{
    if(src == dest){
        return false;
    }
    std::vector<bool> visited(size(), false);
    return connectedDFS(src, dest, visited, bottleneck);
}

// @thesis nadjiPutDFS
// Funkcija findPathDFS se koristi kao pomocna za findPath i predstavlja obilazak grafa u dubinu
void Graph::findPathDFS(int src, std::vector<bool>& visited, std::vector<int>& pred) const{
    visited[src] = true;

    // obrada neoznacenih suseda cvora src
    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false){
            pred[cur.dest] = src;
            findPathDFS(cur.dest, visited, pred);
        }
    }
}

// @thesis nadjiPut
// Funkcija findPath vraca put u grafu od cvora src do cvora dest.
// Put se smesta u strukturu path.
void Graph::findPath(int src, int dest, Path& path) const{
    path = {};
    if(src == dest){
        return;
    }
    std::vector<bool> visited(size(), false);
    std::vector<int> pred(size());
    findPathDFS(src, visited, pred);
    if(visited[dest]){
        formPath(src, dest, pred, path);
    }
}

// @thesis nadjiPutUPodgrafuDFS
// Funkcija findPathDFS se koristi kao pomocna za findPath i predstavlja obilazak grafa u dubinu
// Algoritam ignorise grane manje od bottleneck
void Graph::findPathDFS(int src, std::vector<bool>& visited, int bottleneck, std::vector<int>& pred) const{
    visited[src] = true;

    // obrada neoznacenih suseda cvora src
    // ignorisi grane manje od bottleneck
    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false && cur.weight >= bottleneck){
            pred[cur.dest] = src;
            findPathDFS(cur.dest, visited, bottleneck, pred);
        }
    }
}

// @thesis nadjiPutUPodgrafu
// Funkcija findPath vraca put u grafu od cvora src do cvora dest.
// Put se smesta u strukturu path.
// Prilikom trazenja puta algoritam ignorise grane manje od bottleneck.
void Graph::findPath(int src, int dest, int bottleneck, Path& path) const{
    path = {};
    if(src == dest){
        return;
    }
    std::vector<bool> visited(size(), false);
    std::vector<int> pred(size());
    findPathDFS(src, visited, bottleneck, pred);
    if(visited[dest]){
        formPath(src, dest, pred, path);
    }
}


// @thesis komponentePovezanostiDFS
// Funkcija connected_components_dfs se koristi kao pomocna za connected_components i predstavlja obilazak grafa u dubinu
void Graph::connected_components_dfs(int src, std::vector<bool>& visited, int comp_id, std::vector<int>& comp) const{
    visited[src] = true;
    comp[src] = comp_id;

    // obrada neoznacenih suseda cvora src
    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false){
            connected_components_dfs(cur.dest, visited, comp_id, comp);
        }
    }
}

// Funkcija connected_components_dfs se koristi kao pomocna za connected_components i predstavlja obilazak grafa u dubinu
// Algoritam ignorise grane manje od bottleneck
void Graph::connected_components_dfs(int src, std::vector<bool>& visited, int bottleneck, int comp_id, std::vector<int>& comp) const{
    visited[src] = true;
    comp[src] = comp_id;

    // obrada neoznacenih suseda cvora src
    // ignorisi grane manje od bottleneck
    for(auto & cur : adj[src]){
        if(visited[cur.dest] == false && cur.weight >= bottleneck){
            connected_components_dfs(cur.dest, visited, bottleneck, comp_id, comp);
        }
    }
}

// @thesis komponentePovezanosti
// Funkcija connected_components pronalazi povezane komponente u neusmerenom grafu
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

// Funkcija connected_components pronalazi povezane komponente u neusmerenom grafu
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

// Funkcija shrink sazima graf tako da dva cvora u i v ukoliko su deo iste povezujuce komponente,
// tj. ako je comp[u] == comp[v], sada predstavljaju jedan cvor u novonastalom grafu, ciji je id comp[u].
// Novonastali graf sadrzi grane jedino izmedju povezujucih komponenti.
// Cvorovi novonastalog grafa su zapravo povezujuce komponente.
// Postoji grana izmedju dve komponente novonastalog grafa ako postoji
// cvor u prvoj komponenti i cvor u drugoj komponente i grana izmedju ta dva cvora u inicijalnom grafu.
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

// -----------------------------------------------------------------------------------------------------------------------
// Definicije ostalih operacija sa granama
// -----------------------------------------------------------------------------------------------------------------------

// Funkcija getMinEdge racuna minimalnu granu na putu path
// Ako put postoji return je true i vrednost najmanje grane se cuva u promenljivoj min_edge
// Ako put ne postoji return je false
// Ovaj funkcija se koristi za proveru ispravnosti algoritma
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
        // obrada susednih grana cvora src
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

// -----------------------------------------------------------------------------------------------------------------------
// Definicije debug funkcija
// -----------------------------------------------------------------------------------------------------------------------

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

std::string Edge::toString() const{
    return "(" + std:: to_string(dest) + "," + std::to_string(weight) + ")";
}

std::string EdgeId::toString() const{
    return "(" + std:: to_string(src) + "," + std:: to_string(dest) + "," + std::to_string(weight) + ")";
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

std::ostream& operator<< (std::ostream& os, const Graph& g){
    os << g.toString();
    return os;
}
