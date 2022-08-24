#include <stdio.h>
#include <utility>
#include <iostream>
#include <fstream>
#include <climits>
#include <boost/heap/fibonacci_heap.hpp>
#include <math.h>
#include <algorithm>
#include <list>
#include "time.h"
#include "algorithm.hpp"

struct timespec time_s, time_e;

// -----------------------------------------------------------------------------------------------------------------------
// Deklaracije lokalnih funkcija
// -----------------------------------------------------------------------------------------------------------------------

void maxminBruteForceHelper(const Graph& g, int src, int dest, std::vector<bool>& visited, int minEdge, int& maxmin);
int maxminBruteForce(const Graph& g, int src, int dest);
int median_of_medians(std::vector<EdgeId>& edges, int n , int k);
int bottleneckSortedEdges(const Graph& g, int src, int dest, int M, const std::vector<int>& T);
Path widestPathKnowingBottleneck(const Graph& g, int src, int dest, int bottleneck);

// -----------------------------------------------------------------------------------------------------------------------
// Implementacija algoritama za pronalazenje najsireg puta
// -----------------------------------------------------------------------------------------------------------------------

// @thesis najsiriPutGrubaSila
// Funkcija widestPathBruteForce vraca najsiri put od cvora src do cvora dest u grafu g
Path widestPathBruteForce(const Graph& g, int src, int dest){
    Path path;

    if(src == dest){
        return {};
    }

    int maxmin = maxminBruteForce(g, src, dest);
    g.findPath(src, dest, maxmin, path);

    return path;
}

// @thesis najsiriPutDijkstra
// Funkcija widestPathDijkstra vraca najsiri put od cvora src do cvora dest u grafu g
Path widestPathDijkstra(const Graph& g, int src, int dest){
    // distance_from_src[v] predstavlja kapacitet najsireg puta od src do v, sto se jos naziva kapacitet cvora v
    std::vector<int> distance_from_src(g.size(), INT_MIN);
    std::vector<int> pred(g.size());
    std::vector<bool> visited(g.size(), false);
    std::vector<bool> inQueue(g.size(), false);

    std::vector<boost::heap::detail::node_handle<boost::heap::detail::marked_heap_node<HeapNode> *,
     boost::heap::detail::make_fibonacci_heap_base<HeapNode, 
     boost::parameter::aux::flat_like_arg_list<boost::parameter::aux::flat_like_arg_tuple<boost::heap::tag::compare, 
     boost::heap::compare<CompareHeapNode>, boost::mp11::mp_true>>>::type, HeapNode &>> handle(g.size());
    boost::heap::fibonacci_heap<HeapNode, boost::heap::compare<CompareHeapNode>> heap;
    Path path;

    if(src == dest){
        return {};
    }

    visited[src] = true;
    distance_from_src[src] = INT_MAX;
    handle[src] = heap.push({distance_from_src[src], src});
    inQueue[src] = true;

    while(!heap.empty()){
        // skini sa hipa cvor koji ima najveci kapacitet
        int v = heap.top().val;
        heap.pop();
        // posto je to najveci kapacitet u hipu pronadjen je najsiri put do v, pa se v oznacava
        visited[v] = true;
        inQueue[v] = false;
        // ako je v upravo ciljni cvor prekidamo petlju jer je trazeni najsiri put pronadjen
        if(v == dest){
            break;
        }
        // obrada neoznacenih suseda cvora v
        for(auto & cur : g[v]){
            if(visited[cur.dest] == false){
                // modifikuj kapacitet suseda ako je manji od kapaciteta puta: najsiri put do v unija grana (v, sused)
                if(distance_from_src[cur.dest] < distance_from_src[v] && distance_from_src[cur.dest] < cur.weight){
                    distance_from_src[cur.dest] = distance_from_src[v] < cur.weight ? distance_from_src[v] : cur.weight;
                    pred[cur.dest] = v;
                    if(inQueue[cur.dest]){
                        heap.increase(handle[cur.dest], {distance_from_src[cur.dest], cur.dest});
                    }
                    else{
                        handle[cur.dest] = heap.push({distance_from_src[cur.dest], cur.dest});
                        inQueue[cur.dest] = true;
                    }
                }
            }
        }
    }

    if(visited[dest]){
        formPath(src, dest, pred, path);
    }

    return path;
}

// @thesis najsiriPutBinarnaPretraga
// Funkcija widestPathMedianEdgeWeight vraca najsiri put od cvora src do cvora dest u grafu g
Path widestPathMedianEdgeWeight(const Graph& g, int src, int dest){
    if(src == dest || !g.connected(src, dest)){
        return {};
    }

    int minEdge = INT_MAX;
    int maxEdge = INT_MIN;
    int l = 0;
    int r = 0;
    int m = 0;
    int v = 0;
    std::vector<bool> visited(g.size(), false);
    Path path, curPath;
    Queue q;

    // BFS obilaskom grafa pronadji najmanju i najvecu granu u grafu
    visited[src] = 1;

    q.push_back(src);

    while(!q.empty()){
        v = q.front();
        q.pop_front();
        for(auto & cur : g[v]){
            if(minEdge > cur.weight){
                minEdge = cur.weight;
            }
            if(maxEdge < cur.weight){
                maxEdge = cur.weight;
            }
            if(visited[cur.dest] == 0){
                visited[cur.dest] = 1;
                q.push_back(cur.dest);
            }
        }
    }

    // Binarnom pretragom po tezini grane intervala [minEdge, maxEdge], odrediti najsiri put path
    l = minEdge;
    r = maxEdge;
    while(l <= r){
        m = l + r/2 - l/2;
        g.findPath(src, dest, m, curPath);
        if(curPath.empty()){
            r = m - 1;
        }
        else{
            l = m + 1;
            path = curPath;
        }
    }

    return path;
}

// @thesis najsiriPutSabijanjeKomponenti
// Funkcija widestPathInUndirectedGraph vraca najsiri put od cvora src do cvora dest u grafu g
// Zbog specificne implementacije korektnost se garantuje samo za neusmerene grafove
Path widestPathInUndirectedGraph(const Graph& g, int src, int dest){
    if(src == dest || !g.connected(src, dest)){
        return {};
    }

    int bottleneck;
    int _src = src;
    int _dest = dest;
    std::vector<EdgeId> edges;
    std::vector<int> comp(g.size());
    Graph gc(g);

    // Odrediti grane grafa
    gc.getEdgeIds(edges);

    while(gc.size() > 1 && !edges.empty()){
        int M = median_of_medians(edges, edges.size(), edges.size()/2);
        if(gc.connected(_src, _dest, M)){
            // ako put (u podgrafu) postoji zapamti M kao trenutni kapacitet najsireg puta
            bottleneck = M;
            // izbrisi grane tezine manje ili jednake M iz grafa
            gc.deleteEdges(M + 1);
        }
        else {
            // ako put (u podrafu) ne postoji pronadji komponente povezanosti (u podgrafu)
            int comp_num = gc.connected_components(M, comp);
            // id cvora postaje id komponente povezanosti u novom grafu
            _src = comp[_src];
            _dest = comp[_dest];
            // sazmi graf
            gc.shrink(comp, comp_num);
        }
        // odredi grane izmenjenog grafa
        edges.clear();
        gc.getEdgeIds(edges);
    }
    // Na osnovu bottleneck pronaci put u grafu
    Path path;
    g.findPath(src, dest, bottleneck, path);
    return path;
}

// @thesis najsiriPutSortiraneGrane
// Funkcija widestPathInUndirectedGraph vraca najsiri put od cvora src do cvora dest u grafu g
// Funkcija menja order od grana pa Graph nije const
Path widestPathEdgesOrdering(Graph& g, int src, int dest){
    if(src == dest || !g.connected(src, dest)){
        return {};
    }

    std::vector<EdgeId> edges;

    // Sacuvati trenutne grane grafa u edges
    // Odrediti mininalnu i maksimalnu granu grafa
    for(int i = 0; i < g.size(); ++i){
        for(auto & edge : g[i]){
            edges.push_back({i, edge.dest, edge.weight, &edge});
        }
    }

    int iterationCount = 0;
    int L = INT_MIN;
    int R = INT_MAX;
    int k = log2(edges.size());
    while(edges.size() > 0 && iterationCount < log2(k)){
        int M = median_of_medians(edges, edges.size(), edges.size()/2);
        if(g.connected(src, dest, M + 1)){
            edges.erase(std::remove_if(edges.begin(), edges.end(), [M](auto edge){return edge.weight <= M;}), edges.end());
            L = M;
        }
        else{
            edges.erase(std::remove_if(edges.begin(), edges.end(), [M](auto edge){return edge.weight > M;}), edges.end());
            R = M;
        }
        iterationCount++;
    }

    std::vector<int> T(edges.size()+2);
    T[0] = INT_MIN;
    T[edges.size() + 1] = INT_MAX;

    std::sort(edges.begin(), edges.end(), [](EdgeId e1, EdgeId e2){
        return e1.weight < e2.weight;
    });

    for(int i = 0; i < g.size(); ++i){
        for(auto & edge : g[i]){
            if(edge.weight <= L){
                edge.order = 0;
            }
            if(edge.weight > R){
                edge.order = edges.size() + 1;
            }
        }
    }
    for(int i = 0; i < edges.size(); ++i){
        edges[i].addr->order = i + 1;
        T[i + 1] = edges[i].weight;
    }

    int bottleneck = bottleneckSortedEdges(g, src, dest, edges.size() + 2, T);

    return widestPathKnowingBottleneck(g, src, dest, bottleneck);
}

// -----------------------------------------------------------------------------------------------------------------------
// Definicije lokalnih funkcija
// -----------------------------------------------------------------------------------------------------------------------

// @thesis maxminGrubaSilaPomocna
// Funkcija maxminBruteForceHelper je pomocna funkcija koja rekurzivno prolazi kroz sve puteve koji polaze od cvora src,
// cuvajuci kapacitet trenutnog puta u rekurziji. Ako se put zavrsava u ciljnom cvoru d, tada funkcija modifikuje
// trenutni maxmin, i na kraju izvrsavanja maxmin predstavlja kapacitet najsireg puta od cvora src do dvora dest u grafu G
void maxminBruteForceHelper(const Graph& g, int src, int dest, std::vector<bool>& visited, int minEdge, int& maxmin){
    visited[src] = true;

    if(src == dest){
        if(maxmin < minEdge){
            maxmin = minEdge;
        }
    }
    else{
        for(auto & cur : g[src]){
            if(visited[cur.dest] == false){
                maxminBruteForceHelper(g, cur.dest, dest, visited, 
                                        minEdge < cur.weight ? minEdge : cur.weight,
                                        maxmin);
            }
        }
    }

    visited[src] = false;
}

// @thesis maxminGrubaSila
// Funkcija maxminBruteForce vraca kapacitet najsireg puta od cvora src do cvora dest u grafu g
int maxminBruteForce(const Graph& g, int src, int dest){
    std::vector<bool> visited(g.size(), false);
    int maxmin = INT_MIN;

    maxminBruteForceHelper(g, src, dest, visited, INT_MAX, maxmin);

    return maxmin;
}

// Funkcija widestPathKnowingBottleneck vraca put, ako postoji, od cvora src do cvora dest u grafu g, 
// s tim sto svaka grana tog puta mora biti najmanje bottleneck tezine
Path widestPathKnowingBottleneck(const Graph& g, int src, int dest, int bottleneck){
    Path path = {};

    if(src == dest){
        return {};
    }

    g.findPath(src, dest, bottleneck, path);

    return path;
}

// Funkcija median_of_medians vraca k-ti po velicini element vektora edges.
// Posmatra se samo deo vektora od indeksa 0 do indeksa n.
int median_of_medians(std::vector<EdgeId>& edges, int n , int k){
    int i = 0;
    int j = 0;
    int pivot = 0;
    int low = 0;
    int high = 0;
    CompareEdgeIds compare;

    if(n <= 5){
        std::sort(edges.begin(), edges.begin() + n, compare);
        pivot = edges[n/2].weight;
    }
    else{
        while(i < n && j + 4 < n){
            std::sort(edges.begin() + j, edges.begin() + j + 5, compare);
            swap(edges[i], edges[j + 2]);
            ++i;
            j+=5;
        }

        while(i < n && j < n){
            swap(edges[i], edges[j]);
            ++i;
            ++j;
        }

        pivot = median_of_medians(edges, i, i/2);
    }
    
    for(i = 0; i < n; ++i){
        if(edges[i].weight < pivot){
            ++low;
        }
        else if(edges[i].weight > pivot){
            ++high;
        }
    }

    if(k < low){
        i = 0;
        j = n - 1;
        while(i < j){
            while(i < n && edges[i].weight < pivot){
                ++i;
            }
            while(j >=0 && edges[j].weight >= pivot){
                --j;
            }
            if(i < j){
                swap(edges[i], edges[j]);
                ++i;
                --j;
            }
        }
        return median_of_medians(edges, low, k);
    }
    else if(k >= n - high){
        i = 0;
        j = n - 1;
        while(i < j){
            while(i < n && edges[i].weight > pivot){
                ++i;
            }
            while(j >=0 && edges[j].weight <= pivot){
                --j;
            }
            if(i < j){
                swap(edges[i], edges[j]);
                ++i;
                --j;
            }
        }
        return median_of_medians(edges, high, k - (n - high));
    }
    else{
        return pivot;
    }
}

// Funkcija bottleneckSortedEdges vraca bottleneck tj. najmanju granu najsireg puta
// od cvora src do cvora dest u grafu g. Podrazumeva se da put postoji.
// Ovaj algoritam zahteva da postoji uredjenje grana grafa po tezini tj. da su prethodno sortirane
// Argument M predstavlja broj razlicitih vrednosti Edge.order
// Tabela T preslikava redosled grane u tezinu grane, T[order] = weight
int bottleneckSortedEdges(const Graph& g, int src, int dest, int M, const std::vector<int>& T){
    // B, niz skupova u koje se dodaju/oduzimaju(push/pop) cvorovi grafa
    std::list<int> B[M];
    // Flag NO_INDEX oznacava da cvor nije u B
    const int NO_INDEX = -1;
    // b[v] predstavlja indeks cvora v u B tj. b]v] oznacava da se cvor v nalazi u skupu B[b[v]]
    std::vector<int> b(g.size(), NO_INDEX);
    // f[v] je flag koji oznacava da li je cvor v izbacen(pop) iz B
    std::vector<bool> f(g.size(), false);
    // addr cuva adresu elementa liste u B kako bi elementu liste pristupili u O(1)
    std::list<int>::iterator addr[g.size()];

    // Oznaci src
    f[src] = true;

    // Iteriraj kroz susede od src
    for(auto i = g[src].begin(); i != g[src].end(); ++i){
        // grana (src, i->dest)
        B[i->order].push_front(i->dest);
        addr[i->dest] = B[i->order].begin();
        b[i->dest] = i->order;
    }

    int U = M - 1;
    while(U >= 0){
        while(!B[U].empty()){
            // pop bilo koji v iz skupa B[U]
            int v = B[U].front();
            B[U].pop_front();
            f[v] = true;
            if(v == dest){
                // kraj algoritma
                return T[b[dest]];
            }
            else{
                // Iteriraj kroz susede od v
                for(auto i = g[v].begin(); i != g[v].end(); ++i){
                    // grana (v, w)
                    int w = i->dest;
                    if(f[w] == false){
                        int k = std::min(b[v], i->order);
                        if(k > b[w]){
                            if(b[w] != NO_INDEX){
                                B[b[w]].erase(addr[w]);
                            }
                            B[k].push_front(w);
                            addr[w] = B[k].begin();
                            b[w] = k;
                        }
                    }
                }
            }
        }
        U--;
    }
    // kada d nije dostizan
    return INT_MIN;
}
