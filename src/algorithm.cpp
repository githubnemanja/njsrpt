#include <stdio.h>
#include <utility>
#include <iostream>
#include <fstream>
#include <climits>
#include <boost/heap/fibonacci_heap.hpp>
#include <math.h>
#include <algorithm>
#include <list>
#include "algorithm.hpp"

// Funkcija widestPathBruteForce vraca najsiri put, ako postoji, od cvora src do cvora dest u grafu g
Path widestPathBruteForce(const Graph& g, int src, int dest){
    std::vector<std::pair<Path, int>> paths;
    Path p;
    Path maxPath;
    int maxEdge = INT_MIN;
    int curEdge = 0;

    if(src == dest){
        return {};
    }

    g.findPaths(src, dest, paths);

    for(auto & pair : paths){
        p = pair.first;
        curEdge = pair.second;
        if(curEdge > maxEdge){
            maxEdge = curEdge;
            maxPath = p;
        }
    }

    return maxPath;
}

// Funkcija widestPathDijkstra vraca najsiri put, ako postoji, od cvora src do cvora dest u grafu g
Path widestPathDijkstra(const Graph& g, int src, int dest){
    int v = 0;
    int distance_from_src[g.size()] = {INT_MIN};
    int pred[g.size()];
    bool visited[g.size()] = {false};
    boost::heap::fibonacci_heap<HeapNode, boost::heap::compare<CompareHeapNode>> heap;
    Path path;

    if(src == dest){
        return {};
    }

    visited[src] = true;
    distance_from_src[src] = INT_MAX;
    heap.push({distance_from_src[src], src});

    while(!heap.empty()){
        v = heap.top().val;
        heap.pop();
        visited[v] = true;
        for(auto & cur : g[v]){
            if(visited[cur.dest] == false){
                if(distance_from_src[cur.dest] < distance_from_src[v] && distance_from_src[cur.dest] < cur.weight){
                    distance_from_src[cur.dest] = distance_from_src[v] < cur.weight ? distance_from_src[v] : cur.weight;
                    pred[cur.dest] = v;
                }
                heap.push({distance_from_src[cur.dest], cur.dest});
            }
        }
    }

    if(distance_from_src[dest] != INT_MIN){
        path.push_front(dest);
        v = dest;
        while(v != src){
            v = pred[v];
            path.push_front(v);
        }
    }

    return path;
}

// Funkcija widestPathKnowingBottleneck vraca put, ako postoji, od cvora src do cvora dest
// u grafu g, s tim sto svaka grana tog puta mora biti najmanje bottleneck tezine
Path widestPathKnowingBottleneck(const Graph& g, int src, int dest, int bottleneck){
    Path path = {};

    if(src == dest){
        return {};
    }

    g.findPath(src, dest, bottleneck, path);

    return path;
}

// Funkcija widestPathMedianEdgeWeight vraca najsiri put, ako postoji, od cvora src do cvora dest u grafu g
Path widestPathMedianEdgeWeight(const Graph& g, int src, int dest){
    int minEdge = INT_MAX;
    int maxEdge = INT_MIN;
    int l = 0;
    int r = 0;
    int m = 0;
    int v = 0;
    bool visited[g.size()] = {false};
    Path path, curPath;
    Queue q;

    if(src == dest){
        return {};
    }

    //pronadji najmanju i najvecu granu u grafu
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

    // Binarna pretraga po tezini grane
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

// Funkcija sort5 sortira deo niza od start do start + num, interval [start, start + num)
// Koristi se za sortiranje malog skupa podataka
void sort5(std::vector<int> a, int start, int num){
    int i, j, tmp;

    for(i = 0; i < num; ++i){
        for(j = i + 1; j < num; ++j){
            if(a[i + start] > a[j + start]){
                tmp = a[i + start];
                a[i + start] = a[j + start];
                a[j + start] = tmp;
            }
        }
    }
}

// Funkcija median_of_medians vraca k-ti po velicini element niza a, niz je duzine n
int median_of_medians(std::vector<int> a, int n , int k){
    int i = 0;
    int j = 0;
    int tmp = 0;
    int pivot = 0;
    int low = 0;
    int high = 0;

    if(n <= 5){
        sort5(a, 0, n);
        pivot = a[n/2];
    }
    else{
        while(i < n && j + 4 < n){
            sort5(a, j, 5);
            tmp = a[i];
            a[i] = a[j + 2];
            a[j + 2] = tmp;
            ++i;
            j+=5;
        }

        while(i < n && j < n){
            tmp = a[i];
            a[i] = a[j];
            a[j] = tmp;
            ++i;
            ++j;
        }

        pivot = median_of_medians(a, i, i/2);
    }
    
    for(i = 0; i < n; ++i){
        if(a[i] < pivot){
            ++low;
        }
        else if(a[i] > pivot){
            ++high;
        }
    }

    if(low <= k && k < n - high){
        return pivot;
    }
    if(low < k){
        i = 0;
        j = n - 1;
        while(i < j){
            while(i < n && a[i] >= pivot){
                ++i;
            }
            while(j >=0 && a[j] < pivot){
                --j;
            }
            if(i < j){
                tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
            }
        }
        return median_of_medians(a, n - low, k - low - 1);
    }
    else{
        i = 0;
        j = n - 1;
        while(i < j){
            while(i < n && a[i] < pivot){
                ++i;
            }
            while(j >=0 && a[j] >= pivot){
                --j;
            }
            if(i < j){
                tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
            }
        }
        return median_of_medians(a, low, k);
    }
}

// Funkcija median_of_medians vraca k-ti po velicini element vektora a.
// Posmatra se samo deo vektora od indeksa 0 do indeksa n.
int median_of_medians(std::vector<EdgeId> edges, int n , int k){
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

    if(low <= k && k < n - high){
        return pivot;
    }
    if(low < k){
        i = 0;
        j = n - 1;
        while(i < j){
            while(i < n && edges[i].weight >= pivot){
                ++i;
            }
            while(j >=0 && edges[j].weight < pivot){
                --j;
            }
            if(i < j){
                swap(edges[i], edges[j]);
            }
        }
        return median_of_medians(edges, n - low, k - low - 1);
    }
    else{
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
            }
        }
        return median_of_medians(edges, low, k);
    }
}

// Funkcija widestPathInUndirectedGraph vraca najsiri put, ako postoji, od cvora src do cvora dest u grafu g
// Zbog specificne implementacije korektnost se garantuje samo za neusmerene grafove
Path widestPathInUndirectedGraph(const Graph& g, int src, int dest){
    int bottleneck;
    int _src = src;
    int _dest = dest;
    std::vector<EdgeId> edges;
    std::vector<int> comp(g.size());
    Graph gc(g);

    if(src == dest){
        return {};
    }

    gc.getEdgeIds(edges);

    while(gc.size() > 1 && !edges.empty()){
        int M = median_of_medians(edges, edges.size(), edges.size()/2);
        // ignorisi grane manje od M
        if(gc.isConnected(_src, _dest, M)){
            // postoji put
            bottleneck = M;
            // izbrisi grane tezine manje od M iz grafa
            int num_deleted = gc.deleteEdges(M + 1);
            if(num_deleted == 0){
                std::cout << "[widestPathInUndirectedGraph][ERROR] Did not delete any edges!" << std::endl;
            }
        }
        else {
            // ne postoji put
            // pronadji povezujuce komponente
            int comp_num = gc.connected_components(M, comp);

            if(comp_num < 2){
                std::cout << "[widestPathInUndirectedGraph][ERROR] comp_num=" << comp_num << std::endl;
            }
            // id cvora postaje id povezujuce komponente u novom grafu
            _src = comp[_src];
            _dest = comp[_dest];
            // ponovo koristi grane manje od M tj. sve grane
            // sazmi graf
            gc.shrink(comp, comp_num);
        }
        // odredi grane izmenjenog grafa
        edges.clear();
        gc.getEdgeIds(edges);
    }

    Path path;
    g.findPath(src, dest, bottleneck, path);
    return path;
}

// Funkcija bottlemeckSortedEdgeWeights vraca bottleneck tj. najmanju granu najsireg puta
// od cvora src do cvora dest u grafu g. Podrazumeva se da put postoji.
// Ovaj algoritam zahteva da postoji uredjenje grana grafa po tezini tj. da su predhodno sortirane
// Argument M predstavlja broj razlicitih vrednosti Edge.order
// Tabela T mapira redosled grane u tezinu grane, T[order] = weight
int bottleneckSortedEdges(const Graph& g, int src, int dest, int M, const std::vector<int>& T){
    // Buket, niz skupova u koje se dodaju/oduzimaju(push/pop) cvorovi grafa
    std::list<int> B[M];
    // b[v] predstavlja indeks cvora v u Buketu
    int b[g.size()] = {0};
    // f[v] je flag koji oznacava da li je cvor v izbacen(pop) iz Buketa
    bool f[g.size()] = {false};
    // addr cuva adresu elementa liste u Buketu kao bi element liste mogao da se obrse u O(1)
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
                            B[b[w]].erase(addr[w]);
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
    // ovo ne bi trebalo nikad da vrati
    std::cout << "[ERROR] Wrong return!" << std::endl;
    return INT_MIN;
}

// Funkcija widestPathInUndirectedGraph vraca najsiri put, ako postoji, od cvora src do cvora dest u grafu g
Path widestPathEdgesOrdering(Graph g, int src, int dest){
    int minEdge = INT_MAX;
    int maxEdge = INT_MIN;
    std::vector<EdgeId> edges;

    if(src == dest){
        return {};
    }

    // Odrediti grane grafa
    // Odrediti mininalnu i maksimalnu granu grafa
    for(int i = 0; i < g.size(); ++i){
        for(auto & edge : g[i]){
            edges.push_back({i, edge.dest, edge.weight, &edge});
            if(minEdge > edge.weight){
                minEdge = edge.weight;
            }
            if(maxEdge < edge.weight){
                maxEdge = edge.weight;
            }
        }
    }

    std::vector<EdgeId> E = edges;
    int iterationCount = 0;
    int L = minEdge;
    int U = maxEdge;
    int sm = log2(edges.size());
    while(iterationCount < sm){
        int M = median_of_medians(edges, edges.size(), edges.size()/2);
        if(g.isConnected(src, dest, M + 1)){
            std::vector<EdgeId> new_edges;
            for(auto eid : edges){
                if(eid.weight > M){
                    new_edges.push_back(eid);
                }
            }
            edges = new_edges;
            L = M;
        }
        else{
            U = M;
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
            if(edge.weight > U){
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