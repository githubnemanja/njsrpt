#include <stdio.h>
#include <utility>
#include <iostream>
#include <climits>
#include <boost/heap/fibonacci_heap.hpp>
#include "algorithm.hpp"

// Funkcija widestPathBruteForce vraca najsiri put, ako postoji, od cvora src do cvora dest u grafu g
Path widestPathBruteForce(const Graph& g, int src, int dest){
    std::vector<std::pair<Path, int>> paths;
    Path p;
    Path maxPath;
    int maxEdge = INT_MIN;
    int curEdge = 0;

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
    int v;
    std::vector<int> pred(g.size());
    std::vector<bool> visited(g.size(), false);
    Queue q;
    Path path;

    visited[src] = true;
    q.push_back(src);

    while(!q.empty()){
        v = q.front();
        q.pop_front();
        for(auto & cur : g[v]){
            if(visited[cur.dest] == false && cur.weight >= bottleneck){
                visited[cur.dest] = true; 
                pred[cur.dest] = v;
                q.push_back(cur.dest);
            }
        }
    }

    if(visited[dest] != true){
        return {};
    }

    path.push_front(dest);
    v = dest;
    while(v != src){
        v = pred[v];
        path.push_front(v);
    }

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

    //find minEdge, maxEdge

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

    l = minEdge;
    r = maxEdge;
    while(l <= r){
        m = l + (r - l)/2;
        curPath = widestPathKnowingBottleneck(g, src, dest, m);
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
void sort5(int* a, int start, int num){
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
int median_of_medians(int* a, int n , int k){
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

// Funkcija widestPathInUndirectedGraph vraca najsiri put, ako postoji, od cvora src do cvora dest u grafu g
// Zbog specificne implementacije korektnost se garantuje samo za neusmerene grafove
Path widestPathInUndirectedGraph(const Graph& g, int src, int dest){
    return {};
}