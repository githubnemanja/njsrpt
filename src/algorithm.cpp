#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "algorithm.hpp"
#include <boost/heap/fibonacci_heap.hpp>
#include <utility>
#include <iostream>

struct node
{
    int key;
    int val;

    node(int key, int val)
      : key(key), val(val)
    { }
};

struct compare_node
{
    bool operator()(const node& n1, const node& n2) const
    {
        return n1.key < n2.key;
    }
};

void DFS_fp(const Graph& g, int src, int dest, bool * visited, Path p, std::deque<std::pair<Path, int>>& paths, int minEdge){
    if(visited == NULL){
        return;
    }

    visited[src] = 1;

    if(src == dest){
        paths.push_back(std::make_pair(p, minEdge));
    }
    else{
        for(auto & cur : g[src]){
            if(visited[cur.dest] == 0){
                p.push_back(cur.dest);
                DFS_fp(g, cur.dest, dest, visited, p, paths, 
                        minEdge < cur.weight ? minEdge : cur.weight);
                p.pop_back();
            }
        }
    }

    visited[src] = 0;
}

void findPaths(const Graph& g, int src, int dest, std::deque<std::pair<Path, int>>& paths){
    Path p;
    int i = 0;
    bool visited[g.size()] = {false};

    p.push_back(src);

    DFS_fp(g, src, dest, visited, p, paths, INT_MAX);
}

Path widestPathBruteForce(const Graph& g, int src, int dest){
    std::deque<std::pair<Path, int>> paths;
    Path p;
    Path maxPath;
    int maxEdge = INT_MIN;
    int curEdge = 0;

    findPaths(g, src, dest, paths);

    while(!paths.empty()){
        auto pair = paths.back();
        paths.pop_back();
        p = pair.first;
        curEdge = pair.second;
        if(curEdge > maxEdge){
            maxEdge = curEdge;
            maxPath = p;
        }
    }

    return maxPath;
}

Path widestPathDijkstra(const Graph& g, int src, int dest){
    int v = 0;
    int distance_from_src[g.size()] = {INT_MIN};
    int pred[g.size()];
    bool visited[g.size()] = {false};
    boost::heap::fibonacci_heap<node, boost::heap::compare<compare_node>> heap;
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

Path widestPathKnowingBottleneck(const Graph& g, int src, int dest, int bottleneck){
    int v = 0;
    int pred[g.size()];
    bool visited[g.size()] = {false};
    Queue  q;
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

// @funkcija sort5 : sortira deo niza od start do start + num, interval [start, start + num)
//                   koristi se za sortiranje skupa podataka koji zauzima malu memoriju
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

// @funkcija median_of_medians : vraca k-ti po velicini element niza, niz je duzine n
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