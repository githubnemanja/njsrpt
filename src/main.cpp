#include "graph.hpp"
#include "tests.hpp"
#include <fstream>
#include <tuple>
#include <climits>

// Ako je DEBUG flag jednak 0 stampa se na standardni izlaz, ako je jednak 1 stampa se u fajl out.txt
#define DEBUG 0

// Prilikom izvrsavanja testova potrebno je postaviti parametre:
// NUM_OF_TESTS: broj ulaznih instanci, INPUT_SIZE: broj cvorova ulaza, NUM_OF_EDGES: broj grana ulaza
const int NUM_OF_TESTS = 100;
const int INPUT_SIZE  = 10;
const int NUM_OF_EDGES  = 100;

int main(){
    std::vector<std::pair<std::string, std::tuple<double, double, double>>> avgs{
        {"widestPathBruteForce", {INT_MAX, 0, INT_MIN}},
        {"widestPathDijkstra", {INT_MAX, 0, INT_MIN}},
        {"widestPathMedianEdgeWeight", {INT_MAX, 0, INT_MIN}},
        {"widestPathInUndirectedGraph", {INT_MAX, 0, INT_MIN}},
        {"widestPathEdgesOrdering", {INT_MAX, 0, INT_MIN}},
    };

    #if DEBUG == 1
    std::ofstream out("out.txt");
    std::streambuf *coutbuf = std::cout.rdbuf();
    std::cout.rdbuf(out.rdbuf());
    #endif

    int success = 0;
    for(int i = 0; i < NUM_OF_TESTS; ++i){
        Graph g(INPUT_SIZE);
        generateEdges(g, NUM_OF_EDGES, false);
        std::vector<double> times;
        success += runTests(g, 0, 1, times, false) ? 1 : 0;
        updateAvgs(avgs, times, NUM_OF_TESTS);
    }

    printAvgs(avgs);

    printResult(success, NUM_OF_TESTS);

    #if DEBUG == 1
    std::cout.rdbuf(coutbuf);
    #endif
    return 0;
}
