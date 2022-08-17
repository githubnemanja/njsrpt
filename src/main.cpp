#include "graph.hpp"
#include "tests.hpp"

// Ako je DEBUG flag jednak 0 stampa se na standardni izlaz, ako je jednak 1 stampa se u fajl out.txt
#define DEBUG 0

const int NUM_OF_TESTS = 100;
const int INPUT_SIZE  = 10000;

int main(){
    std::vector<std::pair<std::string, double>> avgs{
        {"widestPathBruteForce", 0},
        {"widestPathDijkstra", 0},
        {"widestPathMedianEdgeWeight", 0},
        {"widestPathInUndirectedGraph", 0},
        {"widestPathEdgesOrdering", 0},
    };

    #if DEBUG == 1
    std::ofstream out("out.txt");
    std::streambuf *coutbuf = std::cout.rdbuf();
    std::cout.rdbuf(out.rdbuf());
    #endif

    int success = 0;
    for(int i = 0; i < NUM_OF_TESTS; ++i){
        Graph g(INPUT_SIZE);
        generateRandomOVUndirectedEdges(g);
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
