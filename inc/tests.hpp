#ifndef __TESTS_HPP__
#define __TESTS_HPP__

#include "time.h"
#include "graph.hpp"

void generateEdges(Graph& g, int m);
void generateConstantO1Edges(Graph& g);
void generateConstantOVEdges(Graph& g);

bool runTests(Graph& g, int src, int dest, std::vector<double>& times, bool incbf);

bool check_result(const Graph& g, int src, int dest, const Path& path, int& bottleneck, bool first_test);
double printTimeSpecs(std::string name, struct timespec time_s, struct timespec time_e);
void updateAvgs(std::vector<std::pair<std::string, std::tuple<double, double, double>>>& avgs, std::vector<double> times, int divident);
void printAvgs(std::vector<std::pair<std::string, std::tuple<double, double, double>>> avgs);
void printResult(int success, int total);


#endif
