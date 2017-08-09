#pragma once
#include "Planning/LinkGraph.h"

class Impactfulness{
public:
    Impactfulness(std::string domain) {
        std::cout << "Impactfulness string input: " << domain << "\n" ;
        domain_file = domain;
        graph = new LinkGraph(domain);
        N = graph->get_n_vertices();
    };

    std::string domain_file;
    LinkGraph* graph;
    int N;

    typedef std::list<int> Path;
    typedef std::vector<std::vector<std::list<int> > > Paths;
    
    void printWeights() {
        auto eiter = boost::edges(graph->g);
        for (auto ei = eiter.first; ei != eiter.second; ++ei) {
            double w = boost::get(boost::edge_weight_t(), graph->g, *ei);
            std::printf("%f, ", w);
        }
        std::printf("\n");
    }

    Paths getBestPaths() {
        //std::printf("Getting the best paths for %i x %i nodes.", N, N);
        Paths P(N, std::vector<Path>(N));
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                if (i == j) continue;
                Path p = Planning::astar(graph, i, j);
                P[i][j] = p;
            }
        }
        return P;
    }

    double compare(Paths p1, Paths p2) {
        int same = 0;
        int allunique = 0;
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                if (i == j) continue;
                auto pi1 = p1[i][j];
                auto pi2 = p2[i][j];
                if (p1[i][j] == p2[i][j]) {
                    same++;
                }  else {
                    /*
                    std::printf("%i,%i: ", i, j);
                    std::printf("{");
                    for (auto p1i : p1[i][j]) {
                        std::printf("%i ", p1i);
                    }
                    std::printf("} ");
                    
                    std::printf("{");
                    for (auto p2i : p2[i][j]) {
                        std::printf("%i ", p2i);
                    }
                    std::printf("} ");
                    std::printf("\n");
                    //*/
                }
                allunique++;
            }
        }
        return double(allunique - same) / double(allunique);
    }

    void calculate() {
        // calculate A* optimal path for each node on graph
        // iterate across all EDGES


        double avg = 10.0;
        double hi = 1000000.0;
        double lo = 0.0;

        matrix1d w(graph->get_n_edges(),avg);
        graph->set_weights(w);
        Paths flat_paths = getBestPaths();
        matrix1d impactfulness_lo(graph->get_n_edges());
        matrix1d impactfulness_hi(graph->get_n_edges());

        //for (int i = 0; i < graph->get_n_edges(); i++) {
        
        auto eiter = boost::edges(graph->g);
        auto ei = eiter.first;
        auto ei_end = eiter.second;
        int i = 0;


        std::vector<std::pair<int, int> > edges_first(graph->get_n_edges());
        for (ei = eiter.first; ei != ei_end; ++ei) {
            std::pair<int, int> p(boost::source(*ei, graph->g),
            boost::target(*ei, graph->g));
            edges_first[i++] = p;
        }

        i = 0;
        for (ei=eiter.first; ei != ei_end; ++ei){
            //std::cout << edge_distance[*ei] << endl;
            
            printf("Edge %i, %i,%i...\n",i, boost::source(*ei,graph->g), boost::target(*ei,graph->g));
            //system("pause");
            // go high
            //w[i] = 1000000;
            boost::put(boost::edge_weight_t(), graph->g, *ei, hi);
            //graph->set_weights(w);
            //graph->set_weights(w);
            
            //printWeights();

            Paths hi_paths = getBestPaths();
            impactfulness_hi[i] = compare(flat_paths, hi_paths);
            printf("hi: %f\n", impactfulness_hi[i]);
            
            // go low
            //w[i] = 0;
            boost::put(boost::edge_weight_t(), graph->g, *ei, lo);
            //graph->set_weights(w);
            Paths(lo_paths) = getBestPaths();
            impactfulness_lo[i] = compare(flat_paths, lo_paths);

            // 
            boost::put(boost::edge_weight_t(), graph->g, *ei, avg);
            printf("lo: %f\n", impactfulness_lo[i]);
            i++;
            //system("pause");
        }
        cio::print<double>(impactfulness_lo, domain_file + "impactfulness_lo");
        cio::print<double>(impactfulness_hi, domain_file + "impactfulness_hi");
    }
};
