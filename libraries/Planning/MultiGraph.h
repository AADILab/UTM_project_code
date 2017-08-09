// Copyright 2015 Carrie Rebhuhn
#ifndef PLANNING_MULTIGRAPH_H
#define PLANNING_MULTIGRAPH_H

#include "Math/easymath.h"
#include "STL/easystl.h"

/**
* This class handles multiple layers of weights that have the same underlying
* graph structure. Creates n identical instances of 'base_set'
*/
template <class GraphType>
class MultiGraph {
  public:
    MultiGraph() {}
    MultiGraph(size_t n, GraphType* base_set)
    {
      base = base_set;
      layers = std::vector<GraphType*>(n);
      for (size_t i = 0; i < n; i++)
        layers[i] = new GraphType(*base);
    }
    ~MultiGraph() {
      delete base;
      easystl::clear(layers);
    }
    std::vector<GraphType*> layers; // This holds the set of graph layers.
    GraphType* base;                // This defines the structure of the graph.
    GraphType* at(size_t index) { return layers[index]; }
    GraphType* at() { return base; }
};

#endif
