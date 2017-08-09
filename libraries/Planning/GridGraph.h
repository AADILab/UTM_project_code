// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_GRIDGRAPH_H_
#define PLANNING_GRIDGRAPH_H_


// from boost
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/unordered_map.hpp>

// from stl
#include <float.h>
#include <fstream>
#include <utility>
#include <string>
#include <vector>
#include <functional>


#include "Planning.h"
// from libraries
#include "Math/easymath.h"



/**
* This is a specialization of a graph for an 8-connected grid.
*/
typedef boost::grid_graph<2> grid;

// A hash function for vertices.
struct vertex_hash :std::unary_function<grid::vertex_descriptor, std::size_t> {
  std::size_t operator()(grid::vertex_descriptor const& u) const {
    return this->operator()(u[0], u[1]);
  }
  std::size_t operator()(size_t x, size_t y) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, x);
    boost::hash_combine(seed, y);
    return seed;
  }
};


struct vertex_equal {
  bool operator()(const grid::vertex_descriptor &rhs, const grid::vertex_descriptor &lhs) const {
    return rhs[0] == lhs[0] && rhs[1] == lhs[1];
  }
};


typedef IBoostGraph<grid, easymath::XY, vertex_hash, vertex_equal> GridBase;

class GridGraph : public GridBase {
  public:
    // Types required for boost A* use
    typedef typename GridBase::vertex_descriptor vertex_descriptor;
    typedef typename GridBase::dist_map dist_map;
    typedef typename GridBase::pred_map pred_map;

    GridGraph & operator =(const GridGraph &) {
      return *this;
    }
    
    typedef boost::unordered_set<vertex_descriptor, vertex_hash, vertex_equal> vertex_set;
    typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type filtered_grid;

    vertex_descriptor get_descriptor(easymath::XY pt) {
      return{ static_cast<size_t>(pt.x), static_cast<size_t>(pt.y) };
    }
    easymath::XY get_vertex_base(vertex_descriptor v) {
      return easymath::XY(static_cast<double>(v[0]), static_cast<double>(v[1]));
    }
    double get_x(vertex_descriptor v) { return static_cast<double>(v[0]); }
    double get_y(vertex_descriptor v) { return static_cast<double>(v[1]); }

    typedef std::pair<size_t, size_t> edge;
    typedef std::vector<std::vector<bool> > barrier_grid;


    //! Should only be called from other constructor
    explicit GridGraph(barrier_grid obstacle_map);

    //! Defines the membership for each cell in the grid
    matrix2d members;

    //! Create the underlying rank-2 grid with the specified dimensions.
    grid create_grid(std::size_t x, std::size_t y);

    //! Filter the barrier vertices out of the underlying grid.
    filtered_grid create_barrier_grid();

    //! The barriers in the AStarGrid
    vertex_set m_barriers;

    grid m_grid;

  public:
    //! The underlying AStarGrid grid with barrier vertices filtered out
    filtered_grid g;

    GridGraph() : GridGraph(easymath::zeros(1, 1)) {}

    explicit GridGraph(const matrix2d &members);
    virtual ~GridGraph() {}

    //! Adds barriers if a cell does not match membership m1 or m2
    void occlude_nonmembers(int m1, int m2);

    // Accessor functions
    const int get_membership(easymath::XY p) {
      return static_cast<int>(
        members[static_cast<size_t>(p.x)][static_cast<size_t>(p.y)]);
    }
};
#endif  // PLANNING_GRIDGRAPH_H_
