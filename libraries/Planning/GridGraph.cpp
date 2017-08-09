// Copyright 2016 Carrie Rebhuhn
#include "GridGraph.h"

using std::vector;
using easymath::XY;
using easymath::operator <;

GridGraph::GridGraph(barrier_grid obstacle_map) : GridBase(), m_grid(create_grid(obstacle_map.size(), obstacle_map[0].size())), g(create_barrier_grid()) {
  /**
  * This map shows all grid cells except those in obstacle_map as passable.
  */
  int v_index = 0;
  for (size_t y = 0; y < obstacle_map[0].size(); y++) {
    for (size_t x = 0; x < obstacle_map.size(); x++) {
      // Place barriers
      if (obstacle_map[x][y]) {
        vertex_descriptor u = { x, y };
        m_barriers.insert(u);
      }
      // Increment vertex index even if no barrier added
      v_index++;
    }
  }
}

GridGraph::GridGraph(const matrix2d &members_set) : GridGraph(members_set < 0) {
  members = members_set;
}

void GridGraph::occlude_nonmembers(int m1, int m2) {
  /**
  * This map only shows grid cells of membership m1 and m2 as passable.
  * Others are barriers. Backflow (travel from m2 to m1) is allowed but will
  * tend to be suboptimal, so is improbable.
  */
  int v_index = 0;
  for (size_t y = 0; y < members[0].size(); y++) {
    for (size_t x = 0; x < members.size(); x++) {
      // Place barriers
      bool obstacle = members[x][y] < 0;
      bool wrong_member = (members[x][y] != m1 && members[x][y] != m2);
      if (obstacle || wrong_member) {
        vertex_descriptor u = { x, y };
        m_barriers.insert(u);
      }
      // Increment vertex index even if no barrier added
      v_index++;
    }
  }
}

grid GridGraph::create_grid(std::size_t x, std::size_t y) {
  boost::array<std::size_t, 2> lengths = { { x, y } };
  return grid(lengths);
}

GridGraph::filtered_grid GridGraph::create_barrier_grid() {
  return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
}
