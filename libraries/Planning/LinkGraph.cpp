// Copyright 2016 Carrie Rebhuhn
#include "LinkGraph.h"

using std::string;
using std::list;
using std::vector;
using easymath::XY;
using easymath::get_n_unique_square_points;
using easymath::all_combos_of_2;
using easymath::zeros;
using easymath::line_segment;
using easymath::intersects_in_center;

LinkGraph::LinkGraph(size_t n_vertices, size_t xdim, size_t ydim) :
  g(n_vertices) {
  locations = get_n_unique_square_points(0.0, static_cast<double>(xdim), 0.0, static_cast<double>(ydim), n_vertices);

  for (size_t i = 0; i < n_vertices; i++) {
    loc2mem[locations[i]] = i;  // add in reverse lookup
  }

  vector<edge> candidates = all_combos_of_2(n_vertices);
  random_shuffle(candidates.begin(), candidates.end());

  // Add as many edges as possible, while still planar
  for (edge c : candidates) {
    if (!intersects_existing_edge(c)) {
      boost::add_edge(c.first, c.second, g);
      boost::add_edge(c.second, c.first, g);
    }
  }
}

vector<LinkGraph::edge> LinkGraph::get_edges() const {
  edge_iter ei, ei_end;
  vector<edge> edges_out(get_n_edges());
  size_t index = 0;
  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
    edges_out[index++] = edge((*ei).m_source, (*ei).m_target);
  }
  return edges_out;
}

LinkGraph::LinkGraph() : LinkGraph(1, 1, 1) {}


LinkGraph::LinkGraph(vector<XY> locs, const vector<edge> &edge_array) : LinkBase(), locations(locs) {
  for (size_t i = 0; i < locs.size(); i++)
    loc2mem[locs[i]] = i;

  g = mygraph_t(edge_array.begin(), edge_array.end(), locations.size());
  set_weights(matrix1d(edge_array.size(), 1.0));
}

//! This allows the blocking and unblocking of sectors by making travel
//! through a sector highly suboptimal.
void LinkGraph::blockVertex(int vertexID) {
  // Makes it highly suboptimal to travel to a vertex
  saved_weights = get_weights();

  edge_iter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
    if ((*ei).m_target == vertex_descriptor(vertexID))
      put(boost::edge_weight, g, *ei, 999999.99);
  }
}

void LinkGraph::unblockVertex() {
  set_weights(saved_weights);
}

void LinkGraph::set_weights(matrix1d weights) {
  // iterate over all edge descriptors...
  typedef boost::graph_traits<mygraph_t>::edge_iterator edge_iter;
  edge_iter ei, ei_end;
  size_t i = 0;

  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
    put(boost::edge_weight, g, *ei, weights[i++]);
  }
}

matrix1d LinkGraph::get_weights() const {
  edge_iter ei, ei_end;
  matrix1d weights;
  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
    weights.push_back(get(boost::edge_weight, g, *ei));
  }
  return weights;
}

void LinkGraph::print_graph(string file_path) {
  matrix2d connections_matrix = zeros(locations.size(), locations.size());

  for (auto ei = edges(g).first; ei != edges(g).second; ++ei) {
    connections_matrix[(*ei).m_source][(*ei).m_target] = true;
  }

  string CONNECTIONS_FILE = file_path + "connections.csv";
  string NODES_FILE = file_path + "nodes.csv";
  string EDGES_FILE = file_path + "edges.csv";
  
  cio::printPairs(locations, NODES_FILE);
  cio::printPairs(get_edges(), EDGES_FILE);
  cio::print2(connections_matrix, CONNECTIONS_FILE);
}

bool LinkGraph::fully_connected() {
  for (size_t i = 0; i < get_n_vertices(); i++) {
    for (size_t j = 0; j < get_n_vertices(); j++) {
      if (i == j) continue;
      list<size_t> p = Planning::astar(this, i, j);
      if (p.empty()) return false;
    }
  }
  return true;
}

bool LinkGraph::intersects_existing_edge(edge candidate) {
  XY b1 = locations[candidate.first];
  XY b2 = locations[candidate.second];
  line_segment b = line_segment(b1, b2);

  edge_iter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
    XY a1 = locations[(*ei).m_source];
    XY a2 = locations[(*ei).m_target];
    line_segment a = line_segment(a1, a2);

    if (intersects_in_center(a, b))
      return true;
  }

  // Check if any agent locations are being crossed by an edge
  // Exclude case where agent is at endpoint
  for (XY a : locations) {
    if (!is_endpt(a, b) && pt_on_line(a, b))
      return true;
  }
  return false;
}

size_t LinkGraph::get_direction(size_t m1, size_t m2) const {
  XY a = get_vertex_loc(m1);
  XY b = get_vertex_loc(m2);
  return cardinal_direction(b - a);
}

