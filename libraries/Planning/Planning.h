// Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include <boost/unordered_map.hpp>
#include <boost/graph/astar_search.hpp>
#include <list>
#include <vector>
#include <utility>

#include "IBoostGraph.h"

namespace Planning {
  namespace detail {
    //! Exception for graph search termination.
    struct found_goal {};

    //! Defines the point when a goal is found
    template <class V>
    class astar_goal_visitor : public boost::default_astar_visitor {
      public:
        explicit astar_goal_visitor(V goal) : m_goal(goal) {}
        astar_goal_visitor() {}
        template <class G>
        void examine_vertex(V u, G&) {
          if (u == m_goal) {
            throw found_goal();
          }
        }
        V m_goal;
    };

    //! Euclidean heuristic. Requires a class object that has functions that
    //! translate vertex descriptor to x and y locations.
    template<class G, class Gbase>
    struct euclidean_heuristic : public boost::astar_heuristic<Gbase, double> {
      typedef typename boost::graph_traits<Gbase>::vertex_descriptor V;
      euclidean_heuristic(G* funcs, Gbase g, V goal) : m_goal(goal), g(g), funcs(funcs) {}

      double operator()(V v) {
        double dx = funcs->get_x(m_goal) - funcs->get_x(v);
        double dy = funcs->get_y(m_goal) - funcs->get_y(v);
        return sqrt(dx*dx + dy*dy);
      }
      V m_goal;
      Gbase g;
      G* funcs;    // Allows access to get_x and get_y functions
    };

    //! Backend for retrieving the euclidean heuristic.
    //! Takes a boost graph object for Gbase, so that template can be deduced.
    template <class G, class Gbase, class V>
    auto get_euclidean_heuristic(G* funcs, Gbase g, V v) {
      return euclidean_heuristic<G, Gbase>(funcs, g, v);
    }

    //! Gets the bgl named params necessary for astar search
    template<class G, class Gbase, class V>
    auto get_params(G* GraphWrapper, const Gbase&, const V& goal) {
      GraphWrapper->pred_pmap = boost::associative_property_map<typename G::pred_map>(GraphWrapper->predecessor);
      
      return boost::weight_map(GraphWrapper->weight)
        .predecessor_map(GraphWrapper->pred_pmap)
        .distance_map(GraphWrapper->dist_pmap)
        .visitor(detail::astar_goal_visitor<V>(goal));
    }

  }  // namespace detail

  template <class G, class V>
  std::list<V> astar(G* g, V start, V goal) {
    auto s = g->get_descriptor(start);
    auto e = g->get_descriptor(goal);
    auto h = detail::get_euclidean_heuristic(g, g->g, e);

    // initializes and resets the predecessor maps
    g->init_pmaps();

    auto gv = Planning::detail::astar_goal_visitor<typename G::vertex_descriptor>(e);
    //auto p = boost::weight_map(g->weight).predecessor_map(g->pred_pmap).distance_map(g->dist_pmap).visitor(gv);
    //auto p = boost::weight_map(g->weight).predecessor_map(g->pred_pmap).visitor(gv); // dist map removed because it was overriding weight map
    // weight map was causing this not to work; rely instead on externally set edge properties
    auto p = boost::predecessor_map(g->pred_pmap).visitor(gv);
    std::list<V> solution;
    try {
      boost::astar_search(g->g, s, h, p);
    }
    catch (detail::found_goal) {
      for (auto u = e; ; u = g->pred_pmap[u]) {
        V val = g->get_vertex_base(u);
        solution.push_back(val);
        if (u == g->pred_pmap[u])
          break;
      }
      std::reverse(solution.begin(), solution.end());
      return solution;
    }

    return solution;
  }
}  // namespace Planning
#endif  // PLANNING_PLANNING_H_
