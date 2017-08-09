//! Copyright 2016 Carrie Rebhuhn
#ifndef PLANNING_IBOOSTGRAPH_H_
#define PLANNING_IBOOSTGRAPH_H_


//! Interface for a class to use the astar planning
template <class G, class vertex_base, class vertex_hash, class vertex_equal>
class IBoostGraph {
  public:
    IBoostGraph() : pred_pmap(predecessor), weight(boost::static_property_map<double>(1)) {}
    virtual ~IBoostGraph() {}

    typedef typename boost::graph_traits<G>::vertex_descriptor vertex_descriptor;

    virtual vertex_descriptor get_descriptor(vertex_base) = 0;
    virtual vertex_base get_vertex_base(vertex_descriptor) = 0;
    virtual double get_x(vertex_descriptor) = 0;
    virtual double get_y(vertex_descriptor) = 0;
    void init_pmaps() {
      pred_pmap = boost::associative_property_map<pred_map>(predecessor);
//        dist_pmap = boost::associative_property_map<dist_map>(distance);
    }

    //! Maps are vertex-to-vertex mapping.
    typedef typename boost::unordered_map<vertex_descriptor, vertex_descriptor, vertex_hash, vertex_equal> pred_map;
    pred_map predecessor;
    boost::associative_property_map<pred_map> pred_pmap;

    typedef typename boost::unordered_map<vertex_descriptor, double, vertex_hash, vertex_equal> dist_map;
    dist_map distance;
//    boost::associative_property_map<dist_map> dist_pmap;

    //typedef boost::property_map<G, boost::edge_weight_t> WeightMap;
    //WeightMap weight;

    boost::static_property_map<double> weight;
};
#endif  // PLANNING_IBOOSTGRAPH_H_
