// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UAVDETAIL_H_
#define DOMAINS_UTM_UAVDETAIL_H_

// STL includes
#include <queue>
#include <map>

// Library includes
#include "Domains/UTM/UAV.h"
#include "Planning/GridGraph.h"

class UAVDetail : public UAV {
public:
    UAVDetail(easymath::XY start_loc, easymath::XY end_loc, UAVType t,
        LinkGraph* highGraph, GridGraph* lowGraph, size_t n_links_set);
    virtual ~UAVDetail() {};

    // Comparison accessors
    bool has_plan() const { return !target_waypoints.empty(); }
    bool is_next(easymath::XY l) const { return target_waypoints.front() == l; }
    bool is_goal(easymath::XY l) const { return end_loc == l; }
    bool at_boundary();
    bool has_detail_plan() { return !target_waypoints.empty(); };
    bool on_internal_link(size_t next_link_ID) {
        return get_cur_link() < n_links && next_link_ID >= n_links; }
    double distance_to(easymath::XY l) const { return easymath::euclidean_distance(loc, l); }

    // Regular accessors
    easymath::XY get_location() { return loc; }
    //! Gets the sector ID from the current location
    virtual size_t get_cur_sector() const { return lowGraph->get_membership(loc); }

    // Mutators
    virtual void planAbstractPath();
    void planDetailPath();
    void moveTowardNextWaypoint();

private:
    //! Physical UAV location
    easymath::XY loc, end_loc;
    //! Low-level waypoints
    std::list<easymath::XY> target_waypoints;
    //! Low-level graph
    GridGraph* lowGraph;
    size_t n_links;
};

#endif  // DOMAINS_UTM_UAVDETAIL_H_