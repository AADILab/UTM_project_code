// Copyright 2016 Carrie Rebhuhn

#include "UAVDetail.h"
#include "STL/easystl.h"
#include <list>

using easymath::XY;
using easystl::clear;
using std::list;


UAVDetail::UAVDetail(XY start_loc, XY end_loc, UAVType t,
    LinkGraph* highGraph, GridGraph* lowGraph, size_t n_links_set) :
    UAV(lowGraph->get_membership(start_loc), lowGraph->get_membership(end_loc),
        t, highGraph), lowGraph(lowGraph), loc(start_loc), end_loc(end_loc), n_links(n_links_set) {
    std::printf("UAV %i created", get_ID());
}

void UAVDetail::planAbstractPath() {
    set_cur_sector_ID(get_cur_sector());
    if (search_mode == "astar"){
        high_path = Planning::astar(highGraph, cur_sector, end_sector);
    } else {
        // RAGS CALL
    }
}

void UAVDetail::planDetailPath() {
    // Get the high-level path
    high_path = get_best_path();

    // Get the astar low-level path
    XY next_loc = highGraph->get_vertex_loc(get_next_sector());

    if (next_sector != cur_sector) { // if not an internal link
        list<XY> low_path = Planning::astar(lowGraph, loc, next_loc);
        // Add to target waypoints
        target_waypoints.clear();
        for (XY i : low_path)
            target_waypoints.push_front(i);
        target_waypoints.pop_front();  // removes current location from target
        
     }
}


void UAVDetail::moveTowardNextWaypoint() {
    for (int i = 0; i < speed && !target_waypoints.empty(); i++) {
        loc = target_waypoints.front();
        target_waypoints.pop_front();
    }
}

bool UAVDetail::at_boundary() {
    if (target_waypoints.size() <= 1)
        return false; // at destination, not boundary
    else {
        // peek at first two element of queue
        easymath::XY first_pt = target_waypoints.front();
        easymath::XY second_pt = *std::next(target_waypoints.begin());

        int m1 = lowGraph->get_membership(first_pt);
        int m2 = lowGraph->get_membership(second_pt);
        return (m1 != m2);
    }
}