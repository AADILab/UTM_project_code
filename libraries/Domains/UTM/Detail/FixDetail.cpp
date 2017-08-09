#include "FixDetail.h"

using easymath::XY;
using easymath::rand;
using std::list;

bool FixDetail::atDestinationFix(const UAVDetail &u) {
    bool is_close = u.distance_to(loc) < approach_threshold;

    return u.has_plan() && u.is_next(loc) && is_close && u.is_goal(loc);
}

UAVDetail* FixDetail::generate_UAV(int step) {
    if (should_generate_UAV(step))
        return generate_UAV();
    else {
        return NULL;
    }
}

UAVDetail* FixDetail::generate_UAV() {
    static size_t calls = 0;
    XY end_loc;
    if (ID == 0)
        end_loc = destination_locs.back();
    else
        end_loc = destination_locs.at(ID - 1);  // go to previous

    size_t type_id_set = calls%n_types;
    LinkGraph* high = highGraph->at(type_id_set);
    GridGraph* low = lowGraph->at(type_id_set);

    return new UAVDetail(loc, end_loc, type_id_set, high, low, n_types);
}

