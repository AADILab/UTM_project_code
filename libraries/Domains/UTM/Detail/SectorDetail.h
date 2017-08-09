#pragma once
#include "Domains/UTM/Sector.h"
#include "FixDetail.h"

class SectorDetail : public Sector {
public:
    SectorDetail(easymath::XY xy, size_t sectorIDset,
        std::vector<size_t> connections, std::vector<easymath::XY> dest_locs,
        MultiGraph<LinkGraph>* highGraph, MultiGraph<GridGraph>* lowGraph,
        std::list<UAVDetail*>* UAVs_done, size_t n_types_set) :
        Sector(xy, sectorIDset, connections, dest_locs, n_types_set) {

        FixDetail* f = new FixDetail(xy, sectorIDset, highGraph, lowGraph, dest_locs, n_types);
        f->UAVs_stationed = UAVs_done;
        generation_pt = f;
    }
    FixDetail *generation_pt;
};
