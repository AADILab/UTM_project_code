#pragma once

#include "yaml-cpp/yaml.h"
#include "Domains/UTM/Fix.h"
#include "UAVDetail.h"

class FixDetail : public Fix {
 public:
    FixDetail(easymath::XY loc, size_t ID, MultiGraph<LinkGraph>* highGraph,
        MultiGraph<GridGraph>* lowGraph, std::vector<easymath::XY> dest_locs, size_t n_types_set) :
        Fix(loc, ID, highGraph, dest_locs, n_types_set),
        lowGraph(lowGraph)
    {
        YAML::Node configs = YAML::LoadFile("config.yaml");
        approach_threshold = configs["constants"]["approach_threshold"].as<double>();
        conflict_threshold = configs["constants"]["conflict_threshold"].as<double>();
    };

    virtual ~FixDetail() {}
    MultiGraph<GridGraph>* lowGraph;

    //! Calls a conditional, then creates UAV in the world
    virtual UAVDetail* generate_UAV(int step);


    virtual bool atDestinationFix(const UAVDetail &u);
    std::list<UAVDetail*> * UAVs_stationed;

private:

    //! Creates a new UAV in the world
    virtual UAVDetail* generate_UAV();
    double approach_threshold;
    double conflict_threshold;
    size_t n_types;
};