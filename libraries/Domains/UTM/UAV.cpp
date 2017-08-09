// Copyright 2016 Carrie Rebhuhn

#include "UAV.h"


using std::list;

UAV::UAV(YAML::Node configs, int start_sector, int end_sector, LinkGraph* high_graph, size_t *id) :
  high_graph_(high_graph), end_sector_(end_sector), k_id_(*id) {
  (*id)++;

  position_ = Position();
  path_ = Path();
  position_.cur_sector_ = start_sector;

  k_search_mode_ = configs["modes"]["search"].as<std::string>();

  // Get initial plan and update
  planAbstractPath();
}

void UAV::reset(int start_sector, int end_sector) {
  //if (k_id_ == 0) {
  //    printf("Resetting the UAV!\n");
  //}
  position_.cur_sector_ = start_sector;
  end_sector_ = end_sector;

  // Get initial plan and update
  planAbstractPath();
}

bool UAV::planAbstractPath() {
  auto prev_path = path_;
  if (k_search_mode_ == "astar") {
    path_ = getBestPath();
  }
  if (path_.empty()) {
    printf("Path not found!");
  }

  return path_ == prev_path;
}
