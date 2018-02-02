// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_SECTOR_H_
#define SRC_DOMAINS_UTM_SECTOR_H_

#include "FileIO/FileIn.h"
#include "Simulation/SimTime.h"
#include "Link.h"
#include <vector>
#include <map>
#include <math.h>

class Sector {
  public:
    typedef std::pair<size_t, size_t> edge;
    // An area of airspace to control
    Sector(easymath::XY loc, size_t sector_id, std::vector<size_t> connections,
      std::vector<easymath::XY> dest_locs, YAML::Node configs_set, size_t id, LinkGraph* high_graph,
      std::vector<size_t> dest_IDs) : k_loc_(loc), k_id_(sector_id), k_connections_(connections),
      configs(configs_set), high_graph_(high_graph), k_destination_IDs_(dest_IDs) {
      k_traffic_mode_ = configs["modes"]["traffic"].as<std::string>();
      k_destination_mode_ = configs["modes"]["destinations"].as<std::string>();
      if (k_traffic_mode_ == "probabilistic")
        k_gen_prob_ = configs["constants"]["generation_probability"].as<double>();
      else if (k_traffic_mode_ == "deterministic")
        k_gen_rate_ = configs["constants"]["generation_rate"].as<size_t>();
	
      destination_index = 0;
      k_num_generated_ = configs["constants"]["num_generated"].as<size_t>();
    }
    ~Sector() {}
    
	  SimTime* T;
	  size_t last_eval = 0;
	  std::vector<std::vector<size_t> > genfile;
    LinkGraph* high_graph_;
    std::vector<size_t> k_destination_IDs_;
    std::string k_traffic_mode_, k_destination_mode_;
    double k_gen_prob_;
    size_t k_gen_rate_, destination_index, k_num_generated_;
    
    void reset() {
      destination_index = 0;
	    waiting.clear();
    }

	  std::string domain_dir;

    YAML::Node configs;

    // Location properties
    const size_t k_id_;  // the identifier for this sector
    const std::vector<size_t> k_connections_;
    const easymath::XY k_loc_;  // sector center


    std::vector<UAV*> waiting;

    void remake(UAV* u) {
      resetUav(u);
    }

    /*
    Fix::Fix(YAML::Node configs_set, XY loc, size_t id, LinkGraph* high_graph,
        vector<size_t> dest_IDs, size_t sector_id) : configs(configs_set),
        high_graph_(high_graph), k_destination_IDs_(dest_IDs), k_id_(id),
        k_loc_(loc), k_sector_id_(sector_id) {
        k_traffic_mode_ = configs["modes"]["traffic"].as<std::string>();
        k_destination_mode_ = configs["modes"]["destinations"].as<std::string>();
        if (k_traffic_mode_ == "probabilistic")
            k_gen_prob_ = configs["constants"]["generation_probability"].as<double>();
        else if (k_traffic_mode_ == "deterministic")
            k_gen_rate_ = configs["constants"]["generation_rate"].as<size_t>();
        destination_index = 0;
        k_num_generated_ = configs["constants"]["num_generated"].as<size_t>();
    }*/

    bool shouldGenerateUav(size_t step) {
      if (k_traffic_mode_ == "constant") {
        return false;
      } else if (k_traffic_mode_ == "probabilistic") {
        double pnum = easymath::rand(0, 1);
        if (pnum > k_gen_prob_)
          return false;
        else
          return true;
      } else {
        // deterministic
        if (step% k_gen_rate_ != 0)
          return false;
        else
          return true;
      }
    }

    size_t generateUavs(size_t step, size_t *uavs_generated) {
//      std::cout << "Creating new UAVs in the world...\n" ;
      // Creates new UAVs in the world
      std::vector<UAV*> uavs = std::vector<UAV*>();
		  // This first if/else block is for "playing back" a particular pattern of UAV generation
		  // Not really used, but might be useful later?
		  if (k_traffic_mode_ == "playback") {
			  if (!genfile.size() || last_eval != T->eval) {
				  std::string fn = domain_dir +
					  "uav_gen_run" + std::to_string(T->run) + "_epoch" + std::to_string(T->epoch) +
					  "_eval" + std::to_string(T->eval) + ".csv";
				  genfile = cio::read2<size_t>(fn, ",");
				  last_eval = T->eval; // There's one file per eval
			  }
			  for (int i = 0; i < genfile[k_id_][T->step]; i++) {
				  uavs.push_back(generateUav(uavs_generated));
			  }
		  }
		  // If UAVs should be generated at this timestep...
      else if (shouldGenerateUav(step)) {
			  // ...generate number of UAVs specified in config file
//        for (int i = 0; i < k_num_generated_; i++) {
//            uavs.push_back(generateUav(uavs_generated));
//        }
        // JJC edit: generate at least one, but up to number of UAVs specified in config file
        int n_new_uavs = (int)ceil(easymath::rand(0,k_num_generated_)) ;
        for (int i = 0; i < n_new_uavs; i++) {
            uavs.push_back(generateUav(uavs_generated));
        }
      }
      
      if (uavs.size()) // If UAVs were generated, they are waiting at sector
        waiting.insert(waiting.end(), uavs.begin(), uavs.end());

		  return uavs.size();
    }

    UAV* generateUav(size_t *uavs_generated) {
      auto e = high_graph_->get_edges();
      size_t end_id = selectDestinationID();

      UAV* u = new UAV(configs, k_id_, end_id, high_graph_, uavs_generated);
      return u;
    }

    void resetUav(UAV* u) {
      // Resets the destination of the UAV.
      // Unless the destination mode is static, it sends the UAV to a fixed 'partner node'
      size_t index = selectDestinationID();
      u->reset(k_id_, index);
    }

    size_t selectDestinationID() {
      size_t num_destinations = k_destination_IDs_.size();
      
      if (k_destination_mode_ == "static") {
        destination_index++;
        if (destination_index >= num_destinations) {
          destination_index = 0;
        }
      } else {
        destination_index = rand() % num_destinations;
      }
      return k_destination_IDs_[destination_index];
    }
};

#endif  // SRC_DOMAINS_UTM_SECTOR_H_
