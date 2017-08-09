// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_UAV_H_
#define SRC_DOMAINS_UTM_UAV_H_

// STL includes
#include <string>
#include <list>
#include <set>
#include <utility>
#include <tuple>
#include <map>

// libraries includes
#include "Planning/LinkGraph.h"
#include "yaml-cpp/yaml.h"

class UAV {
    /*
    This class is for moving UAVs in the airspace. They interact with the
    environment through planning. Planning is done through boost.
    */
  public:
	  size_t end_sector_; // goal

    typedef std::pair<size_t, size_t> edge;

    // Mutators

    UAV(YAML::Node configs, int start_sector, int end_sector, LinkGraph* high_graph, size_t *id);
    virtual ~UAV() {};
    virtual bool planAbstractPath();
    void reset(int start_sector, int end_sector);

    int getDestinationID() { return end_sector_; }

    std::pair<size_t, size_t> getNthEdge(size_t n) const {
      // Current edge requires no lookahead
      if (n == 0)
        return position_.cur_edge_;
      
      // If UAV is not on the path, it takes on extra step to get on it
      if (!onPath())
        n--;

      // Return the nth step of the projected path
      return path_.getNthEdge(n);
    }

    void setCurEdge(size_t source, size_t target, int id) {
      position_.cur_edge_ = std::pair<size_t, size_t>(source, target);
      position_.cur_edge_id_ = id;
    }
    void setWait(int time) { position_.t_ = time; }
    void decrementWait() { position_.t_--; }
    void setCurSector(size_t s) { position_.cur_sector_ = s; }

    // accessors
    bool terminal() const { return position_.cur_sector_ == end_sector_; }
    bool travelling() const { return position_.t_ != 0; }
    int getWait() const { return position_.t_; }
    size_t getId() const { return k_id_; }
    std::pair<size_t, size_t> getCurEdge() const { return position_.cur_edge_; }
    int getCurEdgeID() const { return position_.cur_edge_id_; }
    bool onPath() const {
      return position_.cur_edge_ == path_.getNthEdge(0);
    }


    size_t getCurSector() const {
      return position_.cur_sector_;
    }
    
    void setCurEdgeFromPath() {
      position_.cur_edge_ = path_.getNthEdge(0);
    }

	  //For debugging
	  std::list<size_t> getPath() {
		  return path_.getHighPath();
	  }

	  void incrementPath() {
		  path_.incrementPath();
	  }

  private:

    // All variables relating to the path/plan
    struct Path {
      size_t front() const {
        return high_path_.front();
      }

      size_t incrementPath() {
        // Pops off the first element of the path
        high_path_.pop_front();

        // Returns the new first element
        return high_path_.front();
      }

      size_t getNthSector(size_t n) const {
        if (high_path_.size() <= n || n == 0)
          return front();
        else
          return *std::next(high_path_.begin(), n);  // zero indexed
      }

      std::pair<size_t, size_t> getNthEdge(size_t n) const {
        return std::make_pair(getNthSector(n), getNthSector(n + 1));
      }
      void operator=(std::list<size_t> p) {
        high_path_ = p;
      }

      bool empty() const { return high_path_.empty(); }

      void print() {
        for (auto i : high_path_) {
          std::printf("%i, ", i);
        }
        std::printf("\n");
      }

      bool operator==(const Path &p) {
        return high_path_ == p.high_path_;
      }

		  //For debugging
		  std::list<size_t> getHighPath() {
			  return high_path_;
		  }
    private:
      std::list<size_t> high_path_;
    } path_;

    int getTravelDirection() const {
      return high_graph_->get_direction(position_.cur_sector_, position_.nextSector());
    }

    std::string k_search_mode_;

    LinkGraph* high_graph_;
    std::list<size_t> getBestPath() const {
      return Planning::astar<LinkGraph, size_t> (high_graph_, position_.cur_sector_, end_sector_);
    }
    size_t k_id_;  //! const in run, but based on non-constant variable

    // Typedefs
    
    struct Position {
      size_t nextSector() const { return cur_edge_.second; }
      std::pair<size_t, size_t> cur_edge_; // Stored to be consistent with physical movement
      int cur_edge_id_;
      int t_;
      size_t cur_sector_;
    } position_;
};
#endif  // SRC_DOMAINS_UTM_UAV_H_
