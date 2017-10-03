// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_SECTORAgent_H_
#define SRC_DOMAINS_UTM_SECTORAgent_H_

#include <map>
#include <list>
#include <vector>

//! Class that manages sectors_ as agents
class SectorAgent : public IAgentBody {
  public:
    SectorAgent(std::vector<Link*> links, std::vector<Sector*> sectors, size_t num_state_elements) :
      IAgentBody(sectors_.size(), num_state_elements), links_(links), sectors_(sectors) {
      for (Link* l : links_) {// stores incoming links
         k_links_toward_sector_[l->k_target_].push_back(l);
      }
    }
    virtual ~SectorAgent() {}

    std::vector<Link*> links_;  // links_ in the entire system
    std::map<int, std::vector<Link*> > k_links_toward_sector_;
    
    matrix2d computeCongestionState(const std::list<UAV*>& uavs) {
      size_t n_agents = sectors_.size();
      matrix2d allStates = easymath::zeros(n_agents, k_num_states_);

      std::vector<int> sector_congestion_count(n_agents, 0);
      for (UAV* u : uavs) {
        sector_congestion_count[u->getCurSector()]++;
      }
      for (size_t i = 0; i < sectors_.size(); i++) {
        for (int conn : sectors_[i]->k_connections_) {
          easymath::XY dx = sectors_[conn]->k_loc_ - sectors_[i]->k_loc_;
          size_t dir = cardinal_direction(dx);
          allStates[i][dir] += sector_congestion_count[conn];
        }
      }
      agent_states_.push_back(allStates);
      return allStates;
    }

    virtual matrix1d actionsToWeights(matrix2d agent_actions) {
      // Converts format of agent output to format of A* weights
      matrix1d weights = easymath::zeros(links_.size());
      
      // Note that directions are discretised into {NE - 0, NW - 1, SE - 2, SW - 3}
      for (size_t i = 0; i < links_.size(); i++) {
        double predicted = links_.at(i)->predictedTraversalTime();
        size_t s = links_[i]->k_source_;
        size_t d = links_[i]->k_cardinal_dir_;
        weights[i] = predicted + agent_actions[s][d] * k_alpha_;
      }
      
      return weights ;
    }
    
    std::vector<Sector*> sectors_;
};
#endif  // SRC_DOMAINS_UTM_SECTORAgent_H_
