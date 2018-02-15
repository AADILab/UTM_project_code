// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_MULTIAGENT_INCLUDE_MULTIAGENTNE_H_
#define SRC_MULTIAGENT_INCLUDE_MULTIAGENTNE_H_

#include <stdio.h>
#include <vector>
#include "IMultiagentSystem.h"
#include "Learning/NeuroEvo.h"
#include "Domains/IDomainStateful.h"

class MultiagentNE : public IMultiagentSystem<NeuroEvo> {
  public:
    MultiagentNE(void) {}
    MultiagentNE(YAML::Node configs, size_t num_agents, std::vector<bool> active);
    explicit MultiagentNE(YAML::Node configs, IDomainStateful *domain, std::vector<bool> active=std::vector<bool>());
    ~MultiagentNE(void);
	  
	  matrix1d get_team_member_ids();
    void generate_new_members();
    virtual void select_survivors(bool sh = true);
    virtual bool set_next_pop_members();
    std::vector<double> get_survivor_persistence() {
      std::vector<double> sp(agents.size());
      for (size_t i = 0; i < agents.size(); i++) {
        sp[i] = agents.at(i)->getSurvivorPersistence();
      }
      return sp;
    }
	
    /** Get mean squared difference between all parent-child pairs for all agents
    *  \return 2D matrix where each row contains msds for one agent's networks
    */
	  virtual matrix2d get_msds()
	  {
		  matrix2d msds(agents.size());
		  for (size_t i = 0; i < agents.size(); i++) {
			  msds[i] = agents.at(i)->get_msds();
		  }
		  return msds;
	  }

    /** Get ranges of weights for all pop members for all agents
    *  \return 3D matrix. Each primary index indexes a 2D matrix which contains, 
    *  for each row (network), the min and max weights.
    */
	  virtual matrix3d get_ranges()
	  {
		  matrix3d allRanges(agents.size());
		  for (size_t i = 0; i < agents.size(); i++) {
			  allRanges[i] = agents.at(i)->getWtRanges();
		  }
		  return allRanges;
	  }

    /** Get fitness values for all pop members for all agents
    *  \return 3D matrix. Each primary index indexes a 2D matrix which contains,
    *  for each row (network), the ID of the network and its fitness.
    */
	  virtual matrix3d get_evals()
	  {
		  matrix3d allEvals(agents.size());
		  for (size_t i = 0; i < agents.size(); i++) {
			  allEvals[i] = agents.at(i)->getPopEval();
		  }
		  return allEvals;
	  }
};
#endif  // SRC_MULTIAGENT_INCLUDE_MULTIAGENTNE_H_
