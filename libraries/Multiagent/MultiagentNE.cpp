// Copyright 2016 Carrie Rebhuhn
#include "MultiagentNE.h"

using std::vector;

MultiagentNE::MultiagentNE(YAML::Node configs, size_t num_agents, std::vector<bool> active) {
  std::printf("Creating a MultiagentNE object.\n");
  
  if (!active.size()) { // assume all are active
    active_ = std::vector<bool>(num_agents, true);
  }

  for (size_t i = 0 ;i < num_agents; i++) {
    agents.push_back(new NeuroEvo(configs,active_[i]));
  }
}

MultiagentNE::MultiagentNE(YAML::Node configs, IDomainStateful* domain, std::vector<bool> active):
  MultiagentNE(configs, domain->getNumAgents(), active) {
}

MultiagentNE::~MultiagentNE(void) {
  for (size_t i = 0; i < agents.size(); i++) {
    delete agents[i];
  }
}

// Get IDS of team members (debugging purposes)
matrix1d MultiagentNE::get_team_member_ids() {
	size_t numMems = reinterpret_cast<NeuroEvo*>(agents[0])->pop_old.size()*2;
	matrix1d ids;
	for (size_t i = 0; i < numMems; i++) {
		size_t id = reinterpret_cast<NeuroEvo*>(agents[i])->getCurrentMemberId();
		ids.push_back(id);
	}
	return ids;
}

void MultiagentNE::generate_new_members() {
  // Generate new population members
  for (size_t i = 0; i < agents.size(); i++) {
    reinterpret_cast<NeuroEvo*>(agents[i])->generateNewMembers();
  }
}

void MultiagentNE::select_survivors(bool sh) {
  // Specific to Evo: select survivors
  for (size_t i = 0; i < agents.size(); i++) {
    reinterpret_cast<NeuroEvo*>(agents[i])->selectSurvivors(sh);
  }
}

bool MultiagentNE::set_next_pop_members() {
  // Kind of hacky; select the next member and return true if not at the end
  // Specific to Evo

  vector<bool> is_another_member(agents.size(), false);
  for (size_t i = 0; i < agents.size(); i++) {
    is_another_member[i] = reinterpret_cast<NeuroEvo*>(agents[i])->selectNewMember();
  }
  for (size_t i = 0; i < is_another_member.size(); i++) {
    if (!is_another_member[i]) {
      return false;
    }
  }
  return true;
}
