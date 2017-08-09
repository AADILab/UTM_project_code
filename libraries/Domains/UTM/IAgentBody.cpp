// Copyright 2016 Carrie Rebhuhn
#include "IAgentBody.h"

using std::string;

IAgentBody::IAgentBody(size_t num_agents, size_t num_states) :
	k_num_states_(num_states)
{
	YAML::Node config = YAML::LoadFile("config.yaml");
	k_square_reward_mode_ = config["modes"]["square"].as<bool>();
	k_alpha_ = config["constants"]["alpha"].as<double>();
}

void IAgentBody::logAgentActions(matrix2d agent_step_actions) {
  agent_actions_.push_back(agent_step_actions);
}

bool IAgentBody::lastActionDifferent() {
  if (agent_actions_.size() > 1) {
    matrix2d last_action = agent_actions_.back();
    matrix2d cur_action = agent_actions_[agent_actions_.size() - 2];

    return last_action != cur_action;
  }
  return true;
}

void IAgentBody::exportAgentActions(int fileID) {
  string actionfile = "actions-" + std::to_string(fileID) + ".csv";
  string statefile = "states-" + std::to_string(fileID) + ".csv";
  cio::print3(agent_actions_, actionfile);
  cio::print3(agent_states_, statefile);
}

void IAgentBody::reset() {
  agent_actions_.clear();
  agent_states_.clear();
}
