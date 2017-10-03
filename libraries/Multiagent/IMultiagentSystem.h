// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_MULTIAGENT_INCLUDE_IMULTIAGENTSYSTEM_H_
#define SRC_MULTIAGENT_INCLUDE_IMULTIAGENTSYSTEM_H_


// STL includes
#include <iostream>
#include <vector>

// Library includes
#include "Learning/IAgent.h"

//! Agents container
template<class Agent>
class IMultiagentSystem {
  public:
    typedef typename Agent::State State;
    typedef typename Agent::Action Action;
    typedef typename Agent::Reward Reward;

    IMultiagentSystem(void) {}
    virtual ~IMultiagentSystem(void) {}

    // Set of agents in the system (set externally)
    std::vector<Agent*> agents;
    std::vector<bool> active_; // agents that are active in the system

    std::vector<Action> getActions(std::vector<State> S) {
      std::vector<Action> A(S.size());
      // get all actions, given a list of states
      for (std::size_t i = 0; i < agents.size(); i++) {
        A[i] = agents[i]->getAction(S[i]);
      }
      return A;
    }
    
    inline void update_policy_values(std::vector<Reward> R) {
      for (std::size_t i = 0; i < agents.size(); i++)
        agents[i]->updatePolicyValues(R[i]);
    }
};
#endif  // SRC_MULTIAGENT_INCLUDE_IMULTIAGENTSYSTEM_H_
