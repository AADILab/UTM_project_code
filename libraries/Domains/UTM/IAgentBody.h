// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_IAGENTBODY_H_
#define SRC_DOMAINS_UTM_IAGENTBODY_H_

#include <string>
#include <memory>
#include <list>
#include <vector>

#include "Math/easymath.h"
#include "FileIO/FileIn.h"
#include "UAV.h"
#include "Learning/NeuralNet.h"
#include "yaml-cpp/yaml.h"

class IAgentBody {
  public:
    // Life cycle
    IAgentBody(size_t num_agents, size_t num_states);
    virtual ~IAgentBody() {}
    //! Resets for the next simulation call
    void reset();

    // State
    virtual matrix2d computeCongestionState(const std::list<UAV*> &UAVs) = 0;
    //! Stored agent states, [*step][agent][state]
    matrix3d agent_states_;
    size_t k_num_states_;

    // Actions
    //! Translates neural net output to link search costs
    virtual matrix1d actionsToWeights(matrix2d agent_actions) = 0;
    //! Stored agent actions, [*step][agent][action]
    matrix3d agent_actions_;
    //! Adds to agentActions
    void logAgentActions(matrix2d agentStepActions);
    //! Returns true if the last action was different.
    //! Used to prompt replanning.
    bool lastActionDifferent();
    //! Exports list of agent actions to a numbered file.
    void exportAgentActions(int fileID);


    // Reward
    double k_alpha_;
    bool k_square_reward_mode_;
};
#endif  // SRC_DOMAINS_UTM_IAGENTBODY_H_
