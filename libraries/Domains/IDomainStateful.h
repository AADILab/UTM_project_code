// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_IDOMAINSTATEFUL_H_
#define SRC_DOMAINS_IDOMAINSTATEFUL_H_

#include <vector>
#include <string>
#include "yaml-cpp/yaml.h"
#include "Simulation/SimTime.h"

typedef std::vector<double> matrix1d;
typedef std::vector<matrix1d> matrix2d;
typedef std::vector<matrix2d> matrix3d;

class IDomainStateful {
  public:
    typedef matrix1d State;
    typedef double Reward;
    IDomainStateful() {}

    double max_action, min_action;
    void printMaxMinAction() {
     std::printf("Max/min: %f/%f\n", max_action, min_action);
    }
    SimTime* T;
    virtual void initializeBase(YAML::Node configs) = 0;
    virtual ~IDomainStateful(void) { }

    #ifdef ROS_ROOT
    virtual std::vector<StateActionSequence> getStateActionSequences()=0;
    virtual std::vector<StateActionSequence> getStateActionSequencesAlternate()=0;

    virtual void storeBestActions() = 0;
    virtual void outputBestActions() = 0;
    #endif
    // Returns the state vector for the set of agents, [AGENTID][STATEELEMENT]
    virtual matrix2d getStates() = 0;

    //! Returns the reward vector for a set of agents [AGENTID]
    virtual std::vector<Reward> getRewards() = 0;

    //! Returns the performance (usually G) for a set of agents
    virtual Reward getPerformance() = 0;
    virtual double simulateStep(matrix2d agent_action) = 0;
    virtual Reward simulateSteps(matrix3d agent_actions) = 0;
    virtual void reset() = 0;
    virtual void logStep() = 0;
    virtual void printTrackingInfo() = 0;
    virtual void printUavGen() = 0;
    virtual matrix1d getExtraInfo() = 0;
    virtual matrix2d getExtraExtraInfo() = 0;

    virtual matrix2d stateHistory() = 0;
    virtual matrix2d actionHistory() = 0;

    //! Creates a directory for the current domain's parameters
    virtual std::string createExperimentDirectory(std::string config_file) = 0;

    //size_t getNumSteps() const { return k_num_steps_; }
    size_t getNumAgents() const { return k_num_agents_; }
    size_t getNumNNInputs() const { return k_num_states_; }
    size_t getNumNNOutputs() const { return k_num_actions_; }
    
  protected:
    size_t k_num_states_, k_num_actions_, k_num_agents_;   // agents determined later!
};

#endif  // SRC_DOMAINS_IDOMAINSTATEFUL_H_
