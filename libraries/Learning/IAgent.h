// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_IAGENT_H_
#define SRC_LEARNING_INCLUDE_IAGENT_H_

#include "IPolicy.h"

template<class Policy>
class IAgent {
  public:
    typedef typename Policy::State State;
    typedef typename Policy::Action Action;
    typedef typename Policy::Reward Reward;

    //! Life cycle
    IAgent(void) {}
    virtual ~IAgent(void) {}

    //! Accessors
    Action getAction(State state) {
      return (*policy_)(state);
    }

    //! Mutators
    void update(Reward R) { policy_->update(R); }
    void setPolicy(Policy* p) { policy_ = p; }

  private:
    Policy* policy_;
};

#endif  // SRC_LEARNING_INCLUDE_IAGENT_H_
