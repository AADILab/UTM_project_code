// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_EVOLUTION_H_
#define SRC_LEARNING_INCLUDE_EVOLUTION_H_

// This is a template class; thereofore it has no cpp

#include <list>
#include "IAgent.h"
#include "STL/easystl.h"

template <class Policy>
class Evolution : public IAgent<Policy> {
  public:
    //! Life cycle
    Evolution() {}
    Evolution(const Evolution &E) {
      population_ = new Population();
      for (Policy* p : E.population)
        population_->push_back(new Policy(*p));
      PopulationMember* pop_member_active_ = new PopulationMember(population_->begin());
    }
    ~Evolution() { easystl::clear(population_); }

    //! Mutators
    virtual void generateNewMembers() = 0;
    virtual void activateNextMember() {
      pop_member_active_++;
      this->setPolicy(*pop_member_active_);
    }
    virtual void selectSurvivors() = 0;
    void update(const Reward &rwd) { this->policy->update(rwd); }
    void setFirstMember() {
      pop_member_active_ = this->population_->begin();
      this->policy = *pop_member_active_;
    }

    //! Accessor
    bool atLastMember() const {
      return *this->pop_member_active_ == this->population_->end();
    }

  protected:
    typedef std::list<Policy*> Population;
    typedef typename std::list<Policy*>::iterator PopulationMember;
    Population population_;
    PopulationMember pop_member_active_;
};

#endif  // SRC_LEARNING_INCLUDE_EVOLUTION_H_
