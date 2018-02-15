// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_NEUROEVO_H_
#define SRC_LEARNING_INCLUDE_NEUROEVO_H_

#include <string>
#include <vector>
#include <algorithm>
#include <list>

#include "NeuralNet.h"
#include "Evolution.h"
#include "FileIO/FileIn.h"

class NeuroEvo : public Evolution<NeuralNet> {
  public:
    //! Life cycle
     NeuroEvo(YAML::Node configs, bool active);
    ~NeuroEvo(void) { deletePopulation(); }
    void deepCopy(const NeuroEvo &NE);
    void deletePopulation();
    bool active_;

	  size_t getCurrentMemberId();

    //! Mutators
    void generateNewMembers();
    bool selectNewMember();
    void selectSurvivors(bool sh = true);
    void updatePolicyValues(double R);
    void load(std::string filein);

    double getSurvivorPersistence() {
      std::vector<int> pop_new(population_.size());
      int i = 0;
      for (auto p: population_) {
        pop_new[i++] = p->NNID;
      }
      std::sort(pop_new.begin(), pop_new.end());
      std::sort(pop_old.begin(), pop_old.end());
      std::vector<int> v(pop_old.size());

      auto it = std::set_intersection(pop_new.begin(), pop_new.end(), pop_old.begin(), pop_old.end(), v.begin());
      v.resize(it - v.begin());
      pop_old = pop_new;

      return double(v.size())/double(population_.size()); // maximum 50% kept; so multiply by 2
    }
    std::vector<int> pop_old;
    std::vector<int> pop_new;
    int nn_generated; // number of neural networks generated so far
	
    /** Calculates mean squared difference between parent and child NNs
    *  \param p parent network
    *  \param c child network
    *  \return mean squared difference of weights
    */
  	double find_msd(NeuralNet *p, NeuralNet *c);

	  /** Get weight ranges for all networks in population, store in ranges_ */
	  void recordWtRange();

    /** Return mean squared difference in weights for each parent-child pair. */
    matrix1d get_msds()
    {
	    matrix1d output = pcmsd_;
	    pcmsd_.clear();
	    return output;
    }

	  /** Return ranges of weights for all survivor networks in population */
	  matrix2d getWtRanges()
	  {
		  matrix2d output = ranges_;
		  ranges_.clear();
		  return output;
	  }

    /** Get fitness of each network in population
    *  \return 2d matrix of fitness values. First column gives ID of network, second column gives fitness
    */
	  matrix2d getPopEval()
	  {
		  matrix2d output;
		  std::vector<int> ids;
		
		  population_.sort(NNCompare2);
		  for (auto pop : population_)
		  {
			  matrix1d temp;
			  temp.push_back(pop->NNID);
			  temp.push_back(pop->getEvaluation());
			  output.push_back(temp);
		  }
		  return output;
	  }

    //! Accessors
    double getBestMemberVal();
    static bool NNCompare(const NeuralNet *x, const NeuralNet *y) {
      return (x->getEvaluation() > y->getEvaluation());
    }
	  static bool NNCompare2(const NeuralNet *x, const NeuralNet *y) {
		  return (x->NNID > y->NNID);
	  }
    Action getAction(State state);
    Action getAction(std::vector<State> state);
    void save(std::string fileout);

  private:
    size_t k_population_size_;
	  matrix1d pcmsd_; /**< Parent-child mean squared difference across members in pop */
	  matrix2d ranges_; /**< min and max weights for all networks in pop */
};
#endif  // SRC_LEARNING_INCLUDE_NEUROEVO_H_
