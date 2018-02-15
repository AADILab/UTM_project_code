//! Copyright 2016 Carrie Rebhuhn
#include "NeuroEvo.h"

using std::list;
using std::vector;

NeuroEvo::NeuroEvo(YAML::Node configs, bool active) {
  active_ = active;
  k_population_size_ = configs["neuroevo"]["popsize"].as<size_t>();

  nn_generated = 0;
  for (size_t i = 0; i <k_population_size_; i++) {
    NeuralNet* nn = new NeuralNet(configs["neuroevo"], nn_generated++);
    population_.push_back(nn);
  }
  pop_member_active_ = population_.begin();
  
  pop_old = std::vector<int>(population_.size());
  int i = 0;
  for (auto p : population_) {
    pop_old[i++] = p->NNID;
  }
}

void NeuroEvo::updatePolicyValues(double R) {
  // Add together xi values, for averaging
//  double xi = 0.1;  // "learning rate" for NE
  // Edit JJC: suspicious of above equation introducing hysteresis into learning, removing it.
  double xi = 1.0 ;

  if ((*pop_member_active_)->isFirstEvaluation()) {
    xi = 1.0; // don't take initial (zero) value into account
  }

  double V = (*pop_member_active_)->getEvaluation();
  V = xi*(R - V) + V;
  (*pop_member_active_)->update(V);
}

NeuroEvo::Action NeuroEvo::getAction(NeuroEvo::State state) {
	if (active_)
    return (**pop_member_active_)(state);
  else
    return Action(1, 1.0);
}

NeuroEvo::Action NeuroEvo::getAction(std::vector<NeuroEvo::State> state) {
  State stateSum(state[0].size(), 0.0);

  // state[type][state_element] -- specifies combination for state
  for (size_t i = 0; i < state.size(); i++) {
    for (size_t j = 0; j < state[i].size(); j++) {
      stateSum[j] += state[i][j];
    }
  }

  return getAction(stateSum);
}

void NeuroEvo::deletePopulation() {
  while (!population_.empty()) {
    delete population_.back();
    population_.pop_back();
  }
}

bool NeuroEvo::selectNewMember() {
  ++pop_member_active_;
  if (pop_member_active_ == population_.end()) {
    pop_member_active_ = population_.begin();
    return false;
  } else {
    return true;
  }
}

void NeuroEvo::generateNewMembers() {
  // Mutate existing members to generate more
  list<NeuralNet*>::iterator popMember = population_.begin();
  for (size_t i = 0; i < k_population_size_; i++) {  // add new members
    // dereference pointer AND iterator
    NeuralNet* m = new NeuralNet(**popMember);
    m->mutate(nn_generated++);
  	pcmsd_.push_back(find_msd(*popMember, m));
    population_.push_back(m);
    ++popMember;
  }
}

double NeuroEvo::getBestMemberVal() {
  // Find the HIGHEST FITNESS value of any neural network
  double highest = population_.front()->getEvaluation();
  for (NeuralNet* p : population_) {
    if (highest < p->getEvaluation()) highest = p->getEvaluation();
  }
  return highest;
}

void random_shuffle(list<NeuralNet*> *L) {
  vector<NeuralNet*> tmp(L->begin(), L->end());
  random_shuffle(tmp.begin(), tmp.end());
  copy(tmp.begin(), tmp.end(), L->begin());
}

void NeuroEvo::selectSurvivors(bool sh) {
  // Select neural networks with the HIGHEST FITNESS
  population_.sort(NNCompare);  // Sort by the highest fitness
  size_t nExtraNN = population_.size() - k_population_size_;
  for (size_t i = 0; i < nExtraNN; i++) {  // Remove the extra
    delete population_.back();
    population_.pop_back();
  }
  if (sh) // don't shuffle on the last epoch
    random_shuffle(&population_);

  pop_new.clear();
  for (auto pop : population_) {
    pop_new.push_back(pop->NNID);
  }

  pop_member_active_ = population_.begin();

	recordWtRange();
}

void NeuroEvo::deepCopy(const NeuroEvo &NE) {
  // Creates new pointer addresses for the neural nets
  k_population_size_ = NE.k_population_size_;

  deletePopulation();
  for (NeuralNet* p : NE.population_) {
    population_.push_back(new NeuralNet(*p));
  }
}


void NeuroEvo::save(std::string fileout) {
  for (NeuralNet* p : population_) {
    p->save(fileout);
  }
}

void NeuroEvo::load(std::string filein) {
  matrix2d netinfo = cio::read2<double>(filein);

  int i = 0;
  for (NeuralNet* p : population_) {
    // assume that population_ already has the correct size
    p->load(netinfo[i], netinfo[i + 1]);
    i += 2;
  }
}

double NeuroEvo::find_msd(NeuralNet *p, NeuralNet *c) {
	matrix1d pweights;
	matrix1d cweights;
	for (NeuralNet::Layer &l : p->layers_) {
		for (matrix1d &wt_outer : l.w_bar_) {
			for (double &w : wt_outer) {
				pweights.push_back(w);
			}
		}
	}
	for (NeuralNet::Layer &l : c->layers_) {
		for (matrix1d &wt_outer : l.w_bar_) {
			for (double &w : wt_outer) {
				cweights.push_back(w);
			}
		}
	}

	// mean squared differencee
	double msd = 0;
	for (int i = 0; i < pweights.size(); i++)
	{
		double diff = pweights[i] - cweights[i];
		msd += diff*diff;
	}
	msd /= (double)pweights.size();

	return msd;
}

void NeuroEvo::recordWtRange() {
	for (auto pop : population_) {
		matrix1d temp;
		std::pair<double, double> range = pop->getWtRange();
		temp.push_back(range.first);
		temp.push_back(range.second);
		ranges_.push_back(temp);
	}
}

size_t NeuroEvo::getCurrentMemberId() {
	return (*pop_member_active_)->NNID;
}
