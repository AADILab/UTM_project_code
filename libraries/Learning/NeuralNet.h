// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_LEARNING_INCLUDE_NEURALNET_H_
#define SRC_LEARNING_INCLUDE_NEURALNET_H_

#include <vector>
#include <string>
#include <iostream>
#include <random>
#include <ctime>
#include "Math/MatrixTypes.h"
#include "Math/easymath.h"
#include "FileIO/FileIn.h"
#include "IPolicy.h"
#include "yaml-cpp/yaml.h"

typedef matrix1d State;
typedef matrix1d Action;
typedef double Reward;

class NeuralNet : public IPolicy<State, Action, Reward> {
  public:
    // Creation/destruction
    NeuralNet(int id_num) : NNID(id_num), firstEvaluation(true), nn_state_(new BPState()) {}
    NeuralNet(YAML::Node configs, int id_num);
    virtual ~NeuralNet() {}

    bool sigmoid_output_;
    typedef Action Action;
    typedef State State;
    typedef Reward Reward;
    double backProp(const matrix1d &X, const matrix1d &Y);
    double backPropBatch(matrix2d X, matrix2d Y, int iterations);
    int getNumInputs() {
      return getTopology()[0];
    }

    Action operator()(State s) const;
    bool isFirstEvaluation() { return firstEvaluation; };

    Reward getEvaluation() const { return evaluation_; }
    void save(std::string fileout) const;

    void update(Reward R) {
      evaluation_ = R;
      firstEvaluation = false;
    }
    void mutate(int new_id);  // different if child class. Change id when mutating.
    void load(std::string file_in);
    void load(matrix1d node_info, matrix1d wt_info);
	  /** Find minimum and maximum weights in network. */
	  void updateMinMax();
	  
	  /** Get minimum and maximum weights in network 
	  *  \return minimum and maximum weight values, as std::pair
	  */
	  std::pair<double, double> getWtRange();

    int NNID;
	
	  struct Layer {
		  size_t k_num_nodes_above_, k_num_nodes_below_;
		  matrix2d w_bar_, w_;
		  Layer(size_t above, size_t below);
		  Layer(const Layer& l) :Layer(l.k_num_nodes_above_, l.k_num_nodes_below_) {
			  w_bar_ = l.w_bar_;
			  w_ = l.w_;
		  }
		  Layer& operator=(const Layer& l) {
			  k_num_nodes_above_ = l.k_num_nodes_above_;
			  k_num_nodes_below_ = l.k_num_nodes_below_;
			  w_bar_ = l.w_bar_;
			  w_ = l.w_;
			  return *this;
		  }
	  };

	  std::vector<Layer> layers_;
  protected:
    
    std::normal_distribution<double> normal_;
    
    bool firstEvaluation;

	  double min_w_; /**< minimum weight value */
	  double max_w_; /**< maximum weight value */

    size_t N;
    double k_eta_;

    // Changing elements (backprop)
    struct BPState {
      BPState() {
        s = matrix2d(2);
        z = matrix2d(2);
        f = matrix2d(2);
      }
      matrix2d s, z, f, d;
    };

  private:
    // Life cycle
    BPState* nn_state_;

    matrix1d fwrdProp(matrix1d X, BPState* S);
    void backProp(matrix1d Y, BPState* S);
    void updateWt(matrix1d X, const BPState& S);

    // Mutators
    std::default_random_engine generator;

    // Accessors
    double evaluation_;
    double k_gamma_;
    double k_mut_std_;      //! mutation standard deviation
    double k_mut_rate_;     //! probability that each connection is changed

    matrix1d getTopology() const;
    double mutateAmount();

};

#endif  // SRC_LEARNING_INCLUDE_NEURALNET_H_
