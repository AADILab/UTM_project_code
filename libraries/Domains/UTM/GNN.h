#ifndef SRC_DOMAINS_UTM_GNN_H_
#define SRC_DOMAINS_UTM_GNN_H_

// Copyright 2017 Carrie Rebhuhn
// A neural network approximator for G in the UTM domain.
// This takes in stepwise information and outputs a G prediction.
// Several child classes, take different inputs.


#include "Simulation/SimTime.h"
#include "Learning/NeuralNet.h"
#include "Link.h"

#define UNSET -1.0

class GNN {
  public:
    GNN(YAML::Node configs, SimTime* simtime, int id_num): NN(NeuralNet(configs, id_num)) {
      T = simtime;
      x = matrix1d(NN.getNumInputs(), 0.0);
      X = matrix2d(T->MAX_STEP, matrix1d());
    }

    void update(double a, Link * l) {
      setLinkState(l);
      setActionState(a);
    }
    void train(double a, Link* l, double G) {
      update(a, l);

      matrix1d y(1, G);
      NN.backProp(x, y);
      X[T->step] = x;  // record the state history for later rollout
    }

    double estimate() {
      double G = 0.0;
      for (auto xt : X) {
        G += NN.operator()(xt)[0];
      }
      return G;
    }

  protected:
    virtual void setActionState(double action) = 0;
    void setLinkState(Link* l) {
      x[0] = l->countTraffic();
      x[1] = l->countIncomingTraffic();
    }
    matrix1d x;

  private:
    NeuralNet NN;
    double estimate(matrix1d val) {
      return NN.operator()(val)[0];
    }
    matrix2d X;  // all state history
    SimTime* T;
};

// Uses the current cost in the neural network state
class GNNCurrent : public GNN {
  public:
    GNNCurrent(YAML::Node configs, SimTime* T, int id_num) : GNN(configs, T, id_num) {}
  private:
    enum StateElements { TRAFFIC, INCOMING, COST_T, NUMSTATES }; // note: now disjoint, may not match config file
    void setActionState(double a) { x[COST_T] = a; }
};

// Uses the previous cost
class GNNPast : public GNN {
  public:
    GNNPast(YAML::Node configs, SimTime* T, int id_num) : GNN(configs, T, id_num), last_a(UNSET) {}
  private:
    enum StateElements { TRAFFIC, INCOMING, COST_TMINUS1, NUMSTATES };
    void setActionState(double a) {
      if (last_a == UNSET)
          last_a = a;
      x[COST_TMINUS1] = last_a;
    }
    double last_a;
};

// Uses a combination of state elements
class GNNCombined : public GNN {
  public:
    GNNCombined(YAML::Node configs, SimTime* T, int id_num) : GNN(configs, T, id_num), last_a(UNSET) {}
  private:
    enum StateElements { TRAFFIC, INCOMING, COST_T, COST_TMINUS1, NUMSTATES };
    void setActionState(double a) {
      if (last_a == UNSET)
          last_a = a;
      x[COST_T] = a;
      x[COST_TMINUS1] = last_a;
    }
    double last_a;
};


// Uses the difference between a last action and the current action
// as a state element
class GNNDelta : public GNN {
  public:
    GNNDelta(YAML::Node configs, SimTime* T, int id_num) : GNN(configs, T, id_num), last_a(UNSET) {}
  private:
    enum StateElements { TRAFFIC, INCOMING, COST_DELTA, NUMSTATES };
    void setActionState(double a) {
      if (last_a == UNSET) {
        last_a = a;
      }
      x[COST_DELTA] = a - last_a;
    }
    double last_a;
};

#endif  // SRC_DOMAINS_UTM_GNN_H_
