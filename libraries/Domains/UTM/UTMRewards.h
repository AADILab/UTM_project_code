#pragma once

class UTMGlobal {
  // Class for collecting global reward information
  public:
    UTMGlobal(int num_steps) { G = matrix1d(num_steps, 0.0); }
    double operator()() {
      return easymath::sum(G);
    }
    double& operator()(int t) {
      return G.at(t);
    }
    int getNumSteps() { return G.size(); };
  private:
    matrix1d G;
};

class UTMDifferenceApprox {
  public:
    UTMDifferenceApprox(UTMGlobal* G_set, int num_agents) {
      G = G_set;
      nn_state_history = matrix3d(G->getNumSteps(), matrix2d(num_agents));
    }
    UTMGlobal* G;
    matrix3d nn_state_history;
};
