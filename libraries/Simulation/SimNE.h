// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_SIMULATION_INCLUDE_SIMNE_H_
#define SRC_SIMULATION_INCLUDE_SIMNE_H_

// C
#include <time.h>
#include <stdio.h>

// C++
#include <vector>
#include <limits>
#include <iostream>

// Libraries
#include "ISimulator.h"
#include "Multiagent/MultiagentNE.h"
#include "SimTime.h"

class Timer {
public:
  Timer() {}
  void start() {
    t_start = time(NULL);
  }
  void stop() {
    t_stop = time(NULL);
  }
  void printTimePassed() {
    time_t delta_t = t_stop - t_start;
    printf("Time passed: %i seconds. ", static_cast<int>(delta_t));
  }
  void printEndEstimate(int num_left) {
    time_t delta_t = t_stop - t_start;
    time_t run_time_left = time_t(num_left)*delta_t;
    time_t run_end_time = t_stop + run_time_left;
#ifdef _WIN32
    char end_clock_time[26];
    ctime_s(end_clock_time, sizeof(end_clock_time), &run_end_time);
    printf("Estimated end: %s. ", end_clock_time);
#else
    time(&run_end_time) ;
    struct tm * time_info = localtime(&run_end_time) ;
    printf("Estimated end: %s. ", asctime(time_info));
#endif
  }

private:
  time_t t_start;
  time_t t_stop;
};

class SimNE : public ISimulator<NeuroEvo> {
public:
	void print_nets(MultiagentNE* MAS, size_t run);

  SimNE(IDomainStateful* domain, MultiagentNE* MAS, YAML::Node configs, std::string metric_filepath);
  SimNE() {}
  MultiagentNE* MAS;
  Timer timer;
  ~SimNE() {}
  
  SimTime* T; /**< grabbed from domain, for synching */

  // Set from yaml file
  std::string fitness_metric;
	bool tracking_enabled; // config: whether or not to output files for unit testing
	bool tracking_all_enabled ; // config: whether to output all epoch logs or only from first and last epochs
  bool tracking ; // output log files
	bool record_enabled; /**< spawn UAVs according to file */
	std::string objective; 
    
  void ChangeTrackingStatus(bool tracking_on){ // used for disabling tracking if modes/tracking_type: first_last
    tracking_enabled = tracking_on ;
  }

  std::vector<std::vector<double> > survivor_persistence;
	
	/* A bunch of stuff for unit testing */
	matrix3d mutation_difference; /**< The mean squared difference in parent-child pairs.
								       Contains a 2D matrix for each team of agents, 
									   where each row contains msds for one agent's networks */
	std::vector<matrix3d> weight_ranges; /**< Get ranges of weights for all pop members for all agents (each vector contains values for one team)
											  Each primary index indexes a 2D matrix which contains, 
										      for each row (network), the min and max weights. */
	std::vector<matrix3d> evaluations; 	/**< Fitness values for all pop members for all agents (each vector contains values for one team)
										     Each secondary index indexes a 2D matrix which contains,
										     for each row (network), the ID of the network and its fitness. */
	matrix2d extra_metric_log; /**< For each row, contains delay time, moving time, wait time, and num UAVs generated of best team. One row is for one epoch. */
	matrix3d extra_extra_metric_log; /**< LAST MINUTE ADDITION, probably good to ignore */

  struct SimHistory {
    SimHistory() {}
    SimHistory(size_t T, size_t N) :
      N(N), T(T) {
      A = matrix3d(N, matrix2d(T));
      S = matrix3d(N, matrix2d(T));
      G = matrix1d(T);

      E = matrix2d(T);
      EE = matrix2d(T);
    }
    void addState(matrix2d s, int t) {
      for (size_t i = 0; i < N; i++) {
        S[i][t] = s[i];
      }
    }
    void addAction(matrix2d a, int t) {
      for (size_t i = 0; i < N; i++) {
        A[i][t] = a[i];
      }
    }
    matrix1d G;
    matrix2d E, EE; // E is for Extra!// EE is for Extra Extra!
    matrix3d S, A; 
    size_t N, T;
  };

  SimHistory simGlobal();
  std::pair<matrix1d, double> simDifferenceResimA();
  std::pair<matrix1d, double> simDifferenceResimP();
  std::pair<matrix1d, double> simDifferenceApprox(std::vector<NeuralNet>* DNN);

  /** Gets actions based on current state: OVERLOAD FOR TYPES */
  virtual std::vector<Action> getActions(std::vector<State>);

	/** Mutates populations and runs a simulation for each team of agents. */
  virtual void epoch();
	
	/** Starts a single run of training */
  virtual void runExperiment();

  struct accounting {
    accounting();
    bool update(const std::vector<double> &R, const double &perf, int n);
    /** Once we update best team performance (as above), we get corresponding extra info
     *  \param extra includes delay time, moving time, wait time for each step, as well as num UAVs
     *  \param extraExtra don't worry about it
     *  \param eval Huh, turns out this isn't even used.
     */
	  void update_extra(const matrix2d &extra, const matrix2d &extraExtra, size_t eval);
    int n, best_perf_idx;
    double best_run, best_run_performance;
	  bool best_update; /**< Flag that says we updated, next need to record extra info */
	  matrix1d best_extra; /** extra info of best performing team */
	  matrix2d best_extra_extra; 
  };

  /** For unit testing purposes. 
  *  Print to file mean squared difference between parent-child pairs, 
  *  as well as ranges of weights of the learning agents 
  */
	void outputLearningLog(std::string filename);
	
  /** For unit testing purposes.
  *  Print to file the fitness values of the learning agents.
  */
	void outputEvalLog(std::string filename);
	void outputExtraMetrics(std::string filename = "");
	
private:
	std::string metric_filepath;
};

#endif  // SRC_SIMULATION_INCLUDE_SIMNE_H_
