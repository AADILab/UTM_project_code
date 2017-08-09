#pragma once
#include "Domains/IDomainStateful.h"
#include "Multiagent/IMultiagentSystem.h"
#include "FileIO/FileIn.h"

/*
Instructions for using ISimulator:

1) declare child class of ISimulator
2) set Domain externally

*/
template <class Agent>
class ISimulator{
  public:
    typedef IMultiagentSystem<Agent> MultiagentSystem;
    ISimulator<Agent>(){};
	  ISimulator<Agent>(IDomainStateful* domain, MultiagentSystem* MAS): domain(domain), MAS(MAS) {}
    virtual ~ISimulator(void){}
	  IDomainStateful* domain;
	  MultiagentSystem* MAS;
	  virtual void runExperiment(void)=0; // run the experiment
	  void outputRewardLog(std::string reward_file){
		  cio::print(reward_log, reward_file);
	  }
    void outputMetricLog(std::string metric_filepath_full, int run = 0) {
      bool overwrite = (run == 0); // overwrite if the first run
      cio::print(metric_log, metric_filepath_full + ".csv", overwrite);
    }
	  std::vector<double> reward_log;
	  std::vector<double> metric_log;
};
