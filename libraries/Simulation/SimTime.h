#pragma once
#include <yaml-cpp/yaml.h>

class SimTime {
    // Tracker for simulation synchronicity
    // Also keeps track of max times
    // Reads in a yaml to load these max times

  public:
    SimTime(YAML::Node configs) {
      MAX_EPOCH = configs["time"]["epochs"].as<size_t>();
      MAX_RUN = configs["time"]["runs"].as<size_t>();
      MAX_STEP = configs["time"]["steps"].as<size_t>();
      MAX_TRIAL = configs["time"]["trials"].as<size_t>();
      reset();
    }
    void reset() {
      run = 0;
      epoch = 0;
      trial = 0;
      step = 0;
      eval = 0;
    }

    bool iterateSteps() {
      step++;
      if (step < MAX_STEP)
        return true;
      else
        return false;
    }
    bool iterateEpochs() {
      epoch++;
      if (epoch < MAX_EPOCH)
        return true;
      else
        return false;
    }

	  bool iterateRuns() {
		  run++;
		  if (run < MAX_RUN)
			  return true;
		  else
			  return false;
	  }

    size_t MAX_EPOCH, MAX_TRIAL, MAX_STEP, MAX_RUN;
    size_t epoch, eval, trial, step, run;
};
