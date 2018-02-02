// Copyright 2016 Carrie Rebhuhn
// AbstractUTMSimulation.cpp : Defines the entry point for the console
// application.
//

#include <iostream>
#include <string>
#include <stdlib.h>

// Project-specific includes
#include "Simulation/SimNE.h"
#include "Domains/UTM/UTMDomainAbstract.h"
#include "Impactfulness.h"

using std::vector;
std::string config_file = "config.yaml";

void print_nets(MultiagentNE* MAS, size_t run) {
  for (size_t i = 0; i < MAS->agents.size(); i++) {
    std::string file_out = "neural_nets/net_" + std::to_string(run) + "_" + std::to_string(i) + ".csv";
    MAS->agents.at(i)->save(file_out);
  }
}

void abstractUtmSimulation(std::string fitness) {
	// Read some parameters from config file
	std::cout << "Loading config file...\n" ;
  YAML::Node configs = YAML::LoadFile(config_file);
  std::cout << "Loaded!\n" ;
	auto e = configs["time"]["epochs"].as<int>();
	auto runs = configs["time"]["runs"].as<int>();
	auto state_rep = configs["modes"]["state"].as<std::string>();
	
	// Check tracking configuration
  bool tracking = configs["modes"]["tracking"].as<bool>();
  bool track_all = false ;
  if (tracking){
    auto track_type = configs["modes"]["tracking_type"].as<std::string>() ;
    if (track_type == "all"){
      std::string all_track ;
      std::cout << "WARNING: configuration set to store all simulation logs! This may result in the generation of a very large number of log files. Do you wish to continue [y/N]? " ;
      std::getline(std::cin, all_track) ;
      if (all_track == "y" || all_track == "Y")
        track_all = true ;
      else{
        std::cout << "Change configuration by updating {modes/tracking} to {false} OR {modes/tracking_type} to {first_last} in the config.yaml.\nExiting!\n" ; 
        exit(EXIT_FAILURE) ;
      }
    }
  }

	std::cout << "Creating UTM domain...\n" ;
	// Create UTM Domain
  UTMDomainAbstract* domain = new UTMDomainAbstract(configs);
  std::cout << "Created!\n" ;
  
	std::cout << "Creating MAS...\n" ;
  // Load MAS
	MultiagentNE* MAS = new MultiagentNE(configs, domain);
  char buffer[50] ;
  std::cout << "Created!\n" ;
		
	std::cout << "Creating simulator...\n" ;
	// Create a simulator
	SimNE sim(domain, MAS, configs, "This string does nothing right now");
	sim.fitness_metric = fitness;
  std::cout << "Created!\n" ;
  
	// Start the runs
	for (int r = 0; r < runs; r++)
	{
	  for (size_t i = 0; i < domain->getNumAgents(); i++){
      sprintf(buffer,"../../neural_nets/net_%i_%i.csv",r,i) ;
      MAS->agents[i]->load(buffer) ;
	  }
	  srand(r+1); // increment random seed
	  if (tracking){ // log simulation data for all runs if track_all = true, only log for first stat run if track_all = false
	    if (!track_all && r > 0){
	      domain->ChangeTrackingStatus(false) ;
	      sim.ChangeTrackingStatus(false) ;
      }
      else{
        domain->ChangeTrackingStatus(true) ;
        sim.ChangeTrackingStatus(true) ;
      }
    }
		sim.runExperimentTest();
		
		// Output performance, one file for each run
		std::string metrics_dir = domain->getOutputMetricsDirectory() ;
		sim.outputMetricLog(metrics_dir + "global_" + state_rep + "_" + std::to_string(e) + "_" + std::to_string(r));
		// Output extra stuff: delay, moving time, and wait time
		sim.outputExtraMetrics(metrics_dir);
		// Increment run in domain, reset epoch counter
		domain->T->run++;
		domain->T->epoch = 0;
		
	}
  
	delete MAS;

  delete domain;
}


void impactfulness() {
  //YAML::Node configs = YAML::Load(config_file);
  //UTMDomainAbstract* domain = new UTMDomainAbstract(configs);

  int sectors = 11;
  int numperms = 1;
  for (int i = 0; i < numperms; i++) {
    std::string imp_string = "Domains\/"+ std::to_string(sectors) + "_Sectors\/" ;
    Impactfulness imp(imp_string);
    imp.calculate();
  }
}


int main() {

  // Run impactfulness testing for link agents
//  impactfulness();
  
  // Run abstract UTM simulation given configuration stored in config.yaml
  abstractUtmSimulation("global");
  
  return 0;
}
