// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_
#define SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_

#include <iostream>
#include <utility>
#include <map>
#include <string>
#include <list>
#include <vector>
#include <float.h>

#include "Domains/IDomainStateful.h"
#include "Planning/LinkGraph.h"
#include "yaml-cpp/yaml.h"
#include "Simulation/SimTime.h"
#include "FileIO/FileIn.h"
#include "Math/easymath.h"
#include "STL/easystl.h"
#include "IAgentBody.h"
#include "GNN.h"
#include "Link.h"
#include "Sector.h"
#include "SectorAgent.h"
#include "UTMFileNames.h"
#include "UTMTracker.h"
#include "UTMRewards.h"

class UTMDomainAbstract : public IDomainStateful {
public:
  /**
  * Constructor for UTM Domain, which loads in a config file and creates domain based on parameters
  * \param configs the root YAML node of the config file
  * \param costmode ask Carrie Rebhuhn about this if necessary
  */
  explicit UTMDomainAbstract(YAML::Node configs, std::string costmode = "current");
  UTMDomainAbstract(YAML::Node configs, std::string costmode, bool only_abstract);
  ~UTMDomainAbstract(void);

  UTMTracker* tracker; /**< Pointer to the UTMTracker instance used to record tracking info */
  UTMDifferenceApprox* d_approx;
  UTMGlobal* G;
  // Saved variables and accessing
  //matrix2d weights_saved;
  //matrix2d states_saved;
  //matrix3d nn_states_saved;
    
  void ChangeTrackingStatus(bool tracking_on){ // used for disabling tracking if modes/tracking_type: first_last
    tracking = tracking_on ;
  }

  matrix2d stateHistory() { return tracker->getStates(); }
  matrix2d actionHistory() { return tracker->getActions(); }
  matrix2d trafficGenerated;

  // Modes/simulation parameters
  std::string k_generation_mode_; /**< "all" - UAVs generated at all sectors, "list" - UAVs generated at specific sectors */
  std::string k_destination_mode_;/**< "all" - UAVs' goals can be set to any sector, "list" - UAVs' goals can be set only to specific sectors */
  std::vector<size_t> generation_sectors; /**< sectors where UAVs may be generated */
  std::vector<size_t> destination_sectors; /**< goal sectors for the UAVs */
  std::string k_reward_mode_;
  bool k_extra_info_; /**< If we're recording extra information or not (delays, moving time, ground hold time) */
  std::string k_agent_type_; // JJC: for comparative experiments
  std::string k_travel_time_; // JJC: for listed distances

  // Reward
  std::vector<GNN*> G_t; /**< Predicted timestep G */

  void printTrackingInfo(); /**< Print to file information about state, actions, and uavs at each time step */
  void printUavGen(); /**< Print out number of UAVs generated from each sector. Used for debugging purposes. */
  matrix1d getExtraInfo(); /**< Retrieve other information such as delay, moving time, ground hold time */
  matrix2d getExtraExtraInfo(); /**< Retrieve information, or each link, on how many times its UAVs are delayed as
						           well as how many times the link denies a UAV entry because of full capacity. NOT USED */
  std::string getOutputMetricsDirectory(){return metrics_dir ;}
  
//  size_t getTotalNumberUAVs(){return uav_count ;} // Return the total number of UAVs up to the current timestep
  size_t getTotalNumberUAVs(){return mission_count ;} // Return the total number of assigned missions up to the current timestep
protected:
	//bool resetUavNums = true; 

  //LOGGING
  // matrix1d G; // G at each timestep
  // matrix2d actions_saved;
  // matrix2d link_uavs_;    // The number of UAVs on each link, [step][linkID]
  // matrix2d sector_uavs_;  // The number of UAVs waiting at a sector,
                             // [step][sectorID]
 
    
  typedef std::pair<size_t, size_t> edge;
  IAgentBody* agents_;
  LinkGraph *high_graph_;
  std::map<edge, size_t> *k_link_ids_;
  std::vector<Link*> links_;

  std::string metrics_dir ; // directory for storing traffic metrics
	std::string k_state_rep_; /**< Which state representation the agents use. For link agents,
							       this is: "traffic" state - link agents only have info about traffic on own edge,
								   or "incoming" state - link agents have "traffic" state plus info about traffic
								   on incoming links */
  size_t k_num_sectors_; /**< How many sectors (nodes) in the domain */
  std::string k_position_mode_;
	bool k_record_; /**< Whether or not to record where and when UAVs are generated. 
						Most useful for probabilistic UAV generation. */
	size_t k_window_size_; /**< Window size used by link agents, only applies when "incoming" state is used. */
	std::string k_window_mode_; /**< "cumulative" means link agents keep historical information all 
								     throughout a simulation run (window_size == simulation steps);
									 "variable" means link agents have a specified window size */
	std::string k_capacity_mode_; /**< How each link agent's capacity is determined.
								       "flat" means capacity is the same for all links (specified in config file);
									   "list" means capacity may be different for each link, and is determined from
									   a file */
	//matrix1d num_uavs_at_sector_;
  std::string k_objective_mode_, /**< Objective that the agents try to learn to optimize */
	k_agent_mode_, /**< "link" - agents are links and control edge costs;
				        "sector" - agents are sectors and control cost to travel in a particular direction */
	k_disposal_mode_; /**< Whether or not UAVs get absorbed once they reach their goals ("trash"), or if they remain ("keep") */
  std::vector<Sector*> sectors_;
  std::vector<std::vector<int> > poses;
  std::list<UAV*> uavs_; /**< All UAVs currently in the domain */
  std::map<int, std::list<int> > k_incoming_links_; 
//  size_t uav_count; /**< Current total number of UAVs generated */
  size_t mission_count ; /*Current total number of assigned missions*/
  size_t completed_count ; /*Current total number of completed missions*/
	int delays, /**< total delay time experienced by all UAVs */
	moving, /**< total time UAVs were moving */
	ground_hold; /**< total time experienced by UAVs before traveling first link */
    
  double simulateSteps(matrix3d C) {
    // Simulates all of the steps, as dictated by the premade agent actions

    // reshape this
    matrix3d Calt(C[0].size(), matrix2d(C.size()));
    for (size_t i = 0; i < C.size(); i++) {
        for (size_t j = 0; j < C[i].size(); j++) {
            Calt[j][i] = C[i][j];
        }
    }

    for (T->step = 0; T->step < T->MAX_STEP; T->step++){
        simulateStep(Calt[T->step]);
    }

    return getPerformance();
  }
  
  /** Initializes some important variables
  *  \param configs root node of config file
  */
  void initializeBase(YAML::Node configs) {
    T = new SimTime(configs);
    
    std::string a = configs["modes"]["agent"].as<std::string>();
    std::string d = UTMFileNames::createDomainDirectory(configs);

    k_state_rep_ = configs["modes"]["state"].as<std::string>();

    if (a == "link") {
      k_num_actions_ = 1;
	    if (k_state_rep_ == "incoming")
		    k_num_states_ = 2; // traffic and potential incoming traffic
	    else
		    k_num_states_ = 1; // traffic alone
		    
      auto efile = d + "edges.csv";
      auto edges = cio::read2<double>(efile);
      k_num_agents_ = edges.size();
    } else if (a == "sector") {
      k_num_actions_ = 4;
      k_num_states_ = 4;
      auto vfile = d + "nodes.csv";
      auto nodes = cio::read2<double>(vfile);
      k_num_agents_ = nodes.size();
    } else {
      std::printf("Error with agent definition.");
      exit(1);
    }

  }
  
  /** Attempts to add a UAV to the next link in its path.
  *  \param u the UAV to add to the link
  *	\return true if UAV successfully added, false if otherwise
  */
  bool addUav(UAV* u) {
    u->setCurEdgeFromPath();
    if (getNthLink(u, 0)->add(u)) {
      uavs_.push_back(u);
      int did = u->getDestinationID();
      int s = u->getCurSector();
      trafficGenerated[s][did]++;
      return true;
    } else {
      return false;
    }
  }

	bool tracking; /**< Whether or not to record to disk various 
				        information about the domain for each timestep */
	
  /** Simulate one step in the domain where agent actions are taken into account.
  *  \param agent_actions The actions chosen/computed by the agents (for link agents, this is one action per agent)
  *  \return value of objective accumulated from single step
  */
  double simulateStep(matrix2d agent_actions);
  // static bool uavReadyToMove(std::vector<Link*> L,
  //     std::map<edge, size_t> *L_IDs, UAV *u);

	void generateNewAirspace(std::string dir, size_t xdim, size_t ydim);
  
  /** Creates a link based with a source node, target node, direction, time to travel, and capacity.
  *  \param e edge which specifies source and target nodes
  *  \param flat_capacity the capacity of the new link
  */
//	void addLink(edge e, double flat_capacity);

  // JJC: add links with fixed costs and fixed travel time
	void addLink(edge e, size_t flat_capacity, size_t cost, int time);
	
  std::string createExperimentDirectory(std::string config_file);
	
  /** Generates (with some probability) plane traffic for each generation sector. */
  virtual void getNewUavTraffic();
	
  /** Generates (with some probability) plane traffic for sector s.
  *  \param s the ID of the sector to generate UAVs 
	*/
  void getNewUavTraffic(int s, bool k = false);
	
	/** Removes UAVs from the domain which have reached their destination. */
  virtual void absorbUavTraffic();
	
  /** Calculates state of the agents.
  *  \return state of the traffic agents
  */
  matrix2d getStates();

	/** Deprecated */
  void logStep();

  void exportSectorLocations(int fileID);

	/** Calculates and returns performance of the UTM system
  *  \return performance of the system
  */
  virtual double getPerformance();
	
	/** Calculates and returns reward for each agent */
  virtual matrix1d getRewards();

	/** UAVs travel along their planned paths. */
  virtual double incrementUavPath();

  virtual int getPathPlans();

  virtual void getPathPlans(const std::list<UAV*> &new_uavs);
	
	/** Resets the domain such that a new simulation can be run. */
  virtual void reset();
  //virtual void tryToMove(std::vector<UAV*> * eligible_to_move);
	
	/** Get the ID of the nth link in a UAVs planned path 
  *	\param u pointer to the UAV 
  *	\param n n
  *  \return ID of the nth link in UAVs path
  */
  size_t getNthLinkID(UAV* u, size_t n);
	
  /** Get the pointer to the nth link in a UAVs planned path
  *	\param u pointer to the UAV
  *	\param n n
  *  \return the nth link in UAVs path
  */
  Link* getNthLink(UAV* u, size_t n);
	
  /** Get the ID of the current link that a UAV is traveling
  *	\param u pointer to the UAV
  *  \return ID of the current link in UAVs path
  */
  size_t getCurLinkID(UAV* u) {
    return k_link_ids_->at(u->getCurEdge());
  }
	
  /** Get the pointer to the current link that a UAV is traveling
  *	\param u pointer to the UAV
  *  \return the current link in UAVs path
  */
  Link* getCurLink(UAV* u) {
    return links_[getCurLinkID(u)];
  }
};
#endif  // SRC_DOMAINS_UTM_UTMDOMAINABSTRACT_H_
