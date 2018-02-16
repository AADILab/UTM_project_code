// Copyright 2016 Carrie Rebhuhn
#include "UTMDomainAbstract.h"

using std::string;
using std::list;
using std::vector;
using std::map;
using std::bind;
using std::to_string;
using std::placeholders::_1;
using easymath::XY;
using easymath::zeros;
using easystl::remove_erase_if;

UTMDomainAbstract::UTMDomainAbstract(YAML::Node configs, std::string costmode, bool) :
  IDomainStateful() {
  initializeBase(configs);

//  uav_count = 0;
  mission_count = 0 ;
  completed_count = 0 ;

  max_action = DBL_MIN;
  min_action = DBL_MAX;

  // Pointers to rewards
  G = new UTMGlobal(T->MAX_STEP);
  d_approx = new UTMDifferenceApprox(G, k_num_agents_);

  printf("Creating a UTMDomainAbstract object.\n");

  /*	With tracking enabled, uav paths, states of agents, 
	  and other information is collected and saved disk. (See UTMTracker.h)
	  Tracking is enabled through the config.yaml file
  */
  tracking = configs["modes"]["tracking"].as<bool>();

  string domain_dir = UTMFileNames::createDomainDirectory(configs);
  metrics_dir = UTMFileNames::createMetricsDirectory(configs);
  // Create directory where tracking info will be saved (if enabled)
  string tracker_dir = UTMFileNames::createTrackerDirectory(configs);
  string efile = domain_dir + "edges.csv";
  string vfile = domain_dir + "nodes.csv";
  
  std::cout << "Domain directory: " << domain_dir << "\n" ;
  std::cout << "Edges directory: " << efile << "\n" ;
  std::cout << "Nodes directory: " << vfile << "\n" ;
  
  std::cout << "Output metrics directory: " << metrics_dir << "\n" ;

  /* Load some config file settings */
  string airspace_mode = configs["modes"]["airspace"].as<string>();
  k_disposal_mode_ = configs["modes"]["disposal"].as<string>();
  k_num_sectors_ = configs["constants"]["sectors"].as<size_t>();
  k_reward_mode_ = configs["modes"]["fitness"].as<string>();
  k_record_ = configs["modes"]["prob_record"].as<bool>();
  k_window_mode_ = configs["modes"]["window_mode"].as<std::string>();
  k_window_size_ = configs["constants"]["window_size"].as<size_t>();
  k_capacity_mode_ = configs["modes"]["capacity"].as<string>();
  k_agent_type_ = configs["modes"]["agent_type"].as<string>();
  k_travel_time_ = configs["modes"]["travel_time"].as<string>();
  k_output_ = configs["modes"]["output"].as<string>() ;
  k_pred_ = configs["modes"]["predicted"].as<string>() ;

  // Pointer to tracker
  tracker = new UTMTracker(T, getNumAgents(), k_num_sectors_);

  // Variables to fill
  if (airspace_mode != "saved" || !cio::fileExists(efile)) {
    size_t xdim = configs["constants"]["xdim"].as<size_t>();
    size_t ydim = configs["constants"]["ydim"].as<size_t>();
    generateNewAirspace(domain_dir, xdim, ydim);
  }

  vector<edge> edges = cio::readPairs<edge>(efile);
  vector<XY> locs = cio::readPairs<XY>(vfile);

  high_graph_ = new LinkGraph(locs, edges);
  
  /* JJC: Link construction */
//  if (k_agent_type_ == "fixed") { // Fixed agents for comparative experimentsauto genfile = cio::read2<size_t>(domain_dir + "generation_points.csv");
//    costfile = cio::read2<size_t>(domain_dir + "costs.csv");
//  }

//  // Link construction
//	k_link_ids_ = new map<edge, size_t>();
//	if (k_capacity_mode_ == "flat") // All links have equal capacity
//	{
//		double flat_capacity = configs["constants"]["capacity"].as<double>();
//		for (edge e : edges) addLink(e, flat_capacity);
//	}
//	else // All links have capacities as specified in capacity file (Domain dir)
//	{
//		auto capfile = cio::read2<size_t>(domain_dir + "capacity.csv");
//		for (size_t i = 0; i < capfile.size(); i++) {
//		  if (k_agent_type_ == "fixed") {
//        auto costfile = cio::read2<size_t>(domain_dir + "costs.csv");
//		    addLink(edges[i], capfile[i][0], costfile[i][0]);
//	    }
//	    else{
//  			addLink(edges[i], capfile[i][0], 0);
//			}
//		}
//	}
  
  // Link construction
	k_link_ids_ = new map<edge, size_t>();
	bool isLoopConstruct = false ;
	size_t loopSize = 0 ;
	size_t cap_e = 0 ;
	size_t cost_e = 0 ;
	int dist_e = -1 ;
	vector<vector<size_t>> capfile ;
	vector<vector<size_t>> costfile ;
	vector<vector<int>> distfile ;
	
	if (k_capacity_mode_ == "flat"){ // All links have equal capacity
		cap_e = configs["constants"]["capacity"].as<size_t>();
	}
	else{
	  capfile = cio::read2<size_t>(domain_dir + "capacity.csv");
	  isLoopConstruct = true ;
	  loopSize = capfile.size() ;
  }
  if (k_agent_type_ == "fixed"){
		costfile = cio::read2<size_t>(domain_dir + "costs.csv");
		isLoopConstruct = true ;
		loopSize = costfile.size() ;
	}
	if (k_travel_time_ == "list"){
	  distfile = cio::read2<int>(domain_dir + "distances.csv");
	  isLoopConstruct = true ;
	  loopSize = distfile.size() ;
  }
  if (!isLoopConstruct){
    for (edge e : edges) addLink(e, cap_e, cost_e, dist_e);
  }
  else{
    for (size_t i = 0; i < loopSize; i++) {
      if (k_capacity_mode_ == "list")
        cap_e = capfile[i][0] ;
      if (k_agent_type_ == "fixed")
		    cost_e = costfile[i][0] ;
	    if (k_travel_time_ == "list")
	      dist_e = distfile[i][0] ; 
      
	    addLink(edges[i], cap_e, cost_e, dist_e);
    }
  }
  /* JJC: end link construction */

	// For each link, add links that connect to it
  for (auto l:links_) {
    size_t src = l->k_source_; // my source
	  for (size_t i = 0; i < links_.size(); i++) {
		  // if not the same link and my source == their target
		  if (links_[i] != l && src == links_[i]->k_target_) { 
			  l->k_connections_.push_back(links_[i]);
		  }
  	}
	  l->initHist(); // initialize data structures for link

	  //size_t target = l->k_target_;
	  //links_[target]->k_connections_.push_back(l);
  }

  /* Load more config file settings */
  k_agent_mode_ = configs["modes"]["agent"].as<string>();
  k_generation_mode_ = configs["modes"]["generation"].as<string>();
  k_destination_mode_ = configs["modes"]["destination"].as<string>();

  // Sector
  if (k_generation_mode_ == "all") { // Generate UAVs at all sectors
    for (size_t i = 0; i < locs.size(); i++) {
      generation_sectors.push_back(i);
    }
  } else { // Generate UAVs only at select sectors
    auto genfile = cio::read2<size_t>(domain_dir + "generation_points.csv");
    for (size_t i = 0; i < genfile.size(); i++) {
      generation_sectors.push_back(genfile[i][0]);
    }
  }

  if (k_destination_mode_ == "all") { // UAVs assigned goal to any sector
    for (size_t i = 0; i < locs.size(); i++) {
      destination_sectors.push_back(i);
    }
  } else { // UAVs assigned goal to select sectors
    auto destfile = cio::read2<size_t>(domain_dir + "destination_points.csv");
    for (size_t i = 0; i < destfile.size(); i++) {
      destination_sectors.push_back(destfile[i][0]);
    }
    if (k_destination_mode_ == "p_list"){
      for (size_t i = 0; i < k_num_sectors_; i++) 
        for (size_t j = 0; j < destination_sectors.size(); j++)
          if (i != j)
            paired_destination_sectors.push_back(i) ;
    }
  }
  
  
  // Sector/Fix  construction
  vector<vector<size_t> > connections(k_num_sectors_);
  for (edge e : edges)
    connections[e.first].push_back(e.second);

  vector<XY> sector_locs = high_graph_->get_locations();
  for (size_t i = 0; i < k_num_sectors_; i++) {
    // Only include non-self as destination
    //vector<size_t> dest_IDs(destination_sectors.size()-1);
	  vector<size_t> dest_IDs;
    if (k_destination_mode_ == "p_list"){ // check if current sector is in destination list or not
      bool in_dest = false ;
      for (size_t j = 0; j < destination_sectors.size(); j++){
        if (destination_sectors[j] == i){
          in_dest = true ;
          break ;
        }
      }
      if (in_dest){ // if it is one of the destination sectors, it will always assign to a paired sector
        for (size_t j = 0; j < paired_destination_sectors.size(); j++) {
          if (paired_destination_sectors[j] == i) continue;
          dest_IDs.push_back(paired_destination_sectors[j]);
        }
      }
      else{ // if it is not one of the destination sectors, it will always assign to a destination sector
        for (size_t j = 0; j < destination_sectors.size(); j++) {
          if (destination_sectors[j] == i) continue;
          dest_IDs.push_back(destination_sectors[j]);
        }
      }
    }
    else{
      for (size_t j = 0; j < destination_sectors.size(); j++) {
        if (destination_sectors[j] == i) continue;
        dest_IDs.push_back(destination_sectors[j]);
      }
    }

    Sector* s = new Sector(sector_locs[i], i, connections[i], sector_locs, configs, i, high_graph_, dest_IDs);
    sectors_.push_back(s);

    /* Following code is for UAV playback implemented by Brandon. Not necessary for what experiments
    I did, but here it is in case it's useful later */

    // Give new sector pointer to SimTime
    sectors_[sectors_.size() - 1]->T = T;
    if (configs["modes"]["traffic"].as<string>() == "playback")
	    // Give new sector the path to the domain directory
	    sectors_[sectors_.size() - 1]->domain_dir = UTMFileNames::createDomainDirectory(configs);
  }
  
  // not used?
  trafficGenerated = matrix2d(k_num_sectors_, matrix1d(k_num_sectors_, 0.0));

  // Start with UAVs if given initial UAV poses
  k_position_mode_ = configs["modes"]["position"].as<string>();
  if (k_position_mode_ == "constant") {
    // Create uavs_ on links_
    string pose_file = domain_dir + "initial_pose.csv";
    poses = cio::read2<int>(pose_file);

    for (size_t i = 0; i < poses.size(); i++) {
      // only reset count on the first one
      if (k_disposal_mode_ == "keep") {
        getNewUavTraffic(poses[i][0], true);
      }
      else{
        getNewUavTraffic(poses[i][0]);
      }
    }
  }

  
  /* Generate sector/link agents */
  bool learn = false ;
  if (k_output_ == "learn")
    learn = true ;
    
  if (k_agent_mode_ == "sector") {
    agents_ = new SectorAgent(links_, sectors_, k_num_states_);
    k_num_agents_ = sectors_.size();
  } else {
    agents_ = new LinkAgent(links_.size(), links_, k_num_states_, learn);
    k_num_agents_ = links_.size();
  }
  //num_uavs_at_sector_ = zeros(k_num_sectors_);

  /* The objective that the agents will try to optimize */
  k_objective_mode_ = configs["modes"]["objective"].as<string>();
  /* Not sure why this is set to true here, but won't change anything so code doesn't break :) */
  k_extra_info_ = true; // Prints info about UAV delay, moving, and wait times
    
  // Start with UAVs
  // For experiments carried out by Brandon, below code block not used (disposal mode never set to "keep")
//  if (k_disposal_mode_ == "keep") {
//    if (k_position_mode_ == "constant" && poses.size() == 0) {
//      string pose_file = domain_dir + "initial_pose.csv";
//      poses = cio::read2<int>(pose_file);
//      for (size_t i = 0; i < poses.size(); i++) {
//        getNewUavTraffic(poses[i][0]);
//      }
//    } else {
//      for (size_t i = 0; i < sectors_.size(); i++) {
//        getNewUavTraffic(i);
//      }
//    }
//  }

  /* For experiments carried out by Brandon, agents use G as their reward with no approximations.
   Therefore, code block below doesn't matter */
  for (size_t i = 0; i < k_num_agents_; i++) {
    std::string m = costmode;
	//if (m.find("current") != std::string::npos) {
    if (m == "current") {
      G_t.push_back(new GNNCurrent(configs["approximator"], T,i));
    } else if (m == "past") {
      G_t.push_back(new GNNPast(configs, T,i));
    } else if (m == "combined") {
      G_t.push_back(new GNNCombined(configs, T,i));
    } else if (m == "delta") {
      G_t.push_back(new GNNDelta(configs, T, i));
    } else {
      std::printf("Unknown NN State.");
    }
  }

	if (tracking){ // If tracking enabled, record all link traversal times and capacities
	  for (auto l : links_) {
		  tracker->edgeTimes.push_back(l->get_time());
		  tracker->capacities.push_back(l->get_capacity());
	  }
	}
}

//void UTMDomainAbstract::addLink(UTMDomainAbstract::edge e, double flat_capacity) {
//  size_t source = e.first;   // membership of origin of edge
//  size_t target = e.second;  // membership of connected node
//  vector<XY> locs = high_graph_->get_locations();
//  XY s_loc = locs[source];
//  XY t_loc = locs[target];

//  size_t cardinal_dir = cardinal_direction(t_loc - s_loc);
//  size_t dist = static_cast<size_t>(euclidean_distance(s_loc, t_loc));
//  if (dist == 0)
//    dist = 1;

//  Link* newLink = new Link(links_.size(), source, target, dist,
//	static_cast<size_t>(flat_capacity), cardinal_dir, k_window_size_, k_window_mode_ == "cumulative");
//  links_.push_back(newLink); // add link to list of all links

//  k_link_ids_->insert(make_pair(e, links_.size() - 1));

//  k_incoming_links_[target].push_back(source);
//}

// JJC: comparative experiments with fixed costs
void UTMDomainAbstract::addLink(edge e, size_t flat_capacity, size_t cost, int dist_e) {
  size_t source = e.first;   // membership of origin of edge
  size_t target = e.second;  // membership of connected node
  vector<XY> locs = high_graph_->get_locations();
  XY s_loc = locs[source];
  XY t_loc = locs[target];

  size_t cardinal_dir = cardinal_direction(t_loc - s_loc);
  size_t dist ;
  if (dist_e < 0)
    dist = static_cast<size_t>(euclidean_distance(s_loc, t_loc));
  else
    dist = dist_e ;
  if (dist == 0)
    dist = 1;
  
  bool pred = false ;
  if (k_pred_ == "include")
    pred = true ;

  Link* newLink = new Link(links_.size(), pred, source, target, dist,
	static_cast<size_t>(flat_capacity), cardinal_dir, k_window_size_, k_window_mode_ == "cumulative", cost);
  links_.push_back(newLink); // add link to list of all links

  k_link_ids_->insert(make_pair(e, links_.size() - 1));

  k_incoming_links_[target].push_back(source);
}

void UTMDomainAbstract::generateNewAirspace(string domain_dir, size_t xdim, size_t ydim) {
  // Generate a new airspace
  LinkGraph(k_num_sectors_, xdim, ydim).print_graph(domain_dir);
}

UTMDomainAbstract::UTMDomainAbstract(YAML::Node configs, string costmode) :
  UTMDomainAbstract(configs, costmode, true) {}

// Destructor
UTMDomainAbstract::~UTMDomainAbstract(void) {
  delete k_link_ids_;
  delete agents_;

  for (Link* l : links_) delete l;
  for (Sector* s : sectors_) delete s;
  for (UAV* u : uavs_) delete u;
}

double UTMDomainAbstract::getPerformance() {
	if (k_objective_mode_ == "travel_time")
		return (*G)(); // Performance based on total travel time
	else // delay
		return delays; // Performance based only on total delay
}

matrix1d UTMDomainAbstract::getRewards() {
  double G_actual = getPerformance();
//  std::cout << "G_actual: " << G_actual ;
  // JJC: edit here to normalize by number of completed missions (e.g. uavs generated in system that reached destination)
//  G_actual /= (uav_count - uavs_.size()) ;
  G_actual /= (completed_count) ;
//  std::cout << ", normalized according to #UAVs: " << G_actual << "\n" ;
  if (k_reward_mode_ == "global") {
    return matrix1d(k_num_agents_, G_actual);
  } else {
    matrix1d D(k_num_agents_, 0.0);

    for (size_t i = 0; i < k_num_agents_; i++) {
        D[i] = G_actual - G_t.at(i)->estimate();
    }
    return D;
  }
}

double UTMDomainAbstract::incrementUavPath() {

	moving = 0; // total number of UAVs that are able to move this timestep
	delays = 0; // total number of UAVs that are delayed this timestep
	ground_hold = 0; // total number of UAVs that have yet to leave their origin sector this timestep

  // Try to add waiting UAVs onto links
  for (auto s : sectors_) {
    for (int i = 0; i < s->waiting.size(); i++) { // increment through UAVs waiting at sector
      UAV* u = s->waiting.at(i);
	    u->planAbstractPath();
      if (addUav(u)) { // if uav added to a link
	      // remove UAV from sector
        s->waiting.at(i) = s->waiting.back();
        s->waiting.pop_back();
        i--;
      }
      else {
	      // UAV could not be added, increment wait time
	      ground_hold++;
      } 
    }
  }

  for (auto u : uavs_) {
	  // move UAVs here
    // all not travelling+at destination should have been absorbed by now
    if (!u->travelling()) {
      Link *cur_link, *next_link;

      cur_link = getCurLink(u);

      next_link = getNthLink(u, 1);

      // Attempt move UAV to next link in its path
      bool success = next_link->grab(u, cur_link);

      if (!success) {
        // If not successful, add to count of delayed UAVs
        delays++;
        u->planAbstractPath();
        cur_link->delays++;
      }
      else {
        // otherwise UAV is considered as "moving"
        moving++;
      }
    } else {
      // For all other UAVs on links, let them continue traveling
      u->decrementWait();
      moving++;
      if (!u->travelling()) {
        // got to the end of the link!
        u->setCurSector(getCurLink(u)->k_target_);
      }
    }
  }
  
  // Always return travel time. We can deal with delay vs. travel time later (see getPerformance)
  return static_cast<double>(delays + moving + ground_hold);
}

matrix2d UTMDomainAbstract::getStates() {
  // CONGESTION STATE
  return agents_->computeCongestionState(uavs_);
}


double UTMDomainAbstract::simulateStep(matrix2d agent_actions) {
	matrix2d S = getStates(); // For tracker
  // Returns the next state (S_t+1, and reward R_t)
  bool D_star = true; // todo: change this to a mode

  //bool impactfulness_measuring = true;
  /*
  for (int i = 0; i < agent_actions.size(); i++) {
      agent_actions[i][0] = 1;
  }//*/
  

  // Alter the cost maps (agent actions)
  matrix1d w = agents_->actionsToWeights(agent_actions);
  for (auto outer : agent_actions) {
    for (auto inner : outer) {
      if (inner > max_action) {
        max_action = inner;
      }
      if (inner < min_action) {
        min_action = inner;
      }
    }
  }
  
  // ACTUAL calculation of impactfulness
  /* for (int i = 0; i < agent_actions.size(); i++) {
    matrix2d aa = agent_actions;

    // Go high
    aa[i][0] = 100000;
    matrix1d wi = agents_->actionsToWeights(aa);
    high_graph_->set_weights(wi);
    int num_changed = getPathPlans();

    // Go low
    aa[i][0] = 0;
    wi = agents_->actionsToWeights(aa);
    high_graph_->set_weights(wi);
    num_changed = getPathPlans();
  }*/

  //bool action_changed = true; // sanity check
  bool action_changed = (high_graph_->get_weights() == w);
  high_graph_->set_weights(w);

  // Make uavs_ reach their destination
  absorbUavTraffic();

  if (k_disposal_mode_ != "keep"){ // Only generate new traffic if not in "keep" mode
    // Note: this adds to traffic
    this->getNewUavTraffic();
  }

  // Plan over new cost maps
  if (action_changed && D_star) {
    getPathPlans();
  }
  // uavs_ move
  double step_delay = incrementUavPath();
  list<UAV*> wait;
  for (auto s : sectors_) {
	  for (int i = 0; i < s->waiting.size(); i++) {
		  UAV* u = s->waiting.at(i);
		  wait.push_back(u); // Record which UAVs had to wait at gen sector
	  }
  }

	if (tracking)
		// Give tracker info on states, actions, weights, waiting UAVs, and total UAV count
//		tracker->stepUpdate(getStates(), agent_actions, w, uavs_, wait, uav_count);
		tracker->stepUpdate(getStates(), agent_actions, w, uavs_, wait, mission_count);
	
  if (k_reward_mode_ != "global") {
    for (size_t i = 0; i < G_t.size(); i++) {
      G_t.at(i)->train(agent_actions[i][0], links_.at(i), step_delay);
    }
  }

	// G = -(delay in system) or -(total travel time) [depending on objective]
  (*G)(T->step) = -step_delay;

	if (k_state_rep_ == "incoming") {
		for (auto l : links_) {
			l->updateTrafficProb(); // Update incoming UAV ratios
		}
		// Code can go here if you want to debug something about the data collected by the links (see previous commits for an example)
		for (auto l : links_) {
			l->slideWindow(); // Out with old information, in with the new (unless we have cumulative window ;) )
		}
	}
  return -step_delay; // this may be delay or time travel, depending on objective
}

// Records information about a single step in the domain
// deprecated
void UTMDomainAbstract::logStep() {
  /*if (k_agent_mode_ == "sector" || k_agent_mode_ == "link") {
    matrix1d num_uavs_on_links(links_.size(), 0);
    for (size_t i = 0; i < links_.size(); i++) {
        num_uavs_on_links[i] = links_[i]->traffic_.size();
    }

    link_uavs_.push_back(num_uavs_on_links);
    sector_uavs_.push_back(num_uavs_at_sector_);
    num_uavs_at_sector_ = zeros(sectors_.size());
  }*/
}

void UTMDomainAbstract::exportSectorLocations(int fileID) {
  vector<easymath::XY> sectorLocations;
  for (Sector* s : sectors_)
    sectorLocations.push_back(s->k_loc_);
  cio::printPairs(sectorLocations, "visualization/agent_locations" + to_string(fileID) + ".csv");
}

int UTMDomainAbstract::getPathPlans() {
  int num_changed = 0;
  for (UAV* u : uavs_) {
    if (!u->travelling()) {
      bool changed = u->planAbstractPath();
      if (changed)
        num_changed++;
    }
  }
  return num_changed;
}

void UTMDomainAbstract::getPathPlans(const list<UAV*> &new_UAVs) {
  for (UAV* u : new_UAVs) {
    u->planAbstractPath();  // sets own next waypoint
  }
}

void UTMDomainAbstract::reset() {
  max_action = DBL_MIN;
  min_action = DBL_MAX;
  //nn_states_saved = matrix3d(k_num_steps_, matrix2d(k_num_agents_));
//  std::printf("%i uavs in uavs_, %i uavs according to uav_count\n", uavs_.size(), uav_count);
//  std::printf("%i / %i reached destination\n", (uav_count - uavs_.size()), uav_count);
  std::printf("%i / %i reached destination\n", completed_count, mission_count);
  while (!uavs_.empty()) {
    delete uavs_.back();
    uavs_.pop_back();
  }
  //trafficGenerated = matrix2d(k_num_sectors_, matrix1d(k_num_sectors_, 0.0));

  string domain_dir = "Domains/" + to_string(k_num_sectors_) + "_Sectors/";

  for (Link* l : links_) {
    l->reset(); // reset each link and its data structures
  }
  T->step = 0;
  agents_->reset();
  for (size_t i = 0; i < sectors_.size(); i++) {
    sectors_[i]->reset();
  }
	
	// Start over with UAV count
//	uav_count = 0;
	mission_count = 0 ;
	completed_count = 0 ;

  if (k_disposal_mode_ == "keep") {
    if (k_position_mode_ == "constant"){// && poses.size() == 0) {
      string pose_file = domain_dir + "initial_pose.csv";
      poses = cio::read2<int>(pose_file);
      for (size_t i = 0; i < poses.size(); i++) {
//        std::cout << "Generating UAV at sector: " << poses[i][0] << "\n" ;
        getNewUavTraffic(poses[i][0],true);
      }
    } else {
      for (size_t i = 0; i < sectors_.size(); i++) {
        getNewUavTraffic(i,true);
      }
    }
  }

	//resetUavNums = true;
	// Reset other counters
	delays = 0;
	ground_hold = 0;
	moving = 0;

	tracker->clear();
}

void UTMDomainAbstract::absorbUavTraffic() {
  for (std::list<UAV*>::iterator it = uavs_.begin(); it != uavs_.end();) {
    UAV* u = *it;
    auto cur_link = getCurLink(u);

    // check - done travelling, on terminal link
    if (!u->travelling() && u->terminal()) {
      completed_count++ ;
      if (k_disposal_mode_ == "keep") {
        auto cur_sector = u->getCurEdge().second;
        // JJC: hack to keep UAVs in system, first remove UAV and then generate a new UAV in its place, this avoids the backlog problem where UAVs continue to count towards previous link and the system gets stuck since it requires simultaneous link switching
//        // generates new path
//        sectors_.at(cur_sector)->remake(u);
//        mission_count++ ;
//        auto new_cur_link = getNthLink(u, 0);
//        auto success = new_cur_link->grab(u, cur_link);
//        if (!success) {
//          u->planAbstractPath();  // option to replan if link blocked
//        }
        cur_link->remove(u);
        it = uavs_.erase(it) ;
        delete u ;
        getNewUavTraffic(cur_sector,true) ;
      } else {
        // Remove
        cur_link->remove(u);
        it = uavs_.erase(it);
        delete u;
      }
    } else {
      it++;
    }
  }
}

void UTMDomainAbstract::getNewUavTraffic(int s, bool k) {
//  sectors_.at(s)->generateUavs(T->step, &uav_count, k);
  sectors_.at(s)->generateUavs(T->step, &mission_count, k);
}

void UTMDomainAbstract::getNewUavTraffic() {
  // Generates (with some probability) plane traffic for each generation sector
	std::vector<size_t> num_generated(k_num_sectors_, 0); // keeps track of how many UAVs a sector generates
  for (size_t s : generation_sectors) {
//	  num_generated[s] = sectors_.at(s)->generateUavs(T->step, &uav_count); // adding uavs pushed to later; this just populates a wait list
	  num_generated[s] = sectors_.at(s)->generateUavs(T->step, &mission_count); // adding uavs pushed to later; this just populates a wait list
  }
	if (k_record_) // if recording enabled...
		tracker->updateGenUavs(num_generated); // record how many UAVs
}

size_t UTMDomainAbstract::getNthLinkID(UAV* u, size_t n) {
  return k_link_ids_->at(u->getNthEdge(n));
}

Link* UTMDomainAbstract::getNthLink(UAV* u, size_t n) {
  return links_[getNthLinkID(u, n)];
}

string UTMDomainAbstract::createExperimentDirectory(string config_file) {
  return UTMFileNames::createExperimentDirectory(config_file);
}

void UTMDomainAbstract::printTrackingInfo() {
	tracker->print();
}

void UTMDomainAbstract::printUavGen() {
	tracker->printGen();
}

matrix1d UTMDomainAbstract::getExtraInfo() {
	// This function gets called after every simulation step
	// Total delay, moving time, and wait time for full simulation is calculated later (SimNE::accounting::update_extra)
	matrix1d info;
	if (k_extra_info_) {
		info.push_back(moving);
		info.push_back(delays);
		info.push_back(ground_hold);
		info.push_back(mission_count); // total number of assigned missions throughout epoch
		info.push_back(completed_count) ; // number of successfully completed missions
//		info.push_back(uav_count); // total number of assigned missions throughout epoch
//		info.push_back(uav_count - uavs_.size()) ; // number of successfully completed missions
	}
	return info;
}

matrix2d UTMDomainAbstract::getExtraExtraInfo() {
	// This function was made to help me try and figure out why performance improved
	// only when the "sweet spot" was found in terms of number of gnerated UAVs
	
	// A last minute addition that is best to ignore
	matrix2d info;
	for (size_t i = 0; i < links_.size(); i++) {
		info.push_back(matrix1d());
		info[i].push_back(links_[i]->delays);
		info[i].push_back(links_[i]->not_accepted);
	}
	return info;
}
