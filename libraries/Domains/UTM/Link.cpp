// Copyright 2016 Carrie Rebhuhn
#include "Link.h"

using std::vector;
using std::list;
using std::greater;

//Link::Link(size_t id, size_t source, size_t target, size_t time,
//  size_t capacity, size_t cardinal_dir, size_t window_size, bool cumulative = false) :
//  k_id_(id), k_source_(source), k_target_(target), time_(time),
//  k_cardinal_dir_(cardinal_dir), k_capacity_(capacity),
//  k_window_size_(window_size), traffic_(list<UAV*>()),
//  cumulative_(cumulative), delays(0), not_accepted(0)
//{}

Link::Link(size_t id, bool pred, size_t source, size_t target, size_t time,
  size_t capacity, size_t cardinal_dir, size_t window_size, bool cumulative = false, double cost) :
  k_id_(id), k_include_predicted_(pred), k_source_(source), k_target_(target), time_(time), cost_(cost),
  k_cardinal_dir_(cardinal_dir), k_capacity_(capacity),
  k_window_size_(window_size), traffic_(list<UAV*>()),
  cumulative_(cumulative), delays(0), not_accepted(0)
{}

void Link::initHist() {
	k_incoming_prob_ = std::map<size_t, double>();
	for (auto conn : k_connections_) {
		k_incoming_prob_.insert(std::make_pair(conn->k_source_, 1.0));
	}
	
	if (cumulative_) {
		// If cumulative mode, we don't need big data structures because we're keeping data about entire simulation (instead of only part of it)
		k_traffic_heist_ = 0;
		k_incoming_from_santa_ = std::map<size_t, size_t>();
		for (auto conn : k_connections_) { // for each connected link...
			k_incoming_from_santa_.insert(std::make_pair(conn->k_source_, 0)); // initialize number of UAVs coming from it to 0
		}
	}
	else if (k_window_size_) { // if and only if window size is greater than 0
		// create data structure for keeping track of how many UAVs travel on this link, and those that enter this link from other links
		k_traffic_hist_ = std::deque<size_t>(k_window_size_);
		k_incoming_from_ = std::map<size_t, std::deque<size_t> >();
		for (auto conn : k_connections_) { // for each connected link...
			k_incoming_from_.insert(std::make_pair(conn->k_source_, std::deque<size_t>(k_window_size_))); // initialize number of UAVs coming from it to 0 (for each step)
		}
	}
}

bool Link::atCapacity() {
  return numOverCapacity() >= 0;
}

int Link::numOverCapacity() {
  return static_cast<int>(traffic_.size() - k_capacity_);
}

double Link::predictedTraversalTime() {
  if (k_include_predicted_){
    // Get predicted wait time for each type of UAV
    double predicted = 0;
    
    // Collect wait times on all UAVs ON the link
    matrix1d waits;
    for (UAV* u : traffic_) {
      waits.push_back(u->getWait()+cost_);
    }

    // Sort by wait (descending)
    sort(waits.begin(), waits.end(), greater<double>());

    int n_ok = k_capacity_ - 1;  // UAVs you don't have to wait for
    int n_wait = waits.size() - n_ok;  // UAVs before you in line
    for (int i = 0; i < (n_wait - n_ok); i++) {
      waits.pop_back();
    }

    // Store predicted link time.
    double w = easymath::sum(waits);
    predicted = time_ + cost_ + w ; // cost = 0 if cost_type is not fixed
    if (w < 0) {
      printf("bad");
    }

    return predicted;
    }
  else{
    // JJC: no predicted time, only use known cost
    return time_ + cost_ ;
  }
}

bool Link::grab(UAV* u, Link* l) {
  /*
  * Attempts to grab UAV u from the given link onto this link.
  * First checks if there is capacity on this link.
  * Returns true if the move is successful.
  * Returns false if the move is unsuccesful.
  * Errors out and exits if link l does not contain UAV u.
  */

  if (add(u)) {
    // Remove from previous node (l)
    try {
      l->remove(u);
      updateIncomingHist(l->k_source_);
      u->incrementPath();
    }
    catch (int e) {
      printf("Exception %i occurred. Pausing then exiting.", e);
      system("pause");
      exit(e);
    }
    return true;
  }
  else {
    return false;
  }
}

bool Link::add(UAV* u) {
  //printf("UAV %i added to link %i.\n", u->getId(), k_id_);
  if (countTraffic() >= k_capacity_){
	  not_accepted++;
    return false; // unsuccessful add
  }
  if (time_ < 0) {
    printf("bad");
  }
  u->setCurEdge(k_source_, k_target_,k_id_);
  u->setWait(time_);
  traffic_.push_back(u);

  return true; // successful add
}


void Link::remove(UAV* u) {
  try {
    // printf("UAV %i removed from link %i.\n", u->getId(), k_id_);
    easystl::remove_element(&traffic_, u);
	if (cumulative_)
		k_traffic_heist_++; // Recall this is how many UAVs exit link!
	else if (k_window_size_)
		k_traffic_hist_.front()++; // Recall this is how many UAVs exit link!
  }
  catch (int e) {
    throw e;
  }
}

void Link::reset() {
  traffic_ = list<UAV*>();
	initHist(); // whoops, this may be redundant
	delays = 0;
	not_accepted = 0;
}

void Link::updateIncomingHist(size_t src) {
	if (cumulative_)
		k_incoming_from_santa_[src]++;
	else if (k_window_size_) {
		k_incoming_from_[src].front()++;
	}
}

void Link::updateTrafficProb() {
	if (cumulative_) {
		std::map<size_t, size_t > conn_traffic;
		for (auto conn : k_connections_) { // for each connected link...
			conn_traffic.insert(std::make_pair(conn->k_source_, conn->k_traffic_heist_)); // map its source node to the number of UAVs that have exited from said link
		}

		// Iterate through ratios
		for (std::map<size_t, double>::iterator iter = k_incoming_prob_.begin(); iter != k_incoming_prob_.end(); ++iter){
			size_t src = iter->first; // get the src node ID
			double sum_traffic = conn_traffic[src]; // How many UAVs exited from link that starts at src?
			if (sum_traffic > 0.0) { // If there was at least 1...
				// Update the ratio
				double sum_incoming_from = k_incoming_from_santa_[src]; // Get how many of those UAVs entered my link
				k_incoming_prob_[src] = (sum_incoming_from / sum_traffic); // Math!
			}
			else {
				// Don't update
			}
		}
	}
	if (k_window_size_) {
		std::map<size_t, std::deque<size_t> > conn_traffic;
		for (auto conn : k_connections_) { // for each connected link...
			conn_traffic.insert(std::make_pair(conn->k_source_, conn->k_traffic_hist_)); // map its source node to the number of UAVs that have exited from said link
		}

		// Iterate through ratios
		for (std::map<size_t, double>::iterator iter = k_incoming_prob_.begin(); iter != k_incoming_prob_.end(); ++iter){
			size_t src = iter->first; // get the src node ID
			double sum_traffic;
			// How many UAVs exited from link that starts at src? (total over time window)
			sum_traffic = std::accumulate(conn_traffic[src].begin(), conn_traffic[src].end(), 0.0);  
			if (sum_traffic > 0.0) { // If there was at least 1...
				// Update the ratio
				// Get how many of those UAVs entered my link (total over time window)
				double sum_incoming_from = std::accumulate(k_incoming_from_[src].begin(), k_incoming_from_[src].end(), 0.0); 
				k_incoming_prob_[src] = (sum_incoming_from / sum_traffic); // Math!
			}
			else {
				// Don't update
			}
		}
	}
}

void Link::slideWindow() {
	if (k_window_size_) { // Only if using non-cumulative mode do we slide the window
		for (std::map<size_t, double>::iterator iter = k_incoming_prob_.begin(); iter != k_incoming_prob_.end(); ++iter){
			size_t src = iter->first;
			k_incoming_from_[src].pop_back(); k_incoming_from_[src].push_front(0);
		}
		k_traffic_hist_.pop_back(); k_traffic_hist_.push_front(0);
	}
}

LinkAgent::LinkAgent(size_t num_edges, vector<Link*> links, size_t num_state_elements, bool learn) :
  k_num_edges_(num_edges), k_output_(learn), IAgentBody(links.size(), num_state_elements), links_(links){
	for (size_t i = 0; i < links.size(); i++) {
    k_link_ids_.insert(std::make_pair(std::make_pair(links[i]->k_source_, links[i]->k_target_), i));

		if (k_outgoing_.find(links[i]->k_source_) == k_outgoing_.end()) {
			// Insert new key, and create new vector as the value
			k_outgoing_.insert(std::make_pair(links[i]->k_source_, std::vector<size_t>(1, links[i]->k_target_)));
		}
		else {
			// Add a new node to the vector
			k_outgoing_[links[i]->k_source_].push_back(links[i]->k_target_);
		}
  }
}

// JJC: edit here to test without NN inputs
matrix1d LinkAgent::actionsToWeights(matrix2d agent_actions) {
  matrix1d weights = easymath::zeros(k_num_edges_);

  for (size_t i = 0; i < k_num_edges_; i++) {
    double predicted = links_.at(i)->predictedTraversalTime();
    if (agent_actions[i][0]>0 && k_output_)
        weights[i] = predicted + agent_actions[i][0] * k_alpha_; // only one action
    else
        weights[i] = predicted;
  }
  return weights;
}
