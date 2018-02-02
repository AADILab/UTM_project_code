// Copyright 2016 Carrie Rebhuhn
#ifndef SRC_DOMAINS_UTM_LINK_H_
#define SRC_DOMAINS_UTM_LINK_H_

#include <list>
#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>

#include "IAgentBody.h"

#include "STL/easystl.h"

class Link {
  public:
    enum {NOT_ON_LINK};  // link exceptions
    /** Link is created by specifying...
    *  \param source the ID of the source node
    *  \param target the ID of the target node
    *  \param time the time to travel the link
    *  \param capacity how many UAVs may travel on link simultaneously
    *  \param cardinal_dir number which specifies which relative direction link points in
    *  \param window_size the size of the time window (only applicable when using "incoming" state rep);
    *         if set to 0, no scaling occurs
    *  \param cumulative whether or not the link uses a full window (yes if true)
    */
//    Link(size_t id, size_t source, size_t target, size_t time,
//         size_t capacity, size_t cardinal_dir, size_t window_size, bool cumulative);
    // JJC: new constructor for comparative experiments
    Link(size_t id, size_t source, size_t target, size_t time,
         size_t capacity, size_t cardinal_dir, size_t window_size, bool cumulative, size_t cost = 0);

    std::vector<Link*> k_connections_; /**< all links that connect to this link */
    bool atCapacity();
    size_t get_capacity() { return k_capacity_; }
    size_t countIncomingTraffic() {
      size_t count=0;
      for (auto conn : k_connections_) {
        count += conn->countTraffic();
      }
      return count;
    }
    int numOverCapacity();
    std::list<UAV*> traffic_; /**< UAVs on this link */
    size_t countTraffic() {
      return traffic_.size();
    }
    
    /** Initialize link historical data structures (for "incoming" state rep only) */
    void initHist();

    /** Returns the predicted amount of time it would take to cross the link if 
		the UAV got there immediately */
    double predictedTraversalTime();

    /** Grabs the UAV u from link l */
    bool grab(UAV* u, Link* l);

    /** Add UAV to this link. Also sets the time to travel for UAV.
    *  \param u the UAV to add
    */
    bool add(UAV* u);

    /** Remove UAV from this link 
    *  \param u the UAV to remove
    */
    void remove(UAV* u);

    const int k_source_;
    const int k_target_;
    const int k_cardinal_dir_;
	  std::map<size_t, std::deque<size_t> > k_incoming_from_; /**< the history of traffic from sectors with links connected to me (for non-cumulative mode) */
	  std::deque<size_t> k_traffic_hist_; /**< the history of traffic on my link. Only counts UAVs removed from link (for non-cumulative mode) */
	  size_t k_traffic_heist_; /**< the history of traffic on my link. Only counts UAVs removed from link (for cumulative mode) [variable name makes no sense, sorry] */
	  std::map<size_t, size_t> k_incoming_from_santa_; /**< the history of traffic from sectors with links connected to me (for cumulative mode) [variable name makes no sense, sorry] */
	  std::map<size_t, double> k_incoming_prob_; /**< the approximate probability (CORRECTION: ratio) of incoming UAVs to total UAVs that exited particular sector */
    
    /** Reset the link and its data structures */
	  void reset();
	  
	  /** Update the ratios of UAVs that entered from a particular sector vs total UAVs that exited from said sector */
	  void updateTrafficProb();
	  
	  /** Only for non-cumulative mode. Oldest historical information is dropped, new information ready to be recorded. */
	  void slideWindow();
	  
	  /** Get how long it takes to cross link */
	  int get_time() { return time_; } 
	
	  const size_t k_window_size_; /**< how much data the link can collect about incoming traffic and own traffic */
	
	  // Was private member. Ssshhhh.... Don't tell anyone
	  const size_t k_id_; /**< ID of link */
	
	  const bool cumulative_; /**< If true, use cumulative mode (full window) */
	  size_t not_accepted; /**< LAST MINUTE ADDITION how many times link had to reject UAV from entering */
	  size_t delays; /**< LAST MINUTE ADDITION how many times UAVs on this link could not get added to another link */
  private:
    const int time_;  /**< Amount of time it takes to travel across link */
    const int cost_; // JJC: Fixed cost for crossing link
    size_t k_capacity_;  /**< Capacity of link */
    
    /** Called if a UAV is successfully added to this link from link with source node ID src 
    *  \param src the ID of the source node of the link the UAV exited from 
    */
	  void updateIncomingHist(size_t src);
};

/**
* Provides an interface for link agents to interact with the simulator.
* This allows for redefinition of agents in the UTM simulation, and also
* collects information relevant to calculating difference, global, and
* local rewards. Logging of agent actions can also be performed for
* qualitative assessment of behavior.
*/

class LinkAgent : public IAgentBody {
  public:
    /** The agent that communicates with others */
    LinkAgent(size_t num_edges, std::vector<Link*> links, size_t num_state_elements);
    virtual ~LinkAgent() {}
    // weights are ntypesxnagents

    const size_t k_num_edges_;
	
    /**
    * Translates the output of a neural network into costs applied to a link.
    * This can include the 'predicted' cost of the link, which is the
    * traversal time plus the instantaneous wait time at that link. In
    * addition, this translates neural network output, which is in the form
    * [agent #][type #] into weights on the graph, which is in the form
    * [type #][link #]. In the case of link agents, this mapping is
    * agent # = link #, but this is not the case with sector agents.
    * @param agent_actions neural network output, in the form of [agent #][type #]
    * @return the costs for each link in the graph
    */
    virtual matrix1d actionsToWeights(matrix2d agent_actions);

    std::vector<Link*> links_;
    std::map<std::pair<size_t, size_t>, size_t> k_link_ids_; /**< Maps source and target node to link ID */
	
	  std::map<size_t, std::vector<size_t> > k_outgoing_; /**< holds a source node number as key and vector of target nodes as value */

    size_t getNthLink(UAV* u, size_t n) {
      return k_link_ids_[u->getNthEdge(n)];
    }

	  size_t getCurLink(UAV* u) {
		  return k_link_ids_[u->getCurEdge()];
	  }

    /** Computes the state of the link agent. State can be only traffic on this link, or could also
    *  include (scaled or not) number of incoming UAVs
    *  \param uavs the list of all UAVs in the airspace
    *  \return matrix of states of agents; each row contains state of one agent
    */
    matrix2d computeCongestionState(const std::list<UAV*>& uavs) {
      size_t num_agents = links_.size();
      matrix2d all_states = easymath::zeros(num_agents, k_num_states_);
      for (UAV* u : uavs) {
			  size_t curLink = getCurLink(u);
			  size_t curSect = u->getCurSector();
			  // traffic on current link
        all_states[curLink][0]++;
		  }

		  if (k_num_states_ == 2) {
			  for (size_t l = 0; l < links_.size(); l++){
				  auto src = links_[l]->k_source_;
				  // get vector of sectors, with links that may possibly 
				  //  contribute to traffic for current link
				  std::vector<size_t> outgoing = k_outgoing_[src];
				  for (size_t i = 0; i < outgoing.size(); i++) {
					  std::pair<size_t, size_t> incomingEdge = std::make_pair(outgoing[i], src);
					  // get ID of one of those links
					  size_t temp = k_link_ids_[incomingEdge];
					  // count traffic on that link and add to current link's state
					  all_states[l][1] += static_cast<double>(links_[temp]->countTraffic()) * links_[l]->k_incoming_prob_[links_[temp]->k_source_];
				  }
			  }
		  }

      agent_states_.push_back(all_states);
      return all_states;
    }
};
#endif  // SRC_DOMAINS_UTM_LINK_H_
