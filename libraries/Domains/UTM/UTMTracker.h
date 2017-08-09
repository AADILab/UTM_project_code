#pragma once
#include "Simulation/SimTime.h"
#include "FileIO/FileIn.h"
#include "UAV.h"

// Handles tracking and output of variables
class UTMTracker {
public:
  UTMTracker(SimTime* t, size_t num_agents, size_t num_sectors): num_agents(num_agents) {
    T = t;
    /* initialize all matrices and such */
    states = matrix2d(T->MAX_STEP);
    actions = matrix2d(T->MAX_STEP);
    weights = matrix2d(T->MAX_STEP);
    uav_pos = matrix2d(T->MAX_STEP);
    uav_cur_sectors = matrix2d(T->MAX_STEP);
    uav_end_sectors = matrix2d(T->MAX_STEP);
    uav_count = 0;
    uav_wait = matrix2d(T->MAX_STEP);
    uav_wait_path = std::vector<std::vector<std::string> >(T->MAX_STEP);
    uav_cur_path = std::vector<std::vector<std::string> >(T->MAX_STEP);
    uav_gen = matrix2d(num_sectors);
    prefix = "Tracker/" + std::to_string(num_sectors) + "_Sectors/";
    prefix2 = "Domains/" + std::to_string(num_sectors) + "_Sectors/";
  }

	/* Record info from simulating domain for one timestep.
	 *  \param S states of agents
	 *  \param A actions of agents
	 *	\param W weights of agents
	 *	\param uavs UAVs currently traveling in the system
	 *  \param wait UAVs currently in system, but waiting at gen node
	 *  \param uav_count current UAV count (Includes absorbed UAVs)
  */
  
  void stepUpdate(matrix2d S, matrix2d A, matrix1d W, std::list<UAV*> uavs, std::list<UAV*> wait, size_t uav_count) {
  // Should be relatively easy to read this code and understand what it's doing. If not, let me know. -B
  int t = T->step;
  for (int i = 0; i < num_agents; i++) {
    states[t].push_back(S[i][0]);
    actions[t].push_back(A[i][0]);
    weights[t].push_back(W[i]);
  }

  matrix1d step_positions, end_sectors, cur_sectors;
  std::vector<std::string> cur_path;
  for (auto u : uavs) {
    step_positions.push_back(u->getId());
    step_positions.push_back(u->getCurEdgeID());

	  end_sectors.push_back(u->getId());
	  end_sectors.push_back(u->end_sector_);
	
	  cur_sectors.push_back(u->getId());
	  cur_sectors.push_back(u->getCurSector());

	  cur_path.push_back(std::to_string(u->getId()));
	  std::list<size_t> pth = u->getPath();
	  std::string strPath = "";
	  size_t sz = pth.size();
	  for (int i = 0; i < sz; i++)
	  {
		  strPath += std::to_string(pth.front()) + ';';
		  pth.pop_front();
	  }
	  cur_path.push_back(strPath);
  }
  matrix1d wait_pos;
  std::vector<std::string> wait_path;
  for (auto u : wait) {
	  wait_pos.push_back(u->getId());
	  wait_pos.push_back(u->getCurSector());

	  wait_path.push_back(std::to_string(u->getId()));
	  std::list<size_t> pth = u->getPath();
	  std::string strPath = "";
	  size_t sz = pth.size();
	  for (int i = 0; i < sz; i++)
	  {
		  strPath += std::to_string(pth.front()) + ';';
		  pth.pop_front();
	  }
	  wait_path.push_back(strPath);
  }
  uav_wait[t] = wait_pos;

  uav_pos[t] = step_positions;
  uav_end_sectors[t] = end_sectors;
  uav_cur_sectors[t] = cur_sectors;
  matrix1d temp;
  temp.push_back(uav_pos[t].size() / (int)2);

  uav_wait_path[t] = wait_path;
  uav_cur_path[t] = cur_path;

  this->uav_count = uav_count;
  }

	/** Print various tracking info to several files. Note these files get saves with names reflecting which run, which eval, which epoch */
  void print() {
    cio::print2<double>(uav_pos, prefix + "uav_pos_" + filename() + ".csv");
	  cio::print2<double>(uav_wait, prefix + "uav_wait_" + filename() + ".csv");
	  cio::print2<double>(uav_end_sectors, prefix + "uav_end_sectors_" + filename() + ".csv");
	  //cio::print2<double>(uav_cur_sectors, "Tracker/uav_cur_sectors_" + filename() + ".csv");
    cio::print2<double>(states, prefix + "states_" + filename() + ".csv");
    cio::print2<double>(weights, prefix + "weights_" + filename() + ".csv");
    //cio::print2<double>(actions, "Tracker/actions_" + filename() + ".csv");
	  //matrix1d cnt = matrix1d(1, (double)uav_count);
	  //cio::print<double>(cnt, prefix + "uav_count_" + filename() + ".csv");
	  cio::print<double>(capacities, prefix + "capacities.csv"); // Not really efficient to print this each time
	  cio::print<double>(edgeTimes, prefix + "edgeTime.csv");    // But too lazy to do anything about it right now. :)
	  cio::print2<std::string>(uav_wait_path, prefix + "wait_path_" + filename() + ".csv");
	  //cio::print2<std::string>(uav_cur_path, prefix + "cur_path_" + filename() + ".csv");
  }

	/** Print info on where UAVs got generated. Files can be used later to "replay" UAV generation when comparing experiments. */
	void printGen() {
		cio::print2<double>(uav_gen, prefix2 + "uav_gen_" + filename() + ".csv");
	}

	/** Clear variables used for tracking (occurs once every simulation) */
  void clear() {
	  states = matrix2d(T->MAX_STEP);
	  actions = matrix2d(T->MAX_STEP);
	  weights = matrix2d(T->MAX_STEP);
	  uav_pos = matrix2d(T->MAX_STEP);
	  uav_wait = matrix2d(T->MAX_STEP);
	  uav_end_sectors = matrix2d(T->MAX_STEP);
	  uav_cur_sectors = matrix2d(T->MAX_STEP);
	  uav_count = 0;
	  uav_wait_path = std::vector<std::vector<std::string> >(T->MAX_STEP);
	  uav_cur_path = std::vector<std::vector<std::string> >(T->MAX_STEP);
	  uav_gen = matrix2d(uav_gen.size());
  }

  matrix2d getActions() {
    return actions;
  }
  matrix2d getStates() {
    return states;
  }

	void updateGenUavs(std::vector<size_t> num_gen) {
		for (size_t s = 0; s < num_gen.size(); s++) {
			uav_gen[s].push_back(num_gen[s]);
		}
	}

	matrix1d capacities;
	matrix1d edgeTimes;
private:
  std::string filename() {
    return "run" + std::to_string(T->run) + "_epoch" + std::to_string(T->epoch) + "_eval" + std::to_string(T->eval);
  }
  size_t num_agents;			    /**< how many learning agents exist in domain */
	std::string prefix;			    /**< Path to tracker directory, used as prefix for writing files */
	std::string prefix2;		    /**< Path to domain directory, used as prefix for writing files */
  matrix2d uav_pos;           /**< [uavID, linkID, uavID, linkID... ] */
  matrix2d states;            /**< States given to the agents [timestep x agent] */
  matrix2d weights;           /**< Graph weights (NOT agent outputs) */
  matrix2d actions;           /**< Direct agent outputs */
	matrix2d uav_end_sectors;   /**< Where each UAV's goal is (only for existing UAVs) */
	matrix2d uav_cur_sectors;	  /**< Each UAVs current sector (only for existing UAVs) */
	size_t uav_count;			      /**< How many UAvs have been generated */
	matrix2d uav_wait;			    /**< Which generated UAVs are still waiting at origin sector */
	std::vector<std::vector<std::string> > uav_wait_path; /**< Where waiting UAVs plan to go (shown as: "UAV ID, 1st sector;2nd sector;and so on..." */
	std::vector<std::vector<std::string> > uav_cur_path;  /**< Where traveling UAVs plan to go (shown as: "UAV ID, 1st sector;2nd sector;and so on..." */
	matrix2d uav_gen;			/**< How many UAVs get generated and which sector. First index corresponds to sector ID, 
									 second to how many UAVs have gen'd from it this timestep */
  SimTime* T;
};
