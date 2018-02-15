// Copyright 2016 Carrie Rebhuhn
#include "SimNE.h"

using std::vector;
using easymath::operator-;
using std::pair;
using std::make_pair;

void SimNE::print_nets(MultiagentNE* MAS, size_t run) {
	for (size_t i = 0; i < MAS->agents.size(); i++) {
		std::string file_out = "neural_nets/net_" + std::to_string(run) + "_" + std::to_string(i) + ".csv";
		MAS->agents.at(i)->save(file_out);
	}
	// exit(1);
}

SimNE::SimNE(IDomainStateful* domain, MultiagentNE* MAS, YAML::Node configs, std::string metric_filepath) :
  ISimulator<NeuroEvo>(domain, MAS), MAS(MAS), metric_filepath(metric_filepath) {
  T = domain->T;
	// Load some config settings
  fitness_metric = configs["modes"]["fitness"].as<std::string>();
	record_enabled = configs["modes"]["prob_record"].as<bool>();
	objective = configs["modes"]["objective"].as<std::string>();
	tracking_enabled = configs["modes"]["tracking"].as<bool>();
  auto tracking_type = configs["modes"]["tracking_type"].as<std::string>();
  if (tracking_type == "all")
    tracking_all_enabled = true ;
  else
    tracking_all_enabled = false ;
}

void SimNE::runExperiment() {
	do {
	  if (tracking_enabled){
	    tracking = true ;
	    if (!tracking_all_enabled && (T->epoch != 0 && T->epoch != T->MAX_EPOCH-1))
	      tracking = false ;
    }
    else
      tracking = false ;
    
    if (tracking)
      std::cout << "Logging simulation data for playback.\n" ;
      
		timer.start();
		this->epoch();
		timer.stop();

		printf("Epoch %i. ", T->epoch);
		timer.printTimePassed();
		timer.printEndEstimate(T->MAX_EPOCH - T->epoch);
		printf("\n");
	} while (T->iterateEpochs());
}

void SimNE::runExperimentTest() { // Test trained MAS
	do {
	  if (tracking_enabled){
	    tracking = true ;
	    if (!tracking_all_enabled && (T->epoch != 0 && T->epoch != T->MAX_EPOCH-1))
	      tracking = false ;
    }
    else
      tracking = false ;
    
    if (tracking)
      std::cout << "Logging simulation data for playback.\n" ;
      
		timer.start();
		this->epochTest();
		timer.stop();

		printf("Epoch %i. ", T->epoch);
		timer.printTimePassed();
		timer.printEndEstimate(T->MAX_EPOCH - T->epoch);
		printf("\n");
	} while (T->iterateEpochs());
}

void SimNE::outputLearningLog(std::string filename)
{
	//always overwrite
	std::ofstream file;
	file.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

	if (file.is_open()) {
		for (size_t ep = 0; ep < T->MAX_EPOCH; ep++) {
			file << "Epoch " << ep;
			// First write mean squared difference btwn parent and child
			for (int a = 0; a < mutation_difference[ep].size(); a++)
			{
				file << ",";
				for (int m = 0; m < mutation_difference[ep][a].size(); m++)
				{
					file << mutation_difference[ep][a][m] << ",,";
				}
				file << "\n";
			}
			// Then write weight ranges
			for (int a = 0; a < weight_ranges[ep].size(); a++)
			{
				file << survivor_persistence[ep][a] << ",";
				for (int m = 0; m < weight_ranges[ep][a].size(); m++)
				{
					file << weight_ranges[ep][a][m][0] << "," << weight_ranges[ep][a][m][1] << ",";
				}
				file << "\n";
			}
		}
		file.close();
		printf("... Successfully wrote to file %s.\n", filename.c_str());
	}
	else {
		printf("Failed to open %s.\n", filename.c_str());
	}
}

void SimNE::outputEvalLog(std::string filename)
{
	//always overwrite
	std::ofstream file;
	file.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

	if (file.is_open()) {
		for (size_t ep = 0; ep < T->MAX_EPOCH; ep++) {
			// Write fitness values
			for (int a = 0; a < evaluations[ep].size(); a++)
			{
				for (int m = 0; m < evaluations[ep][a].size(); m++)
				{
					file << evaluations[ep][a][m][0] << ",";
				}
				file << "\n";
				for (int m = 0; m < evaluations[ep][a].size(); m++)
				{
					file << evaluations[ep][a][m][1] << ",";
				}
				file << "\n";
			}
			file << "\n";
		}
		file.close();
		printf("... Successfully wrote to file %s.\n", filename.c_str());
	}
	else {
		printf("Failed to open %s.\n", filename.c_str());
	}
}

void SimNE::outputExtraMetrics(std::string directory)
{
	cio::print2(extra_metric_log, directory + "metrics_" + std::to_string(T->run) + ".csv");
//	cio::print3(extra_extra_metric_log, "extra_extra_" + std::to_string(T->run) + ".csv");
}

SimNE::SimHistory SimNE::simGlobal() {
  size_t N = MAS->agents.size();
  SimNE::SimHistory SH = SimNE::SimHistory(T->MAX_STEP,N);

	matrix3d S_hist;
	matrix3d A_hist;

  do {
    matrix2d S = domain->getStates();
    matrix2d A = this->getActions(S);
    
    double g = domain->simulateStep(A);
    matrix1d extra = domain->getExtraInfo(); // fourth element inside extra stores total UAVs up to current step (OR total missions assigned up to current step), fifth element stores total UAVs that have reached destination up to current step (OR total successful missions up to current step)
    SH.addState(S,T->step);
    SH.addAction(A, T->step);

    S_hist.push_back(S);
    A_hist.push_back(A);

    SH.G[T->step] = g;
    SH.E[T->step] = extra;

  } while (T->iterateSteps());
	
	SH.EE = domain->getExtraExtraInfo(); // Again, you should ignore this
	if (record_enabled) {
		domain->printUavGen(); // Print out files for playing back UAV generation later
	}
	if (tracking) {
		domain->printTrackingInfo();
		//cio::print2(SH.E, "infoextra_run" + std::to_string(T->run) + "_eval" + std::to_string(T->eval) + ".csv");
		//cio::print3(S_hist, "S_run" + std::to_string(T->run) + "_epoch" + std::to_string(T->epoch) + "_eval" + std::to_string(T->eval) + ".csv");
		//cio::print3(A_hist, "A_run" + std::to_string(T->run) + "_epoch" + std::to_string(T->epoch) + "_eval" + std::to_string(T->eval) + ".csv");
	}
	if (record_enabled || tracking)
		T->eval++;

  return SH;
}

void SimNE::epoch() {

  MAS->generate_new_members();
  //print_nets(MAS, T->run);
  SimNE::accounting accounts = SimNE::accounting();

  matrix2d best_W;
  matrix2d best_S;
  double best_G=-1000000;
  int n = 0 ;
  do {
    SimNE::SimHistory SH;
    matrix1d R;
    double G;
    size_t U ;
    
    if (fitness_metric == "difference_resim_A") {
      pair<matrix1d, double> DG = simDifferenceResimA();
      R = DG.first;
      G = DG.second;
		  U = SH.E[T->step -1][3] ; // total assigned missions in epoch
    } else if (fitness_metric == "difference_resim_P") {
      pair<matrix1d, double> DG = simDifferenceResimP();
      R = DG.first;
      G = DG.second;
		  U = SH.E[T->step -1][3] ; // total assigned missions in epoch
	  } else {
		  SH = simGlobal();
		  G = easymath::sum(SH.G);
		  U = SH.E[T->step -1][3] ; // total assigned missions in epoch
		  R = domain->getRewards();
	  }
	  
    //G = -G;
    //R = matrix1d(R.size(), 0.0) - R; ????

    // JJC edit: best performance, etc. all associated with R = G/(completed_missions) i.e. total UAV time divided by number of completed missions
//    if (accounts.update(R, G/(double)U, n))
    double R_out = R[0] ;
    if (accounts.update(R, R_out, n))
		  accounts.update_extra(SH.E, SH.EE, T->eval);

    if (G > best_G) {
      best_W = domain->actionHistory();
      best_S = domain->stateHistory();
    }

    domain->printMaxMinAction();
    domain->reset(); // reset domain for next team
    MAS->update_policy_values(R);
    
    n++ ;
  } while (MAS->set_next_pop_members());
  
	T->eval = 0;
	evaluations.push_back(MAS->get_evals());
	if (T->epoch == T->MAX_EPOCH-1)
	  MAS->select_survivors(false); // don't shuffle on last epoch
	else
  	MAS->select_survivors(); // populations get shuffled here
  survivor_persistence.push_back(MAS->get_survivor_persistence());
	mutation_difference.push_back(MAS->get_msds());
	weight_ranges.push_back(MAS->get_ranges());
	
  // cio::print2(best_W, "weights" + std::to_string(ep) + ".csv");
  // cio::print2(best_S, "states" + std::to_string(ep) + ".csv");

  reward_log.push_back(accounts.best_run);
  metric_log.push_back(accounts.best_run_performance);
	extra_metric_log.push_back(accounts.best_extra);
	extra_extra_metric_log.push_back(accounts.best_extra_extra);
}

void SimNE::epochTest() { // Test trained MAS

  SimNE::accounting accounts = SimNE::accounting();

  matrix2d best_W;
  matrix2d best_S;
  double best_G=-1000000;
  int n = 0 ;
  do {
    SimNE::SimHistory SH;
    matrix1d R;
    double G;
    size_t U ;
    
    if (fitness_metric == "difference_resim_A") {
      pair<matrix1d, double> DG = simDifferenceResimA();
      R = DG.first;
      G = DG.second;
		  U = SH.E[T->step -1][3] ; // total UAVs in epoch
    } else if (fitness_metric == "difference_resim_P") {
      pair<matrix1d, double> DG = simDifferenceResimP();
      R = DG.first;
      G = DG.second;
		  U = SH.E[T->step -1][3] ; // total UAVs in epoch
	  } else {
		  SH = simGlobal();
		  G = easymath::sum(SH.G);
		  U = SH.E[T->step -1][3] ; // total UAVs in epoch
		  R = domain->getRewards();
	  }
	  
    double R_out = R[0] ;
    if (accounts.update(R, R_out, n))
		  accounts.update_extra(SH.E, SH.EE, T->eval);

    if (G > best_G) {
      best_W = domain->actionHistory();
      best_S = domain->stateHistory();
    }

    domain->printMaxMinAction();
    domain->reset(); // reset domain for next team
    
    n++ ;
  } while (MAS->set_next_pop_members());

  reward_log.push_back(accounts.best_run);
  metric_log.push_back(accounts.best_run_performance);
	extra_metric_log.push_back(accounts.best_extra);
	extra_extra_metric_log.push_back(accounts.best_extra_extra);
}

pair<matrix1d, double> SimNE::simDifferenceResimA() {
    auto SH = simGlobal();
    matrix2d ac = matrix2d(SH.T,matrix1d(domain->getNumNNOutputs(),0));  // counterfactual action

    matrix1d G = matrix1d(SH.N, easymath::sum(SH.G));
    matrix1d Gc = matrix1d(SH.N);
    for (size_t i = 0; i < SH.N; i++) {
        auto A = SH.A;
        A[i] = ac;
        Gc[i] = domain->simulateSteps(A);
    }
    matrix1d D = G - Gc;

    return make_pair(D, G[0]);
}

pair<matrix1d, double> SimNE::simDifferenceResimP() {
  auto SH = simGlobal();
  matrix1d ac = matrix1d(domain->getNumNNOutputs(),0);  // counterfactual action

  matrix1d G = matrix1d(SH.N, easymath::sum(SH.G));
  matrix1d Gc = matrix1d(SH.N);
  for (size_t i = 0; i < SH.N; i++) {
    do {
      matrix2d S = domain->getStates();
      matrix2d A = this->getActions(S);
      A[i] = ac;

      double g = domain->simulateStep(A);
      SH.G[T->step] = g;
    } while (T->iterateSteps());

    Gc[i] = easymath::sum(SH.G);
  }
  matrix1d D = G - Gc;

  return make_pair(D, G[0]);
}

pair<matrix1d, double> SimNE::simDifferenceApprox(std::vector<NeuralNet>* DNN) {
  auto SH = simGlobal();
  matrix1d ac = matrix1d(domain->getNumNNOutputs(), 0);  // counterfactual action

  matrix1d G = matrix1d(SH.N, easymath::sum(SH.G));
  matrix1d Gc = matrix1d(SH.N,0.0);

  for (size_t i = 0; i < SH.N; i++) {
    do{
      matrix2d S = domain->getStates();
      matrix2d A = this->getActions(S);

      matrix1d sai = S[i];
      sai.insert(sai.end(), A[i].begin(), A[i].end());

      double g = domain->simulateStep(A);
      SH.G[T->step] = g;

      DNN->at(i).backProp(sai, matrix1d(1, g));


      A[i] = ac;
      matrix1d sac = S[i];
      sac.insert(sac.end(), A[i].begin(), A[i].end());

      Gc[i]+=DNN->at(i)(sac)[0];
    } while (T->iterateSteps());
  }
  matrix1d D = G - Gc;

  return make_pair(D, G[0]);
}

vector<Action> SimNE::getActions(vector<State> S) {
  return MAS->getActions(S);
}

SimNE::accounting::accounting() {
  best_run = -std::numeric_limits<double>::max();
  best_run_performance = -std::numeric_limits<double>::max();
  best_perf_idx = -1;
	best_update = false;
}

bool SimNE::accounting::update(const vector<double> &R, const double &perf, int n) {
  //double avg_G = easymath::mean(R);
  double avg_R = easymath::mean(R);
  best_run = avg_R;
  //if (avg_G > best_run) {
  //    best_run = avg_G;
  //}
  if (perf > best_run_performance) {
    best_run_performance = perf;
    best_perf_idx = n;
    best_update = true;
  }

  printf("Current team #%i, %f. Best team #%i, %f\n", n, perf, best_perf_idx, best_run_performance);

  return best_update;
}

void SimNE::accounting::update_extra(const matrix2d &extra, const matrix2d &extraExtra, size_t eval) {
	//
	// EXTREMELY hacky, sorry I need results...
	//
	//cio::print2(extra, "extra_eval_" + std::to_string(eval) + ".csv");
	if (best_update && extra.size()) {
		matrix1d sum_extra = easymath::zeros(extra[0].size());
		size_t i;
		for (i = 0; i < extra.size(); i++) {
			for (size_t j = 0; j < extra[0].size() - 2; j++) // extra info has 5 fields remember. We're only iterating through first 3
				sum_extra[j] += extra[i][j]; // This is where delay time, moving time, wait time calculated for 			
		}
		sum_extra[extra[0].size() - 2] = extra[i-1][extra[0].size() - 2]; // Total UAVs
		sum_extra[extra[0].size() - 1] = extra[i-1][extra[0].size() - 1]; // UAVs that reached destination
		
		best_extra = sum_extra;
		best_extra.push_back(best_perf_idx) ;
		best_extra_extra = extraExtra;
	}
	best_update = false;
}
