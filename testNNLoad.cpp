//#include <iostream>
//#include <string>
//#include <vector>

//#include "Learning/NeuralNet.h"
//#include "FileIO/FileIn.h"
//#include "yaml-cpp/yaml.h"

//int main(){
//  std::string config_file = "config.yaml";
//  
//	// Read some parameters from config file
//  YAML::Node configs = YAML::LoadFile(config_file);
//  size_t nPop = configs["neuroevo"]["popsize"].as<size_t>();
//  double alpha = configs["constants"]["alpha"].as<double>();
//  size_t nAgents = 18 ;
//  
//  std::vector<double> X ;

//  for (size_t i = 0; i < nAgents; i++){
//    NeuralNet nn(configs["neuroevo"],0) ;
//    char buffer[50] ;
//    sprintf(buffer,"neural_nets/6_Sectors13/net_0_%i.csv",i) ;
//    nn.load(buffer) ;
//    for (size_t k = 0; k < 4; k++){
//      X.clear() ;
//      X.push_back((double)k) ;
//      std::vector<double> Y = nn(X) ;
//      std::cout << "NN(" << X[0] << "): " << Y[0]*alpha << ", " ;
//    }
//    std::cout << "\n" ;
//  }
//}

#include <iostream>
#include <string>
#include <vector>

#include "Learning/NeuralNet.h"
#include "FileIO/FileIn.h"
#include "yaml-cpp/yaml.h"

int main(){
  std::string config_file = "config.yaml";
  
	// Read some parameters from config file
  YAML::Node configs = YAML::LoadFile(config_file);
  size_t nPop = configs["neuroevo"]["popsize"].as<size_t>();
  double alpha = configs["constants"]["alpha"].as<double>();
  size_t nAgents = 56 ;
  
  std::vector<double> X ;

  for (size_t ii = 0; ii < nAgents; ii++){
    std::cout << "Agent: " << ii << "\n" ;
    NeuralNet nn(configs["neuroevo"],0) ;
    char buffer[500] ;
    sprintf(buffer,"neural_nets/net_0_%i.csv",ii) ;
    nn.load(buffer) ;
  
//    std::cout << "layers_.size(): " << nn.layers_.size() << "\n" ;
//    for (size_t i = 0; i < nn.layers_.size(); i++) {
//      std::cout << "layers_[" << i << "].w_bar_.size(): " << nn.layers_[i].w_bar_.size() << "\n" ;
//      for (size_t j = 0; j < nn.layers_[i].w_bar_.size(); j++){
//        std::cout << "layers_[" << i << "].w_bar_[" << j << "].size(): " << nn.layers_[i].w_bar_[j].size() << "\n" ;
//        for (size_t k = 0; k < nn.layers_[i].w_bar_[j].size(); k++){
//          std::cout << nn.layers_[i].w_bar_[j][k] << " " ;
//        }
//        std::cout << "\n" ;
//      }
//    }
    
    
    for (size_t k = 0; k < 2; k++){
      X.clear() ;
      X.push_back((double)k) ;
      std::vector<double> Y = nn(X) ;
      std::cout << "NN(" << X[0] << "): " << Y[0]*alpha << ", " ;
    }
    std::cout << "\n" ;
  }
}
