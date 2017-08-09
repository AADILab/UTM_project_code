#pragma once
#include "yaml-cpp/yaml.h"
#include "FileIO/FileIn.h"

class UTMFileNames {
  public:
    static std::string createDomainDirectory(YAML::Node config) {
      std::string n_sectors = config["constants"]["sectors"].as<std::string>();
      std::string dir_path = "Domains/" + n_sectors + "_Sectors/" + domainNum(config);
        
      std::cout << "Domain path: " << dir_path << "\n" ;

      cio::mkdir_p(dir_path);
      
      std::string dir_path_return = "Domains\/" + n_sectors + "_Sectors\/" + domainNum(config);
      return dir_path_return;
    }

	  static std::string createTrackerDirectory(YAML::Node config) {
		  std::string n_sectors = config["constants"]["sectors"].as<std::string>();
		  std::string dir_path = "Tracker/" + n_sectors + "_Sectors/" + domainNum(config);

		  cio::mkdir_p(dir_path);
		
		  std::string dir_path_return = "Tracker\/" + n_sectors + "_Sectors\/" + domainNum(config);
		  return dir_path_return;
	  }

	  static std::string createMetricsDirectory(YAML::Node config) {
		  std::string n_sectors = config["constants"]["sectors"].as<std::string>();
		  std::string dir_path = "Metrics/" + n_sectors + "_Sectors/" + domainNum(config);

		  cio::mkdir_p(dir_path);
		
		  std::string dir_path_return = "Metrics\/" + n_sectors + "_Sectors\/" + domainNum(config);
		  return dir_path_return;
	  }

    static std::string createExperimentDirectory(std::string config_file) {
      // Creates a directory for the experiment and returns that as a string
      YAML::Node config = YAML::LoadFile("config.yaml");
      std::string agent_defn = config["modes"]["agent"].as<std::string>();
      std::string n_sectors = config["constants"]["sectors"].as<std::string>();
      std::string gen_rate = config["constants"]["generation_rate"].as<std::string>();
      std::string n_steps = config["constants"]["steps"].as<std::string>();
      std::string reward_mode = config["modes"]["reward"].as<std::string>();
      std::string alpha = config["constants"]["alpha"].as<std::string>();

      std::string dir_path = "Experiments/"
        + agent_defn + "_Agents/"
        + n_sectors + "_Sectors/"
        + "Rate_" + gen_rate + "/"
        + n_steps + "_Steps/"
        + reward_mode + "_Reward/"
        + alpha + "_alpha/"
        + domainNum(config);

      // Create new directory
      cio::mkdir_p(dir_path);

      std::string dir_path_return = "Experiments\/"
        + agent_defn + "_Agents\/"
        + n_sectors + "_Sectors\/"
        + "Rate_" + gen_rate + "\/"
        + n_steps + "_Steps\/"
        + reward_mode + "_Reward\/"
        + alpha + "_alpha\/"
        + domainNum(config);

      return dir_path_return;
    }

  private:
    static std::string domainNum(const YAML::Node& config) {
      if (!config["modes"]["numbered_domain"].as<bool>())
        return "";
      std::string domain_num = config["constants"]["domain"].as<std::string>();
      return domain_num + "\/";
    }
};
