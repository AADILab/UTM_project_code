OSU UTM Project Code
Authors: Carrie Rebhuhn, Brandon Gigious, Jen Jen Chung, 2017

Description:

This package contains general libraries for performing neuro-evolution of a multiagent system and specific libraries for simulating a multiagent UAV traffic management system (Domains/UTM).

The package also incorporates the yaml-cpp library from https://github.com/jbeder/yaml-cpp for reading in the simulation configurations from build/config.yaml at the start of the program.

Use:

The package is set up to be cmake compatible on a linux-based system. To compile, navigate to the build folder from the command line, run:

  cmake ..
  make

to generate AbstractUTMSimulation. Simulation parameters can be changed through the configuration file and new traffic graphs can be included by saving the necessary domain files into a build/Domains/X_Sectors folder, where X corresponds to the number of nodes in the graph. See the associated documentation in UTM_Documentation.pdf for more details on the individual elements of the configuration file.

Post-processing data:

The python file pretty.py is used to transform the logged traffic metrics into a format that can then be drag into MJO graph (https://sourceforge.net/projects/mjograph/). To run from the command line:

python pretty.py build/Metrics/X_Sectors/metrics_ Y

where X corresponds to the number of nodes in the graph and Y corresponds to the number of statistical runs (this should match the number of metrics_ files in the folder).

The matlab file replay_utm_log.m plots the motion of the UAVs through the graph and also displays the output costs of the link agents as well as their current capacity. The directory from which to read in the log files are defined at the top of the script.
