# OSU UTM Project Code
Authors: Carrie Rebhuhn, Brandon Gigious, Jen Jen Chung, 2017

## Description:

This package contains general libraries for performing neuro-evolution of a multiagent system and specific libraries for simulating a multiagent UAV traffic management system (Domains/UTM).

The package also incorporates the yaml-cpp library from https://github.com/jbeder/yaml-cpp for reading in the simulation configurations from build/config.yaml at the start of the program.

Use:

The package is set up to be cmake compatible on a linux-based system. To compile, navigate to the build folder from the command line, run:

  cmake ..
  make

to generate AbstractUTMSimulation. Simulation parameters can be changed through the configuration file and new traffic graphs can be included by saving the necessary domain files into a build/Domains/X_Sectors folder, where X corresponds to the number of nodes in the graph. See the associated documentation in UTM_Documentation.pdf for more details on the individual elements of the configuration file.

## Post-processing data

The post processing tools use the Python Pandas and Matplotlib libraries. On Ubuntu, these
can be installed by
```
sudo apt install python-pandas python-matplotlib
```

Performance metrics from all experimental runs can be aggregated
using:
```
./aggregate_metrics.py <path to folder containing metrics_Y.csv files>
```
To plot the aggregated metrics, use:
```
./plot_aggregate_metrics.py <path to metrics_aggregate.csv file>
```

The matlab file replay_utm_log.m plots the motion of the UAVs through the graph and also displays the output costs of the link agents as well as their current capacity. The directory from which to read in the log files are defined at the top of the script.
