#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Aggregate the metrics of several independent evolutionary runs which optimize NN traffic controllers.
"""
import sys
import os
import pandas as pd

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print "Please invoke as follows:\n./aggregate_metrics.py <path to metrics folder>"
        sys.exit(-1)
        
    path = sys.argv[1]
    
    prefix = 'metrics_'
    files = [f for f in os.listdir(path)
             if os.path.isfile(os.path.join(path,f))
             and f.startswith(prefix)
             and f.endswith('.csv')
             and f.find('aggregate') < 0]

    num_runs = len(files)
    data = dict([['moving', pd.DataFrame(columns=range(num_runs))],
                 ['delay', pd.DataFrame(columns=range(num_runs))],
                 ['ground_hold', pd.DataFrame(columns=range(num_runs))],
                 ['total_travel', pd.DataFrame(columns=range(num_runs))],
                 ['assigned_missions', pd.DataFrame(columns=range(num_runs))],
                 ['completed_missions', pd.DataFrame(columns=range(num_runs))],
                 ['cost', pd.DataFrame(columns=range(num_runs))]])

    # Read metrics files for all runs
    # Each metric is read into a separate data frame
    for run in range(num_runs):
        data_run = pd.read_csv(os.path.join(path,prefix+str(run)+'.csv'), header=None)
        data['moving'][run] = data_run[0]
        data['delay'][run] = data_run[1]
        data['ground_hold'][run] = data_run[2]
        data['total_travel'][run] = data_run[0] + data_run[1] + data_run[2]
        data['assigned_missions'][run] = data_run[3]
        data['completed_missions'][run] = data_run[4]
        data['cost'][run] = data['total_travel'][run] / data['completed_missions'][run] 

    aggregate = pd.DataFrame(columns=map(lambda s: s+'_mean',data.keys())
                             +map(lambda s: s+'_std', data.keys()))
    
    for key in data:
        aggregate[key+'_mean'] = data[key].mean(axis=1)
        aggregate[key+'_std'] = data[key].std(axis=1)
    aggregate.index.name = 'epoch'
    
    file_aggregate = os.path.join(path,'metrics_aggregate.csv')
    aggregate.to_csv(file_aggregate)
