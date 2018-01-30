#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 26 11:27:11 2017

@author: Brandon
"""
import sys
import os
import numpy as np

# Take data of multiple runs and write averages to file
# data is numpy array of floats (first index is row)
def summarize(data, filename, numruns):

    avg = np.mean(data, 0)
    std = np.std(data, 0) / np.sqrt(numruns)
        
    print ("Writing to " + filename)
        
    with open(filename, 'a+') as myf:
        # Write csv headers
        myf.write('Epoch,Average,Std\n')
        for i in range(avg.size):
            myf.write(str(i+1) + ',' + str(avg[i]) + ',' + str(std[i]) + '\n')

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print "Please invoke as follows:\n./summarize.py <path to metrics folder>"
        sys.exit(-1)
        
    path = sys.argv[1]

    # Clean up old summaries
    file_moving = os.path.join(path,'moving.csv')
    file_delay = os.path.join(path,'delay.csv')
    file_ground_hold = os.path.join(path,'ground_hold.csv')
    file_travel_time = os.path.join(path,'travel_time.csv')
    file_final = os.path.join(path,'final_mean_std.csv')
    try:
        print('Removing {0}'.format(file_moving))
        os.remove(file_moving)
        print('Removing {0}'.format(file_delay))
        os.remove(file_delay)
        print('Removing {0}'.format(file_ground_hold))
        os.remove(file_ground_hold)
        print('Removing {0}'.format(file_travel_time))
        os.remove(file_travel_time)
        print('Removing {0}'.format(file_final))
        os.remove(file_final)
    except OSError:
        print('There are no old summaries to remove...')
    
    prefix = 'metrics_'
    files = [f for f in os.listdir(path)
             if os.path.isfile(os.path.join(path,f))
             and f.startswith(prefix)
             and f.endswith('.csv')]
    numFiles = len(files)
             
    moving = np.empty([0,0])
    delay = np.empty([0,0])
    ground_hold = np.empty([0,0])

    for i in range(numFiles):
        all_stuff = []
        with open(os.path.join(path,prefix+str(i)+".csv"), 'r') as myf:
            for line in myf:
                stuff = line.split(',')
                stuff = map(float, stuff[:-1])
                all_stuff.append(stuff)

        data = np.array(all_stuff)
        if moving.size == 0:
            moving = np.reshape(data[0:,0], (data[0:,0].size, 1))
            delay = np.reshape(data[0:,1], (data[0:,1].size, 1))
            ground_hold = np.reshape(data[0:,2], (data[0:,2].size, 1))
            print moving.shape
        else:
            moving = np.concatenate((moving, np.reshape(data[0:,0], (data[0:,0].size, 1))), axis=1)
            delay = np.concatenate((delay, np.reshape(data[0:,1], (data[0:,1].size, 1))), axis=1)
            ground_hold = np.concatenate((ground_hold, np.reshape(data[0:,2], (data[0:,2].size, 1))), axis=1)
    moving = moving.T
    delay = delay.T
    ground_hold = ground_hold.T
    summarize(moving, file_moving, numFiles)
    summarize(delay, file_delay, numFiles)
    summarize(ground_hold, file_ground_hold, numFiles)
    travel_time = moving + delay + ground_hold
    summarize(travel_time, file_travel_time, numFiles)

    moving_final_avg = np.mean(moving[0:, -1])
    moving_final_std = np.std(moving[0:, -1])
    delay_final_avg = np.mean(delay[0:, -1])
    delay_final_std = np.std(delay[0:, -1])
    ground_final_avg = np.mean(ground_hold[0:, -1])
    ground_final_std = np.std(ground_hold[0:, -1])

    print('Writing to {}'.format(file_final))
                
    with open(file_final, 'a+') as myf:
        myf.write(str(moving_final_avg) + ',' + str(delay_final_avg) + ',' + str(ground_final_avg) + '\n' +
                  str(moving_final_std) + ',' + str(delay_final_std) + ',' + str(ground_final_std) + '\n')

