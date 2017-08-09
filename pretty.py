# -*- coding: utf-8 -*-
"""
Created on Wed Apr 26 11:27:11 2017

@author: Brandon
"""
import sys
import numpy as np

# Take data of multiple runs and spit out mjo-friendly file
# data is numpy array of floats (first index is row)
def mjo_readyize(data, filename, numruns):
	
	avg = np.mean(data, 0)
	std = np.std(data, 0) / np.sqrt(numruns)
	
	print "Writing to " + filename + ".csv" ;
	
	with open(filename + '.csv', 'a+') as myf:
		for i in range(avg.size):
			myf.write(str(i+1) + ',' + str(avg[i]) + ',' + str(std[i]) + '\n')

if __name__ == '__main__':
	
	if len(sys.argv) < 3:
		print "error"
		sys.exit(-1)
	
	pre = sys.argv[1]
	numFiles = int(sys.argv[2])
	
	if pre.endswith('metrics_'):
		moving = np.empty([0,0])
		delay = np.empty([0,0])
		ground_hold = np.empty([0,0])

		for i in range(numFiles):
			all_stuff = []
			with open(pre + str(i) + ".csv", 'r') as myf:
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
		mjo_readyize(moving, 'moving', numFiles)
		mjo_readyize(delay, 'delay', numFiles)
		mjo_readyize(ground_hold, 'ground_hold', numFiles)
		travel_time = moving + delay + ground_hold
		mjo_readyize(travel_time, "travel_time", numFiles)
		
		
		moving_final_avg = (np.mean(moving[0:, -1]))
		moving_final_std = np.std(moving[0:, -1])
		delay_final_avg = np.mean(delay[0:, -1])
		delay_final_std = np.std(delay[0:, -1])
		ground_final_avg = np.mean(ground_hold[0:, -1])
		ground_final_std = np.std(ground_hold[0:, -1])
	
		print "Writing to final_mean_std.csv"
		
		with open("final_mean_std.csv", 'a+') as myf:
			myf.write(str(moving_final_avg) + ',' + str(delay_final_avg) + ',' + str(ground_final_avg) + '\n' +
				str(moving_final_std) + ',' + str(delay_final_std) + ',' + str(ground_final_std) + '\n')
	else:
		all_stuff = []
		for i in range(numFiles):
			with open(pre + str(i) + ".csv", 'r') as myf:
				for line in myf:
					stuff = line.split(',')
					stuff = map(float, stuff[:-1])
					all_stuff.append(stuff)
					
		data = -np.array(all_stuff)
		
		mjo_readyize(data, pre[:-1], numFiles)
	
