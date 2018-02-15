#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Plot the aggregate metrics from the file prepared by
aggregate_metrics.py
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

if __name__ == '__main__':

    if len(sys.argv) < 2:
        print "Please provide path to aggregate metrics file a argument."
        sys.exit(-1)

    agg = pd.read_csv(sys.argv[1])
    colors = {'ground_hold':'k','delay':'r','moving':'b'}
    for key in colors.keys():
        plt.errorbar(agg.epoch, agg[key+'_mean'], yerr=agg[key+'_std'], fmt=colors[key])

    plt.xlabel('Epoch')
    plt.ylabel('Time/steps')
    plt.legend(colors.keys(),loc='lower center')
    plt.show()

