#!/usr/bin/env python

import pandas as pd
import sys
import matplotlib.pyplot as plt
import numpy as np
import strokeRehabSystem.lib.dmp.Python.train_dmp as dmp

file_path = "/home/cibr-strokerehab/CIBR_ws/src/strokeRehabSystem/raw_motion_data"
xml_path = "/home/cibr-strokerehab/CIBR_ws/src/strokeRehabSystem/xml_motion_data"
files = ["down", "down_left", "down_right","left","left_down","left_up","right","right_down","right_up", "up","up_left","up_right" ]
ext = ".csv"



for name in files:

    traj = pd.read_csv(file_path + "/" + name + ext, delimiter=',')
    x_list = []
    y_list = []
    z_list = []
    for row in traj.values:
         x_list.append([row[0], row[3], row[6]])
         y_list.append([row[1], row[4], row[7]])
         z_list.append([row[2], row[5], row[8]])
    filex = xml_path + "/" + name + "_x.xml"
    filey = xml_path + "/" + name + "_y.xml"
    filez = xml_path + "/" + name + "_z.xml"
    Important_values = dmp.train_dmp(filex, 200, x_list, 0.001)
    Important_values = dmp.train_dmp(filey, 200, y_list, 0.001)
    Important_values = dmp.train_dmp(filez, 200, z_list, 0.001)

    print x_list