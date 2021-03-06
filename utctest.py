from os import listdir, path, mkdir
from os.path import isfile, join, splitext
from pykml import parser as kmlParser
import pandas as pd
import numpy as np
from datetime import datetime, timedelta, tzinfo 
import pickle
from pygeodesy.ellipsoidalKarney import LatLon 
from pygeodesy.utm import toUtm8
import mathutils
from math import sin, cos, radians
import open3d.open3d as o3d
from tqdm import tqdm
import argparse
import json
import sys


# timeConverterFilePath = "C:\\Users\\w4rlo\\Documents\\Workspace\\ppk_mapper\\data\\TestFlightRejodaniSore\\map\\pkl\\utcData.pkl"
timeConverterFilePath = "C:\\Users\\w4rlo\\Documents\\Workspace\\ppk_mapper\\data\\tower30tiga\\map\\pkl\\utcData.pkl"
# timeConverterFilePath = "C:\\Users\\w4rlo\\Documents\\Workspace\\ppk_mapper\\data\\demoDay1\\map\\pkl\\utcData.pkl"

with open(timeConverterFilePath, 'rb') as f:
    timeData_raw = pickle.load(f, encoding="latin1")

ZERO = timedelta(0)
class UTCtzinfo(tzinfo):
    def utcoffset(self, dt):
        return ZERO
    
    def tzname(self, dt):
        return "UTC"

    def dst(self, dt):
        return ZERO

utc = UTCtzinfo()
index = 1

while index < 3000 :


    # print(timeData_raw.index[0])
    epochDatetime = datetime.fromtimestamp(float(timeData_raw.index[index]),utc)
    utcDatetime = datetime(int(timeData_raw['/utc_time/year'][timeData_raw.index[index]]),          \
                            int(timeData_raw['/utc_time/month'][timeData_raw.index[index]]),        \
                            int(timeData_raw['/utc_time/day'][timeData_raw.index[index]]),          \
                            int(timeData_raw['/utc_time/hour'][timeData_raw.index[index]]),         \
                            int(timeData_raw['/utc_time/min'][timeData_raw.index[index]]),          \
                            int(timeData_raw['/utc_time/sec'][timeData_raw.index[index]]),          \
                            int(timeData_raw['/utc_time/nanosec'][timeData_raw.index[index]]/1000),     \
                            tzinfo=utc) 

    utcStatus = timeData_raw['/utc_time/clock_status/clock_stable'][timeData_raw.index[index]]        

    utcEpochDelta = datetime.timestamp(utcDatetime) - datetime.timestamp(epochDatetime)
    print(utcEpochDelta,  end=' ')
    print(float(timeData_raw.index[index]),  end=' ') 
    print(epochDatetime, end=' ')               
    print(utcDatetime, end=' ')               #problem with conversion to timestamp here!
    print(utcStatus)
    index = index + 1