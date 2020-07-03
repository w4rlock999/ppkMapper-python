from os import listdir, path, mkdir
from os.path import isfile, join, splitext
from pykml import parser as kmlParser
import pandas as pd
import numpy as np
from datetime import datetime, timedelta, tzinfo 
import time
import pickle
from pygeodesy.ellipsoidalKarney import LatLon 
from pygeodesy.utm import toUtm8
import mathutils
import math
import open3d.open3d_pybind as o3d
import argparse
from tqdm import tqdm
import math

def argParser() :
    parser = argparse.ArgumentParser(description='Process PPK data')
    parser.add_argument('projectName', metavar="projName",
                        type=str, help='project name to process')
    return parser

parser = argParser()
args = parser.parse_args()

path = './data/'
# projectName = 'JetisPPK7'
projectName = args.projectName
kmlFilePath = join(path, projectName + '/map/kml/kmlData.kml')
# TODO change pkl to JSON (from ppk core embedded first)
imuFilePath = join(path, projectName + '/map/pkl/imuData.pkl')
timeConverterFilePath = join(path, projectName + '/map/pkl/utcData.pkl')
# timeConverterFilePath = join(path, projectName + '/map/pkl/ppkData.pkl')
pcdPath = join(path, projectName + '/map/pcd/')
destPath = join(path, projectName + '/map/transformed_pcd/')
offsetPath = join(path, projectName + '/map/')

with open(imuFilePath, 'rb') as f:
    imuDF_raw = pickle.load(f, encoding="latin1")

imuDF = pd.DataFrame()
imuDF['quat_x'] = imuDF_raw['/ppk_quat/quaternion/x']
imuDF['quat_y'] = imuDF_raw['/ppk_quat/quaternion/y']
imuDF['quat_z'] = imuDF_raw['/ppk_quat/quaternion/z']
imuDF['quat_w'] = imuDF_raw['/ppk_quat/quaternion/w']
imuDF.index = imuDF_raw.index

imuDFselected = imuDF.index[0:10]

print(imuDF)
print(imuDFselected)
print(imuDFselected[::-1])
print(reversed(list(imuDF.index[0:10])))