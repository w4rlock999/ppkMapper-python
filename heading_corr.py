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
import open3d.open3d as o3d
import argparse
import copy
from tqdm import tqdm

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
imuFilePath = join(path, projectName + '/map/pkl/imuData.pkl')
timeConverterFilePath = join(path, projectName + '/map/pkl/utcData.pkl')
# timeConverterFilePath = join(path, projectName + '/map/pkl/ppkData.pkl')
pcdPath = join(path, projectName + '/map/pcd/')
destPath = join(path, projectName + '/map/segmented/')
offsetPath = join(path, projectName + '/map/')

sourceCloud = o3d.io.read_point_cloud(join(destPath,'source.pcd') )
# targetCloud = o3d.io.read_point_cloud(join(destPath,'1582777335.213582000.pcd') )
targetCloud = o3d.io.read_point_cloud(join(destPath,'target.pcd') )

for x in range(0,1) :
    adjustRotation = x +20

    quatAdjustment = mathutils.Quaternion((0.0,0.0,1.0), math.radians(adjustRotation))
    matRot = (quatAdjustment.to_matrix()).to_4x4()

    source_rot = copy.deepcopy(sourceCloud)
    target_rot = copy.deepcopy(targetCloud)
    
    source_rot.transform(matRot)
    target_rot.transform(matRot)

    transInit = np.asarray([  [1.0, 0.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0, 0.0],
                              [0.0, 0.0, 1.0, 0.0], 
                              [0.0, 0.0, 0.0, 1.0] ])
    threshold = 1
    print('initial alignment' + str(adjustRotation))
    evaluation = o3d.registration.evaluate_registration(sourceCloud, targetCloud, threshold, transInit)
    print(evaluation)
    print("\n")
    source_rot.paint_uniform_color([1, 0.706, 0])
    target_rot.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_rot, target_rot])

# source_temp = copy.deepcopy(sourceCloud)
# target_temp = copy.deepcopy(targetCloud)
# source_temp.paint_uniform_color([1, 0.706, 0])
# target_temp.paint_uniform_color([0, 0.651, 0.929])
# source_temp.transform(reg_p2p.transformation)
# o3d.visualization.draw_geometries([source_temp, target_temp])

# o3d.visualization.draw_geometries([sourceCloud])
# o3d.visualization.draw_geometries([targetCloud])