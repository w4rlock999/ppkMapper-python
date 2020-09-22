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
import json
from tqdm import tqdm
import sys

def argParser() :
    parser = argparse.ArgumentParser(description='Process PPK data')
    # parser.add_argument('projectName', metavar="projName",
    #                     type=str, help='project name to process')
    parser.add_argument('pathToProject', metavar="pathToProject",
                        type=str, help='path to project directory')
    parser.add_argument('pathToTrajectory', metavar="pathToTrajectorty",
                        type=str, help='path to GPS trajectory file folder')
    return parser

parser = argParser()
args = parser.parse_args()

# path = './data/'
# projectName = args.projectName
# kmlFilePath = join(path, projectName + '/map/kml/kmlData.kml')
# imuFilePath = join(path, projectName + '/map/pkl/imuData.pkl')
# timeConverterFilePath = join(path, projectName + '/map/pkl/utcData.pkl')
# pcdPath = join(path, projectName + '/map/pcd/')
# destPath = join(path, projectName + '/map/transformed_pcd/')
# offsetPath = join(path, projectName + '/map/')

pathToProject           = args.pathToProject
imuFilePath             = pathToProject + '/map/pkl/imuData.pkl'
timeConverterFilePath   = pathToProject + '/map/pkl/utcData.pkl'
pcdPath                 = pathToProject + '/map/pcd/' 
destPath                = pathToProject + '/map/transformed_pcd/'
offsetPath              = pathToProject + '/map/'

pathToTrajectory        = args.pathToTrajectory
kmlFilePath             = pathToTrajectory + '/kmlData.kml'  

filePath                = path.dirname(path.abspath(__file__)) 

try:
    mkdir(destPath)
except OSError:
    print("creation of folder %s failed" % destPath)
else:
    print("successfully creating folder %s" % destPath)

with open( filePath + '/config.json') as f:
    config = json.load(f)

# Open UTC converter pickle file
# =============================
# =============================

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

# print(timeData_raw.index[0])
epochDatetime = datetime.fromtimestamp(float(timeData_raw.index[0]),utc)
utcDatetime = datetime(int(timeData_raw['/utc_time/year'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/month'][timeData_raw.index[0]]),        \
                        int(timeData_raw['/utc_time/day'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/hour'][timeData_raw.index[0]]),         \
                        int(timeData_raw['/utc_time/min'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/sec'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/nanosec'][timeData_raw.index[0]]/1000), \
                        tzinfo=utc)                              #UTC time into phyton timestamp                                                          

# print(epochDatetime)
# print(utcDatetime)

# print(timeData_raw[['utc_time__hour','utc_time__min']])
# print("epoch timestamp " + str(datetime.timestamp(epochDatetime)))
# print("UTC timestamp   " + str(datetime.timestamp(utcDatetime)))

# print("epoch timestamp " + str(epochDatetime))
# print("UTC timestamp " + str(utcDatetime))

# calculate delta epoch-utc to offset lidar data to UTC (USED BY KML) clock
utcEpochDelta = datetime.timestamp(utcDatetime) - datetime.timestamp(epochDatetime)
# print ('delta epoch: ' + str(utcEpochDelta))

# # Open IMU pickle file
# # =============================
# # =============================
with open(imuFilePath, 'rb') as f:
    imuDF_raw = pickle.load(f, encoding="latin1")

# print(imuDF_raw.columns.values)

imuDF = pd.DataFrame()
imuDF['quat_x'] = imuDF_raw['/ppk_quat/quaternion/x']
imuDF['quat_y'] = imuDF_raw['/ppk_quat/quaternion/y']
imuDF['quat_z'] = imuDF_raw['/ppk_quat/quaternion/z']
imuDF['quat_w'] = imuDF_raw['/ppk_quat/quaternion/w']
imuDF.index = imuDF_raw.index


# # Parse KML into dataframe
# # =============================
# # =============================
with open(kmlFilePath) as f:
    kmlObj = kmlParser.parse(f).getroot().Document.Folder

kmlCoord=[]
kmlTimeUTC=[]
for pm in kmlObj.Placemark:
    pmTimeUTC=pm.TimeStamp.when
    pmCoor=pm.Point.coordinates
    kmlCoord.append(pmCoor.text)
    kmlTimeUTC.append(pmTimeUTC.text)
    
kmlDF_raw=pd.DataFrame()
kmlDF_raw['UTC']=kmlTimeUTC
kmlDF_raw['coordinates']=kmlCoord

kmlDF = pd.DataFrame()
kmlDF['longitude'],kmlDF['latitude'],kmlDF['height'] = zip(*kmlDF_raw['coordinates'].apply(lambda x: x.split(',',2)))
kmlDF.index = kmlDF_raw['UTC'].apply(lambda x: (datetime.strptime(str(x),'%Y-%m-%dT%H:%M:%S.%fZ') - timedelta(0,2) ))
# print("kml timestamp: " + str(datetime.timestamp(kmlDF.index[0])))




# # PCD read into dataframe
# # ==========================
# # ==========================

pcdList = sorted(f for f in listdir(pcdPath) \
            if (isfile(join(pcdPath,f)) and '.pcd' in f))
pcdDf = pd.DataFrame([splitext(each)[0] for each in pcdList],columns=['pcdTimestamp'])
# print(pcdDf['pcdTimestamp'][0])


IMUCurrentIndex = 0
IMUPrevIndex = 0 

KMLCurrentIndex = 0
KMLPrevIndex = 0
KMLTimeInterp_prev = float(datetime.timestamp((kmlDF.index[0].tz_localize(utc)).to_pydatetime()))
KMLTimeInterp_now = KMLTimeInterp_prev

# create offset from pcd data to dummy kml
# ========================================
# kmlTimestampZeroth = (kmlDF.index[0].tz_localize(utc)).to_pydatetime()
# kmlPcdDeltaOffset = float(datetime.timestamp(kmlTimestampZeroth)) - float(datetime.timestamp(datetime.fromtimestamp(float(pcdDf['pcdTimestamp'][0]),utc)))
# utcEpochDelta
# print(kmlPcdDeltaOffset)

i = 0           # i process limit on debugging
print("\n")

utmOffsetX = 0
utmOffsetY = 0

degToRad = 0.0174533

cloudCounter = 0
pcdFinal = o3d.geometry.PointCloud()

def leverArmCalc():
    leverArmArray = np.array([[-0.056872,-0.0000018,0.265729]])
    eulerLeverArm = mathutils.Euler(( 90*degToRad, 44.99999*degToRad, 90*degToRad))

    leverArmCoord = o3d.geometry.PointCloud()
    quatLeverArm = mathutils.Quaternion()

    quatLeverArm = eulerLeverArm.to_quaternion()
    quatLeverArm.invert()
    matRotLeverArm = (quatLeverArm.to_matrix()).to_4x4() 

    leverArmCoord.points = o3d.utility.Vector3dVector(leverArmArray)
    leverArmFromLidar = leverArmCoord.transform(matRotLeverArm)

    return(leverArmFromLidar)

processStart = False

for lidarIndex, lidarCurrentTimestamp in tqdm(pcdDf.iloc[3500:-100].iterrows(), total=len(pcdDf.index[3500:-100]), ascii=True):
    
    sys.stderr.flush()

    if (processStart != True) :
        
        processStart = True
        print("process_start", file = sys.stdout) 
        sys.stdout.flush()

    # Find lidar current epoch
    # ===============================
    lidarCurrentDatetime = datetime.fromtimestamp(float(lidarCurrentTimestamp),utc)
    lidarCurrentEpoch = float(datetime.timestamp(lidarCurrentDatetime))

    
    pcdTransformed = o3d.geometry.PointCloud()

    pcdFile = join(pcdPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd") )    
    pcdCurrent = o3d.io.read_point_cloud(pcdFile)
    
    lidarCurrentEpoch = lidarCurrentEpoch + 0.1
  

    # ====================================================
    #     Crop pointcloud, calculate crop time
    # ====================================================
    
    # find imu data
    # =========================
    for index, imu_t in enumerate(imuDF.index[IMUCurrentIndex:]):
        deltaImuPcd = float(imu_t) - lidarCurrentEpoch
        # print(deltaImuPcd)
        if deltaImuPcd >= 0:
            IMUCurrentIndex = IMUCurrentIndex + index
            IMUTimeInterp_current = float(imuDF.index[IMUCurrentIndex])

            IMUPrevIndex = IMUCurrentIndex - 1
            IMUTimeInterp_prev = float(imuDF.index[IMUPrevIndex])
            break

    # find kml data
    # =========================
    for kmlIndex, kml_t in enumerate(kmlDF.index[KMLCurrentIndex:]):
        kmlTimestamp = (kml_t.tz_localize(utc)).to_pydatetime()
        deltaKmlPcd = (float(datetime.timestamp(kmlTimestamp)) - (lidarCurrentEpoch + utcEpochDelta))
        if deltaKmlPcd >= 0:
            
            KMLCurrentIndex = KMLCurrentIndex + kmlIndex
            KMLTimeInterp_now = float(datetime.timestamp(kmlTimestamp))

            KMLPrevIndex = KMLCurrentIndex - 1
            KMLTimeInterp_prev = float(datetime.timestamp( \
                                            (kmlDF.index[KMLPrevIndex].tz_localize(utc)).to_pydatetime()))
            break   
    
    # interpolate IMU data
    # =========================
    quatTrueNorthCurrent = mathutils.Quaternion(                \
        (imuDF['quat_w'][imuDF.index[IMUCurrentIndex]],     \
        imuDF['quat_x'][imuDF.index[IMUCurrentIndex]],      \
        imuDF['quat_y'][imuDF.index[IMUCurrentIndex]],      \
        imuDF['quat_z'][imuDF.index[IMUCurrentIndex]]) )

    quatTrueNorthPrev = mathutils.Quaternion(                \
        (imuDF['quat_w'][imuDF.index[IMUPrevIndex]],     \
        imuDF['quat_x'][imuDF.index[IMUPrevIndex]],      \
        imuDF['quat_y'][imuDF.index[IMUPrevIndex]],      \
        imuDF['quat_z'][imuDF.index[IMUPrevIndex]]) )

    cropIMUFactor = 0.0
    if (IMUTimeInterp_current - IMUTimeInterp_prev != 0) :
        cropIMUFactor = (lidarCurrentEpoch - IMUTimeInterp_prev)/ \
                        (IMUTimeInterp_current - IMUTimeInterp_prev)
    
    quatInterpolatedTrueNorth = quatTrueNorthPrev.slerp(quatTrueNorthCurrent, cropIMUFactor)

    # interpolate lat,lon,height
    # ===============================
    KMLTimeInterp = [KMLTimeInterp_prev, KMLTimeInterp_now]
    KMLLatInterp = [float(kmlDF['latitude'][KMLPrevIndex]), float(kmlDF['latitude'][KMLCurrentIndex])]
    KMLLongInterp = [float(kmlDF['longitude'][KMLPrevIndex]), float(kmlDF['longitude'][KMLCurrentIndex])]
    KMLHeightInterp = [float(kmlDF['height'][KMLPrevIndex]), float(kmlDF['height'][KMLCurrentIndex])]

    lat = np.interp((lidarCurrentEpoch+utcEpochDelta),KMLTimeInterp,KMLLatInterp)
    long = np.interp((lidarCurrentEpoch+utcEpochDelta),KMLTimeInterp,KMLLongInterp)
    height = np.interp((lidarCurrentEpoch+utcEpochDelta),KMLTimeInterp,KMLHeightInterp)

    # Latlon to UTM
    # =========================
    latlonPos = LatLon(lat, long)
    utmPos = toUtm8(latlonPos,None,None,None)
    
    # true north to grid north
    # =========================
    quatConvergence = mathutils.Quaternion((0.0,0.0,1.0), math.radians(utmPos[6]))
    quatGridNorth = quatConvergence @ quatInterpolatedTrueNorth 

    # add config boresight angle adjustment
    # ======================================
    # eulerAdjustmentRoll = mathutils.Euler((config["boresightAdjustment"]["Roll"] * degToRad,0,0))
    # eulerAdjustmentPitch = mathutils.Euler((0,config["boresightAdjustment"]["Pitch"] * degToRad,0))
    # eulerAdjustmentYaw = mathutils.Euler((0,0,config["boresightAdjustment"]["Yaw"] * degToRad))
    eulerAdjustment = mathutils.Euler(( config["boresightAdjustment"]["Roll"]   * degToRad, \
                                        config["boresightAdjustment"]["Pitch"]  * degToRad, \
                                        config["boresightAdjustment"]["Yaw"]    * degToRad))
                                      
    # quatAdjustmentRoll = eulerAdjustmentRoll.to_quaternion()    
    # quatAdjustmentPitch = eulerAdjustmentPitch.to_quaternion()    
    # quatAdjustmentYaw = eulerAdjustmentYaw.to_quaternion()    
    quatAdjustment = eulerAdjustment.to_quaternion()    
    
    # quatFinal = quatAdjustmentRoll @ quatAdjustmentPitch @ quatAdjustmentYaw @  quatGridNorth                                         
    # quatFinal = quatAdjustment @  quatGridNorth                                         
    quatFinal = quatGridNorth                                         
    matRot = (quatFinal.to_matrix()).to_4x4()
    
    if utmOffsetX == 0 :
        utmOffsetX = utmPos[2]
        utmOffsetY = utmPos[3]
        offsetFile = open(join(offsetPath, str( "UTM_offset.txt" )), "w+")
        LINE = ["easting: " + str(utmOffsetX) + "\n", "northing: " + str(utmOffsetY)]
        offsetFile.writelines(LINE)
        offsetFile.close()

    # calculate lever arm offset
    leverArm = leverArmCalc()
    leverArmOffsetPoint = leverArm.transform(matRot)
    
    leverArmOffsetX     = -(np.asarray(leverArmOffsetPoint.points)[0,0])
    leverArmOffsetY     = -(np.asarray(leverArmOffsetPoint.points)[0,1])
    leverArmOffsetZ     = -(np.asarray(leverArmOffsetPoint.points)[0,2])
    

    matLoc = mathutils.Matrix.Translation(( (utmPos[2] - utmOffsetX + leverArmOffsetX), \
                                            (utmPos[3] - utmOffsetY + leverArmOffsetY), \
                                            height + leverArmOffsetZ) )
    matSca = mathutils.Matrix.Scale(utmPos[7],4)
    
    matTransform = matLoc @ matRot @ matSca
    # Map data 
    # ===========================
    
    pcdTransformed = pcdCurrent.transform(matTransform)



    # if cloudCounter <= 3 :
    #     pcdFinal = pcdFinal + pcdTransformed
    #     cloudCounter += 1
    # else :
    lidarCurrentEpoch = lidarCurrentEpoch - 0.1
    pcdFinal = pcdTransformed
    pcdWriteName = str(datetime.fromtimestamp(lidarCurrentEpoch+utcEpochDelta))
    pcdWriteName = pcdWriteName.replace(" ","__")
    pcdWriteName = pcdWriteName.replace(":","-")
    # print(pcdWriteName)
    # o3d.io.write_point_cloud( join(destPath, str( pcdWriteName +".pcd") ), pcdTransformed) #write with readable UTC clock
    o3d.io.write_point_cloud( join(destPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd")), pcdFinal) #write with EPOCH clock

    pcdFinal = o3d.geometry.PointCloud()
    cloudCounter = 0

    # else :

    # o3d.visualization.draw_geometries([pcdCurrent])
    # o3d.visualization.draw_geometries([pcdTransformed])
    
    # break
    # i = i+1
    # if i >= 4:
    #     break
    

