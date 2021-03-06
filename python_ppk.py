from os import listdir
from os import path
from os.path import isfile, join, splitext
from pykml import parser
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
kmlFilePath = join(path, projectName + '/map/kml/*.kml')
imuFilePath = join(path, projectName + '/map/pkl/imuData.pkl')
timeConverterFilePath = join(path, projectName + '/map/pkl/utcData.pkl')
# timeConverterFilePath = join(path, projectName + '/map/pkl/ppkData.pkl')
pcdPath = join(path, projectName + '/map/pcd/')


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

print(timeData_raw.index[0])
epochDatetime = datetime.fromtimestamp(float(timeData_raw.index[0]),utc)
utcDatetime = datetime(int(timeData_raw['/utc_time/year'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/month'][timeData_raw.index[0]]),        \
                        int(timeData_raw['/utc_time/day'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/hour'][timeData_raw.index[0]]),         \
                        int(timeData_raw['/utc_time/min'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/sec'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/nanosec'][timeData_raw.index[0]]/1000), \
                        tzinfo=utc)                              #UTC time into phyton timestamp                                                          

print(epochDatetime)
print(utcDatetime)

# print(timeData_raw[['utc_time__hour','utc_time__min']])
# print("epoch timestamp " + str(datetime.timestamp(epochDatetime)))
# print("UTC timestamp   " + str(datetime.timestamp(utcDatetime)))

# print("epoch timestamp " + str(epochDatetime))
# print("UTC timestamp " + str(utcDatetime))

# calculate delta epoch-utc to offset lidar data to KML clock
utcEpochDelta = datetime.timestamp(utcDatetime) - datetime.timestamp(epochDatetime)
print (utcEpochDelta)

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
# with open(kmlFile) as f:
#     kmlObj = parser.parse(f).getroot().Document.Folder

# kmlCoord=[]
# kmlTimeUTC=[]
# for pm in kmlObj.Placemark:
#     pmTimeUTC=pm.TimeStamp.when
#     pmCoor=pm.Point.coordinates
#     kmlCoord.append(pmCoor.text)
#     kmlTimeUTC.append(pmTimeUTC.text)
    
# kmlDF_raw=pd.DataFrame()
# kmlDF_raw['UTC']=kmlTimeUTC
# kmlDF_raw['coordinates']=kmlCoord

# kmlDF = pd.DataFrame()
# kmlDF['longitude'],kmlDF['latitude'],kmlDF['height'] = zip(*kmlDF_raw['coordinates'].apply(lambda x: x.split(',',2)))
# kmlDF.index = kmlDF_raw['UTC'].apply(lambda x: datetime.strptime(str(x),'%Y-%m-%dT%H:%M:%S.%fZ'))
# # print(datetime.timestamp(kmlDF.index[0]))




# # PCD read into dataframe
# # ==========================
# # ==========================

pcdList = sorted(f for f in listdir(pcdPath) \
            if (isfile(join(pcdPath,f)) and '.pcd' in f))
pcdDf = pd.DataFrame([splitext(each)[0] for each in pcdList],columns=['pcdTimestamp'])
# print(pcdDf['pcdTimestamp'][0])


# IMUCurrentTimestamp = 0
# IMUPrevTimestamp = 0 

# KMLCurrentTimestamp = 0
# KMLPrevTimestamp = 0
# KMLTimeInterp_prev = float(datetime.timestamp((kmlDF.index[0].tz_localize(utc)).to_pydatetime()))
# KMLTimeInterp_now = KMLTimeInterp_prev


# # create offset from pcd data to dummy kml
# # ========================================
# kmlTimestampZeroth = (kmlDF.index[0].tz_localize(utc)).to_pydatetime()
# kmlPcdDeltaOffset = float(datetime.timestamp(kmlTimestampZeroth)) - float(datetime.timestamp(datetime.fromtimestamp(float(pcdDf['pcdTimestamp'][0]),utc)))
# # print(kmlPcdDeltaOffset)

# i = 0

# for lidarIndex, lidarCurrentTimestamp in pcdDf.iterrows():
    
#     lidarCurrentDatetime = datetime.fromtimestamp(float(lidarCurrentTimestamp),utc)
#     lidarCurrentEpoch = float(datetime.timestamp(lidarCurrentDatetime))

#     # find imu data
#     for index, imu_t in enumerate(imuDF.index[IMUCurrentTimestamp:]):
#         imuTimestamp = (imu_t.tz_localize(utc)).to_pydatetime()
#         deltaImuPcd = float(datetime.timestamp(imuTimestamp)) - lidarCurrentEpoch
#         if deltaImuPcd >= 0:
#             IMUPrevTimestamp = IMUCurrentTimestamp
#             IMUCurrentTimestamp = IMUCurrentTimestamp + index
#             break

#     print("found matching IMU data " + str(IMUCurrentTimestamp))
#     # print(imuDF['ekf_quat__quaternion_w'][IMUCurrentTimestamp])
    
#     for kmlIndex, kml_t in enumerate(kmlDF.index[KMLCurrentTimestamp:]):
#         kmlTimestamp = (kml_t.tz_localize(utc)).to_pydatetime()
#         deltaKmlPcd = (float(datetime.timestamp(kmlTimestamp)) - lidarCurrentEpoch)-(kmlPcdDeltaOffset)
#         if deltaKmlPcd >= 0:
            
#             KMLCurrentTimestamp = KMLCurrentTimestamp + kmlIndex
#             KMLTimeInterp_now = float(datetime.timestamp(kmlTimestamp))
            
#             if KMLCurrentTimestamp == 0:
#                 KMLPrevTimestamp = KMLCurrentTimestamp
#                 KMLTimeInterp_prev = KMLTimeInterp_now
#             else:
#                 KMLPrevTimestamp = KMLCurrentTimestamp - 1
#                 KMLTimeInterp_prev = float(datetime.timestamp((kmlDF.index[KMLPrevTimestamp].tz_localize(utc)).to_pydatetime()))
#             break   
        
#     print("found matching KML data " + str(KMLCurrentTimestamp))
    
#     # interpolate lat,lon,height
#     # ===============================
#     KMLTimeInterp = [KMLTimeInterp_prev, KMLTimeInterp_now]
#     KMLLatInterp = [float(kmlDF['latitude'][KMLPrevTimestamp]), float(kmlDF['latitude'][KMLCurrentTimestamp])]
#     KMLLongInterp = [float(kmlDF['longitude'][KMLPrevTimestamp]), float(kmlDF['longitude'][KMLCurrentTimestamp])]
#     KMLHeightInterp = [float(kmlDF['height'][KMLPrevTimestamp]), float(kmlDF['height'][KMLCurrentTimestamp])]

#     lat = np.interp((lidarCurrentEpoch+kmlPcdDeltaOffset),KMLTimeInterp,KMLLatInterp)
#     long = np.interp((lidarCurrentEpoch+kmlPcdDeltaOffset),KMLTimeInterp,KMLLongInterp)
#     height = np.interp((lidarCurrentEpoch+kmlPcdDeltaOffset),KMLTimeInterp,KMLHeightInterp)
    
#     # print("original latitude  " + str(kmlDF['latitude'][KMLCurrentTimestamp]))
#     # print("interpolated latitude  " + str(lat) )

#     # Latlon to UTM
#     # =========================
#     # =========================
#     latlonPos = LatLon(lat, long)
#     utmPos = toUtm8(latlonPos,None,None,None)
#     print(utmPos)

#     # TODO take into account of UTM convergence and other calculation
#     quat = mathutils.Quaternion(                \
#         (imuDF['quat_w'][IMUCurrentTimestamp],  \
#          imuDF['quat_x'][IMUCurrentTimestamp],  \
#          imuDF['quat_y'][IMUCurrentTimestamp],  \
#          imuDF['quat_z'][IMUCurrentTimestamp]) )
    
#     matRot = (quat.to_matrix()).to_4x4()
#     matLoc = mathutils.Matrix.Translation((utmPos[2],utmPos[3],height))
#     matSca = mathutils.Matrix.Scale(utmPos[7],4)

#     matTransform = matLoc @ matRot @ matSca
#     print(matTransform) 

#     pcdFile = join(path, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd") )
#     pcdCurrent = o3d.io.read_point_cloud(pcdFile)
#     pcdTransformed = pcdCurrent.transform(matTransform)
#     o3d.visualization.draw_geometries([pcdCurrent])
#     o3d.visualization.draw_geometries([pcdTransformed])
    
#     break
#     # i = i+1
#     # if i >= 10:
#     #     break
    
