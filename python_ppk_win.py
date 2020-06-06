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
imuFilePath = join(path, projectName + '/map/pkl/imuData.pkl')
timeConverterFilePath = join(path, projectName + '/map/pkl/utcData.pkl')
# timeConverterFilePath = join(path, projectName + '/map/pkl/ppkData.pkl')
pcdPath = join(path, projectName + '/map/pcd/')
destPath = join(path, projectName + '/map/transformed_pcd/')
offsetPath = join(path, projectName + '/map/')

try:
    mkdir(destPath)
except OSError:
    print("creation of folder %s failed" % destPath)
else:
    print("successfully creating folder %s" % destPath)

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
kmlDF.index = kmlDF_raw['UTC'].apply(lambda x: datetime.strptime(str(x),'%Y-%m-%dT%H:%M:%S.%fZ'))
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

degToRad = 0.0174533

i = 0           # i process limit on debugging
print("\n")

utmOffsetX = 0
utmOffsetY = 0

for lidarIndex, lidarCurrentTimestamp in tqdm(pcdDf.iterrows(), total=len(pcdDf.index), ascii=True):
    

    lidarCurrentDatetime = datetime.fromtimestamp(float(lidarCurrentTimestamp),utc)
    lidarCurrentEpoch = float(datetime.timestamp(lidarCurrentDatetime))
    
    # =========================================================================
    # find imu data
    # =========================================================================
    for index, imu_t in enumerate(imuDF.index[IMUCurrentIndex:]):
        imuTimestamp = datetime.fromtimestamp(float(imu_t),utc)
        deltaImuPcd = float(datetime.timestamp(imuTimestamp)) - lidarCurrentEpoch
        # TODO change to KML like search algorithm
        if deltaImuPcd >= 0:
            IMUPrevIndex = IMUCurrentIndex
            IMUCurrentIndex = IMUCurrentIndex + index
            break

    # print("found matching IMU data " + str(IMUCurrentIndex))
    # print(imuDF['ekf_quat__quaternion_w'][IMUCurrentIndex])

    # =========================================================================
    # find kml data
    # =========================================================================
    for kmlIndex, kml_t in enumerate(kmlDF.index[KMLCurrentIndex:]):
        kmlTimestamp = (kml_t.tz_localize(utc)).to_pydatetime()
        deltaKmlPcd = (float(datetime.timestamp(kmlTimestamp)) - lidarCurrentEpoch)-(utcEpochDelta)
        if deltaKmlPcd >= 0:
            
            KMLCurrentIndex = KMLCurrentIndex + kmlIndex
            KMLTimeInterp_now = float(datetime.timestamp(kmlTimestamp))
            
            if KMLCurrentIndex == 0:
                KMLPrevIndex = KMLCurrentIndex
                KMLTimeInterp_prev = KMLTimeInterp_now
            else:
                KMLPrevIndex = KMLCurrentIndex - 1
                KMLTimeInterp_prev = float(datetime.timestamp( \
                                            (kmlDF.index[KMLPrevIndex].tz_localize(utc)).to_pydatetime()))
            break   
        
    # print("found matching KML data " + str(KMLCurrentIndex))
    
    cloudIterator = 0
    pcdFile = join(pcdPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd") )
    pcdCurrent = o3d.io.read_point_cloud(pcdFile)
    

    cropIMUCurrentIndex = IMUCurrentIndex
    cropIMUPrevIndex    = cropIMUCurrentIndex
    
    while cloudIterator < 180:
    
        # TODO pointcloud transformation per laser section
        
        pointA = [110*math.cos(0*degToRad),110*math.sin(0*degToRad),0.0]
        pointB = [110*math.cos(-(360-2)*degToRad),110*math.sin(-(360-2)*degToRad),0.0]
        # loop on a scanned pointcloud
        # calculate cropping polygon
        cropper = o3d.visualization.SelectionPolygonVolume()
        cropper.axis_max = 25.0
        cropper.axis_min = -25.0
        cropper.orthogonal_axis = "Z"
        
        pointA = pointB
        pointB =    [  
                        110*math.cos(-(360-2*(cloudIterator+1))*degToRad),
                        110*math.sin(-(360-2*(cloudIterator+1))*degToRad),
                        0.0
                    ]
        cropper.bounding_polygon =  [
                                        [0.0,0.0,0.0],
                                        pointA,
                                        pointB
                                    ]

        # crop pointcloud
        pcdCropped = cropper.crop_point_cloud(pcdCurrent)
        # calculate time 
        cropTimeSeconds = cloudIterator * 0.00055
        cropTimeEpoch = lidarCurrentEpoch - cropTimeSeconds
        # find imu time, backward loop on enumerate

        
        for index, imu_t in reversed(list(enumerate(imuDF.index[cropIMUCurrentIndex:]))):
            cropIMUTimestamp = datetime.fromtimestamp(float(imu_t),utc)
            deltaCropImuPcd = float(datetime.timestamp(cropIMUTimestamp)) - cropTimeEpoch

            if deltaCropImuPcd <= 0:
                cropIMUCurrentIndex = cropIMUCurrentIndex - index
                cropIMUTimeInterp_now = float(datetime.timestamp(cropIMUTimestamp))

                if KMLCurrentIndex == 0:
                    cropIMUPrevIndex = cropIMUCurrentIndex
                    cropIMUTimeInterp_prev = cropIMUTimeInterp_now
                else:
                    cropIMUPrevIndex = cropIMUCurrentIndex + 1
                    cropIMUTimeInterp_prev = float(datetime.timestamp( \
                            (imuDF.index[cropIMUPrevIndex].tz_localize(utc)).to_pydatetime()))
                break
        # interpolate rotation & pos 
        quatTrueNorthCurrent = mathutils.Quaternion(                \
            (imuDF['quat_w'][imuDF.index[cropIMUCurrentIndex]],     \
            imuDF['quat_x'][imuDF.index[cropIMUCurrentIndex]],      \
            imuDF['quat_y'][imuDF.index[cropIMUCurrentIndex]],      \
            imuDF['quat_z'][imuDF.index[cropIMUCurrentIndex]]) )
    
        quatTrueNorthPrev = mathutils.Quaternion(                \
            (imuDF['quat_w'][imuDF.index[cropIMUPrevIndex]],     \
            imuDF['quat_x'][imuDF.index[cropIMUPrevIndex]],      \
            imuDF['quat_y'][imuDF.index[cropIMUPrevIndex]],      \
            imuDF['quat_z'][imuDF.index[cropIMUPrevIndex]]) )

        cropIMUFactor = 0.0
        if (cropIMUTimeInterp_prev - cropIMUTimeInterp_now != 0) :
            cropIMUFactor = (cropTimeEpoch - cropIMUTimeInterp_now)/
                            (cropIMUTimeInterp_prev - cropIMUTimeInterp_now)

        quatInterpolated = quatTrueNorthCurrent.slerp(quatTrueNorthPrev, cropIMUFactor)

        # quatConvergence = mathutils.Quaternion((0.0,0.0,1.0), math.radians(utmPos[6]))
        # quatGridNorth = quatConvergence @ quatTrueNorth 
        # matRot = (quatGridNorth.to_matrix()).to_4x4()
 
        # calculate UTM
        # Map
        pcdTransformed = pcdCropped.transform(matTransform)
        cloudIterator += 1

    # =========================================================================
    # interpolate lat,lon,height
    # =========================================================================
    KMLTimeInterp = [KMLTimeInterp_prev, KMLTimeInterp_now]
    KMLLatInterp = [float(kmlDF['latitude'][KMLPrevIndex]), float(kmlDF['latitude'][KMLCurrentIndex])]
    KMLLongInterp = [float(kmlDF['longitude'][KMLPrevIndex]), float(kmlDF['longitude'][KMLCurrentIndex])]
    KMLHeightInterp = [float(kmlDF['height'][KMLPrevIndex]), float(kmlDF['height'][KMLCurrentIndex])]

    lat = np.interp((lidarCurrentEpoch+utcEpochDelta),KMLTimeInterp,KMLLatInterp)
    long = np.interp((lidarCurrentEpoch+utcEpochDelta),KMLTimeInterp,KMLLongInterp)
    height = np.interp((lidarCurrentEpoch+utcEpochDelta),KMLTimeInterp,KMLHeightInterp)

    # print(KMLTimeInterp)
    # print(lidarCurrentEpoch+utcEpochDelta)

    # print("original latitude  " + str(kmlDF['latitude'][KMLCurrentIndex]))
    # print("interpolated latitude  " + str(lat) )

    # =========================================================================
    # Latlon to UTM
    # =========================================================================
    latlonPos = LatLon(lat, long)
    utmPos = toUtm8(latlonPos,None,None,None)
    # print(utmPos)

    quatTrueNorth = mathutils.Quaternion(                \
        (imuDF['quat_w'][imuDF.index[IMUCurrentIndex]],  \
         imuDF['quat_x'][imuDF.index[IMUCurrentIndex]],  \
         imuDF['quat_y'][imuDF.index[IMUCurrentIndex]],  \
         imuDF['quat_z'][imuDF.index[IMUCurrentIndex]]) )
    # print(quatTrueNorth)
    
    quatConvergence = mathutils.Quaternion((0.0,0.0,1.0), math.radians(utmPos[6]))
    quatGridNorth = quatConvergence @ quatTrueNorth 
    matRot = (quatGridNorth.to_matrix()).to_4x4()
    
    # HEADING ADJUSTMENT:
    # quatAdjustment = mathutils.Quaternion((0.0,0.0,1.0), math.radians(4))
    # quatAdjustedNorth = quatAdjustment @ quatGridNorth
    # matRot = (quatAdjustedNorth.to_matrix()).to_4x4()

    if utmOffsetX == 0 :
        utmOffsetX = utmPos[2]
        utmOffsetY = utmPos[3]
        offsetFile = open(join(offsetPath, str( "UTM_offset.txt" )), "w+")
        LINE = ["easting: " + str(utmOffsetX) + "\n", "northing: " + str(utmOffsetY)]
        offsetFile.writelines(LINE)
    
    matLoc = mathutils.Matrix.Translation(( (utmPos[2] - utmOffsetX), (utmPos[3] - utmOffsetY) , height))
    matSca = mathutils.Matrix.Scale(utmPos[7],4)

    # dummy transform mat components to test compression existence
    # matRot = ((mathutils.Quaternion((1,0,0,0))).to_matrix()).to_4x4()
    # matLoc = mathutils.Matrix.Translation((0,0,0))
    # matSca = mathutils.Matrix.Scale(1,4) 
    
    matTransform = matLoc @ matRot @ matSca
    # print(matTransform) 

    pcdFile = join(pcdPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd") )
    pcdCurrent = o3d.io.read_point_cloud(pcdFile)
    pcdTransformed = pcdCurrent.transform(matTransform)

    # print(join(destPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd")))

    pcdWriteName = str(datetime.fromtimestamp(lidarCurrentEpoch+utcEpochDelta))
    pcdWriteName = pcdWriteName.replace(" ","__")
    pcdWriteName = pcdWriteName.replace(":","-")
    # print(pcdWriteName)
    # o3d.io.write_point_cloud( join(destPath, str( pcdWriteName +".pcd") ), pcdTransformed) #write with readable UTC clock
    o3d.io.write_point_cloud( join(destPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd")), pcdTransformed) #write with EPOCH clock


    # o3d.visualization.draw_geometries([pcdCurrent])
    # o3d.visualization.draw_geometries([pcdTransformed])
    
    # break
    # i = i+1
    # if i >= 4:
    #     break
    

