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

epochDatetime = datetime.fromtimestamp(float(timeData_raw.index[0]),utc)
utcDatetime = datetime(int(timeData_raw['/utc_time/year'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/month'][timeData_raw.index[0]]),        \
                        int(timeData_raw['/utc_time/day'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/hour'][timeData_raw.index[0]]),         \
                        int(timeData_raw['/utc_time/min'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/sec'][timeData_raw.index[0]]),          \
                        int(timeData_raw['/utc_time/nanosec'][timeData_raw.index[0]]/1000), \
                        tzinfo=utc)                              #UTC time into python timestamp                                                          

# calculate delta epoch-utc to offset lidar data to UTC (USED BY KML) clock
utcEpochDelta = datetime.timestamp(utcDatetime) - datetime.timestamp(epochDatetime)

# # Open IMU pickle file
# # =============================
# # =============================
with open(imuFilePath, 'rb') as f:
    imuDF_raw = pickle.load(f, encoding="latin1")

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

# # PCD read into dataframe
# # ==========================
# # ==========================

pcdList = sorted(f for f in listdir(pcdPath) \
            if (isfile(join(pcdPath,f)) and '.pcd' in f))
pcdDf = pd.DataFrame([splitext(each)[0] for each in pcdList],columns=['pcdTimestamp'])

IMUCurrentIndex = 0
IMUPrevIndex = 0 

KMLCurrentIndex = 0
KMLPrevIndex = 0
KMLTimeInterp_prev = float(datetime.timestamp((kmlDF.index[0].tz_localize(utc)).to_pydatetime()))
KMLTimeInterp_now = KMLTimeInterp_prev

degToRad = 0.0174533

utmOffsetX = 0
utmOffsetY = 0

print("\n")
for lidarIndex, lidarCurrentTimestamp in tqdm(pcdDf.iloc[350:].iterrows(), total=len(pcdDf.index[350:]), ascii=True):

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
    
    
    # =========================================================================
    # TODO cloud motion compensation per 2 deg
    # =========================================================================

    # TODO make compensation angle a variable
    cloudIterator = 0
    pcdFile = join(pcdPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd") )    
    pcdCurrent = o3d.io.read_point_cloud(pcdFile)
    
    cropIMUCurrentIndex = IMUCurrentIndex
    cropIMUPrevIndex    = cropIMUCurrentIndex

    cropIMUTimeInterp_current = 0.0
    cropIMUTimeInterp_prev = 0.0
    
    pointA = [110*math.cos(0*degToRad),110*math.sin(0*degToRad),0.0]
    pointB = [110*math.cos(0*degToRad),110*math.sin(0*degToRad),0.0]
    pcdTransformed = o3d.geometry.PointCloud()
    
    # print("MOTION COMPENSATION\n")
    while cloudIterator < 180:
    
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
        cropper.bounding_polygon =  o3d.utility.Vector3dVector([[0.0,0.0,0.0],pointA,pointB])
        
        # crop pointcloud
        pcdCropped = cropper.crop_point_cloud(pcdCurrent)
        # calculate time 
        cropTimeSeconds = cloudIterator * (2/360)*0.1 #for use in 10HZ lidar rotation, need to change if lidar rotation changed
        cropTimeEpoch = lidarCurrentEpoch - cropTimeSeconds
        
        # TODO change the whole algo to use KMLCurrentIndex > 0 and final processed data < final data + error handler 
        # find imu baseline data's time, backward loop on enumerate
        # print(enumerate(reversed(imuDF.index[cropIMUCurrentIndex-5:cropIMUCurrentIndex])))
        # print(imuDF.index[cropIMUCurrentIndex-5:cropIMUCurrentIndex])
        
        imuDFCropOriginal = imuDF.index[cropIMUCurrentIndex-4:cropIMUCurrentIndex+1]
        imuDFCropReversed = imuDFCropOriginal[::-1]
        for index, imu_t in enumerate(imuDFCropReversed):
            cropIMUTimestamp = datetime.fromtimestamp(float(imu_t),utc)
            deltaCropImuPcd = float(datetime.timestamp(cropIMUTimestamp)) - cropTimeEpoch
            
            # print("deltaimupcd "+str(deltaImuPcd))
            # print("lidarepoch "+str(lidarCurrentEpoch))
            # print("imu epoch" +str(float(datetime.timestamp(imuTimestamp))))
        
            # print("imu curr index "+str(IMUCurrentIndex))
            # print("crop imu curr index "+str(cropIMUCurrentIndex))
            # # print("deltacropimupcd "+str(deltaCropImuPcd))
            # print("crop epoch " + str(cropTimeEpoch))
            # print("imu crop epoch " + str(float(datetime.timestamp(cropIMUTimestamp))))
            # # print(cropIMUCurrentIndex)
            # print("index" + str(index))
        
            if deltaCropImuPcd <= 0:

                cropIMUCurrentIndex = cropIMUCurrentIndex - index
                cropIMUTimeInterp_current = float(datetime.timestamp(cropIMUTimestamp))
                #TODO to clean this code : 
                if KMLCurrentIndex == 0:
                    cropIMUPrevIndex = cropIMUCurrentIndex
                    cropIMUTimeInterp_prev = cropIMUTimeInterp_current
                else:
                    cropIMUPrevIndex = cropIMUCurrentIndex + 1 #error possibility if found index = 0 (?) at flight end
                    cropIMUTimeInterp_prev = float(imuDF.index[cropIMUPrevIndex])
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
        if (cropIMUTimeInterp_prev - cropIMUTimeInterp_current != 0) :
            cropIMUFactor = (cropTimeEpoch - cropIMUTimeInterp_current)/ \
                            (cropIMUTimeInterp_prev - cropIMUTimeInterp_current)
        
        quatInterpolatedTrueNorth = quatTrueNorthCurrent.slerp(quatTrueNorthPrev, cropIMUFactor)
        # TODO check interpolation value compared to baseline value
        # calculate coordinate interpolation in UTM

        # =========================================================================
        # interpolate lat,lon,height
        # =========================================================================
        cropKMLCurrentIndex = KMLCurrentIndex
        cropKMLPrevIndex = cropKMLCurrentIndex
        # TODO find KML current data (backward)
        # DEBUG & check this code below
        kmlDFCropOriginal = kmlDF.index[cropKMLCurrentIndex-4:cropKMLCurrentIndex+1]
        kmlDFCropReversed = kmlDFCropOriginal[::-1]
        for index, kml_t in enumerate(kmlDFCropReversed):
            cropKmlTimestamp = (kml_t.tz_localize(utc)).to_pydatetime()
            cropDeltaKmlPcd = (float(datetime.timestamp(cropKmlTimestamp)) - cropTimeEpoch)-(utcEpochDelta)
            
            if cropDeltaKmlPcd <= 0:
                
                cropKMLCurrentIndex = cropKMLCurrentIndex - index
                cropKMLTimeInterp_now = float(datetime.timestamp(cropKmlTimestamp))
                
                if KMLCurrentIndex == 0:
                    cropKMLPrevIndex = cropKMLCurrentIndex
                    cropKMLTimeInterp_prev = cropKMLTimeInterp_now
                else:
                    cropKMLPrevIndex = cropKMLCurrentIndex + 1
                    cropKMLTimeInterp_prev = float(datetime.timestamp( \
                                                (kmlDF.index[cropKMLPrevIndex].tz_localize(utc)).to_pydatetime()))
                
                break   

        KMLTimeInterp = [cropKMLTimeInterp_now, cropKMLTimeInterp_prev]
        KMLLatInterp = [float(kmlDF['latitude'][KMLPrevIndex]), float(kmlDF['latitude'][KMLCurrentIndex])]
        KMLLongInterp = [float(kmlDF['longitude'][KMLPrevIndex]), float(kmlDF['longitude'][KMLCurrentIndex])]
        KMLHeightInterp = [float(kmlDF['height'][KMLPrevIndex]), float(kmlDF['height'][KMLCurrentIndex])]
        
        lat = np.interp((cropTimeEpoch+utcEpochDelta),KMLTimeInterp,KMLLatInterp)
        long = np.interp((cropTimeEpoch+utcEpochDelta),KMLTimeInterp,KMLLongInterp)
        height = np.interp((cropTimeEpoch+utcEpochDelta),KMLTimeInterp,KMLHeightInterp)
        
        # convert to UTM
        latlonPos = LatLon(lat, long)
        utmPos = toUtm8(latlonPos,None,None,None)
        
        quatConvergence = mathutils.Quaternion((0.0,0.0,1.0), math.radians(utmPos[6]))
        quatGridNorth = quatConvergence @ quatInterpolatedTrueNorth 
        matRot = (quatGridNorth.to_matrix()).to_4x4()
        
        if utmOffsetX == 0 :
            utmOffsetX = utmPos[2]
            utmOffsetY = utmPos[3]
            offsetFile = open(join(offsetPath, str( "UTM_offset.txt" )), "w+")
            LINE = ["easting: " + str(utmOffsetX) + "\n", "northing: " + str(utmOffsetY)]
            offsetFile.writelines(LINE)
        
        matLoc = mathutils.Matrix.Translation(( (utmPos[2] - utmOffsetX), (utmPos[3] - utmOffsetY) , height))
        matSca = mathutils.Matrix.Scale(utmPos[7],4)
        
        matTransform = matLoc @ matRot @ matSca
        # Map
        pcdCroppedTransformed = pcdCropped.transform(matTransform)
        pcdTransformed = pcdTransformed + pcdCropped
        
        cloudIterator += 1
        # print("iter "+str(cloudIterator))
        

        
    # ##########################################################################
    # ##########################################################################
    # ##########################################################################
    
    # write point cloud
    pcdWriteName = str(datetime.fromtimestamp(lidarCurrentEpoch+utcEpochDelta))
    pcdWriteName = pcdWriteName.replace(" ","__")
    pcdWriteName = pcdWriteName.replace(":","-")
    
    # o3d.io.write_point_cloud( join(destPath, str( pcdWriteName +".pcd") ), pcdTransformed) #write with readable UTC clock
    o3d.io.write_point_cloud( join(destPath, str(pcdDf['pcdTimestamp'][lidarIndex]+".pcd")), pcdTransformed) #write with EPOCH clock

    # o3d.visualization.draw_geometries([pcdCurrent])
    # o3d.visualization.draw_geometries([pcdTransformed])
    
    # break
    # i = i+1
    # if i >= 4:
    #     break
    

