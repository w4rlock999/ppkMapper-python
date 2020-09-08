import open3d.open3d as o3d
import numpy as np
import mathutils

degToRad = 0.0174533

leverArmArray = np.array([[-0.056872,-0.0000018,0.265729]])
eulerLeverArm = mathutils.Euler(( 90*degToRad, 44.99999*degToRad, 90*degToRad))

leverArm = o3d.geometry.PointCloud()
quatLeverArm = mathutils.Quaternion()

quatLeverArm = eulerLeverArm.to_quaternion()
quatLeverArm.invert()
matRotLeverArm = (quatLeverArm.to_matrix()).to_4x4() 

leverArm.points = o3d.utility.Vector3dVector(leverArmArray)
leverArmFromLidar = leverArm.transform(matRotLeverArm)

print(np.asarray(leverArmFromLidar.points)[0,0])
print(np.asarray(leverArmFromLidar.points)[0,1])
print(np.asarray(leverArmFromLidar.points)[0,2])
print(np.asarray(leverArmFromLidar.points))

