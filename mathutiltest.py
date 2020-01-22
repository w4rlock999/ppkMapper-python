import mathutils
import math

# a new rotation 90 degrees about the Y axis quat(w,x,y,z)
quat_a = mathutils.Quaternion((0.7071068, 0.0, 0.0, 0.7071068))
rotMat = quat_a.to_matrix()
# print(rotMat)

mat_loc = mathutils.Matrix.Translation((2.0, 3.0, 4.0))

# create an identitiy matrix
mat_sca = mathutils.Matrix.Scale(0.5, 4, (0.0, 0.0, 1.0))

# create a rotation matrix
mat_rot = mathutils.Matrix.Rotation(math.radians(45.0), 4, 'X')

# combine transformations
mat_out = mat_loc @ mat_rot @ mat_sca
print(mat_out)