import numpy as np
import open3d.open3d as otd

pcd = otd.io.read_point_cloud("./data/1555077737.509494000.pcd")
print(pcd)
otd.visualization.draw_geometries([pcd])

pcdTranslate = pcd.translate([1000,1000,0],False)

rotMat = np.array([  [0.0000000, -1.0000000,  0.0000000],
                     [1.0000000,  0.0000000,  0.0000000],
                     [0.0000000,  0.0000000,  1.0000000] ])
print(rotMat)
pcdRotate = pcd.rotate(rotMat)
otd.visualization.draw_geometries([pcdRotate])



