import numpy as np
import open3d as o3d

pcd = o3d.io.read_point_cloud('/home/aichi2204/Documents/bkl2go/20240404-ib-w-out/aichi-20240404-ib-w-out_mini.pcd')
print("finish reading")
np.savetxt('/home/aichi2204/Documents/bkl2go/20240404-ib-w-out/aichi-20240404-ib-w-out_mini.txt', np.asarray(pcd.points), delimiter=' ', fmt='%.5f')
print("finish writing")