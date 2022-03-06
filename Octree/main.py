import open3d as o3d
import laspy
import os

inFile = laspy.read(os.getcwd() + r'\2743_1234.las')
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(inFile.xyz)
pcd.paint_uniform_color([124, 252, 0])

octree = o3d.geometry.Octree(max_depth=10)
octree.convert_from_point_cloud(pcd, size_expand=0.01)
o3d.visualization.draw_geometries([octree])