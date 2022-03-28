import open3d as o3d
import laspy
import numpy as np

global sphere
global sphere_list
global cropped_pcd
cropped_pcd = o3d.geometry.PointCloud()
cropped_pcd.points = o3d.utility.Vector3dVector()
sphere_list = []

def f_traverse(node, node_info):
    early_stop = False

    if isinstance(node, o3d.geometry.OctreeInternalNode):
        if isinstance(node, o3d.geometry.OctreeInternalPointNode):
            n = 0
            for child in node.children:
                if child is not None:
                    n += 1

    elif isinstance(node, o3d.geometry.OctreeLeafNode):
        if isinstance(node, o3d.geometry.OctreePointColorLeafNode):

            sphere = o3d.geometry.TriangleMesh.create_sphere(node_info.size/2, 20)
            sphere.paint_uniform_color([1, 0.706, 0])
            sphere = sphere.translate(node_info.origin+node_info.size/2)

            points = np.asarray(pcd.points)

            center = np.array(node_info.origin+node_info.size/2)
            radius = node_info.size/2

            distances = np.linalg.norm(points - center, axis=1)
            cropped_pcd.points.extend(o3d.utility.Vector3dVector(points[distances <= radius]))

            sphere_list.append(sphere)
    else:
        raise NotImplementedError('Node type not recognized!')

    return early_stop

print('Enter las file location with name:')
file_location = input()
in_file = laspy.read(file_location)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(in_file.xyz)

octree = o3d.geometry.Octree(max_depth=1)
octree.convert_from_point_cloud(pcd, size_expand=0.01)
octree.traverse(f_traverse)
o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([pcd] + sphere_list)
o3d.visualization.draw_geometries([cropped_pcd,octree] + sphere_list)

octree2 = o3d.geometry.Octree(max_depth=2)
octree2.convert_from_point_cloud(cropped_pcd, size_expand=0.01)
sphere_list = []
octree2.traverse(f_traverse)
o3d.visualization.draw_geometries([cropped_pcd])
o3d.visualization.draw_geometries([cropped_pcd] + sphere_list)
o3d.visualization.draw_geometries([cropped_pcd,octree2] + sphere_list)