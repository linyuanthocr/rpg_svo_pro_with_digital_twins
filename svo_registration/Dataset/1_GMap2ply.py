import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib.tri import Triangulation
import laspy

"""
Script to sample unfiormly distributed points on a 3D mesh. 
"""

def read_obj(filename):
    vertices = []
    normals = []
    faces = []
    faces_normals = []
    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if parts:
                if parts[0] == 'v':
                    vertices.append(list(map(float, parts[1:4])))
                elif parts[0] == 'vn':
                    normals.append(list(map(float, parts[1:4])))
                elif parts[0] == 'f':
                    vertex_indices = []
                    normal_indices = []
                    for part in parts[1:]:
                        face_parts = part.split('/')
                        vertex_indices.append(int(face_parts[0]))
                        if len(face_parts) > 2 and face_parts[2]:
                            normal_indices.append(int(face_parts[2]))
                    faces.append(vertex_indices)
                    if normal_indices:
                        faces_normals.append(normal_indices)
    return np.array(vertices), np.array(normals), faces, faces_normals

def calculate_area(v1, v2, v3):
    return 0.5 * np.linalg.norm(np.cross(v2 - v1, v3 - v1))

def triangle_area(v1, v2, v3):
    a = np.linalg.norm(v2 - v1)
    b = np.linalg.norm(v3 - v2)
    c = np.linalg.norm(v1 - v3)
    s = (a + b + c) / 2
    area = np.sqrt(s * (s - a) * (s - b) * (s - c))
    return area

def sample_points(vertices, normals, faces, faces_normals, point_spacing=0.7) : #0.25):
    point_cloud = []
    normal_cloud = []
    for face, face_normal in zip(faces, faces_normals):
        triangles = []
        normal_tris = []
        if len(face) == 4:
            triangles = [
                [vertices[face[0] - 1], vertices[face[1] - 1], vertices[face[2] - 1]],
                [vertices[face[0] - 1], vertices[face[2] - 1], vertices[face[3] - 1]]
            ]
            normal_tris = [
                [normals[face_normal[0] - 1], normals[face_normal[1] - 1], normals[face_normal[2] - 1]],
                [normals[face_normal[0] - 1], normals[face_normal[2] - 1], normals[face_normal[3] - 1]]
            ]
        else:
            triangles = [[vertices[idx - 1] for idx in face]]
            normal_tris = [[normals[nidx - 1] for nidx in face_normal]]
        
        for tri, norm_tri in zip(triangles, normal_tris):
            area = triangle_area(np.array(tri[0]), np.array(tri[1]), np.array(tri[2]))
            if(np.isnan(area)):
                continue
            samples_per_face = int(area / (point_spacing ** 2))
            for _ in range(samples_per_face):
                r1, r2 = np.random.rand(), np.random.rand()
                sqrt_r1 = np.sqrt(r1)
                b1 = 1 - sqrt_r1
                b2 = sqrt_r1 * (1 - r2)
                b3 = sqrt_r1 * r2
                point = b1 * tri[0] + b2 * tri[1] + b3 * tri[2]
                normal = b1 * norm_tri[0] + b2 * norm_tri[1] + b3 * norm_tri[2]
                point_cloud.append(point)
                normal_cloud.append(normal)

    return np.array(point_cloud), np.array(normal_cloud)

def visualize_point_cloud(points, normals):
    print("Points shape:", points.shape, "dtype:", points.dtype)
    print("Normals shape:", normals.shape, "dtype:", normals.dtype)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])

    #read point cloud from ply_points_path = '/home/roxane/ros_ws/src/svo_mr_utility/aligned_points.ply'
    # pcd2 = o3d.io.read_point_cloud('/home/roxane/ros_ws/src/svo_mr_utility/aligned_points.ply')
    # pcd2 = o3d.geometry.PointCloud()
    # pcd2.points = o3d.utility.Vector3dVector(points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)
    #pcd = force_vertical_normals_ground_plane(pcd)
    # pcd.paint_uniform_color([0.5,0.5,0.5])

    o3d.visualization.draw_geometries([pcd, axis], window_name="Point Cloud Visualization with Normals", point_show_normal=False)

    return pcd

def force_vertical_normals_ground_plane(point_cloud):

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])

    # Load the point cloud
    points = np.asarray(point_cloud.points)
    normals = np.asarray(point_cloud.normals)

    temp_map = point_cloud.voxel_down_sample(voxel_size=4)
    o3d.visualization.draw_geometries([temp_map, axis], point_show_normal=True, window_name="Point Cloud Before modified normals")

    condition = (points[:, 2] > -1) & (points[:, 2] < 1)
    normals[condition] = [0, 0,1]

    # Assign colors based on the modification of normals
    colors = np.zeros_like(points)
    colors[condition] = [0, 1, 0]  # Green where normals were modified
    colors[~condition] = [0, 0, 1]  # Blue otherwise

    # Update the point cloud with new normals and colors
    modified_point_cloud = o3d.geometry.PointCloud()
    modified_point_cloud.points = o3d.utility.Vector3dVector(points)
    modified_point_cloud.colors = o3d.utility.Vector3dVector(colors)
    
    modified_point_cloud.normals = o3d.utility.Vector3dVector(normals)

    temp_map = modified_point_cloud.voxel_down_sample(voxel_size=4)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([temp_map, axis], point_show_normal=True, window_name="Point Cloud with modified normals")

    return modified_point_cloud

def load_las_as_point_cloud(las_file_path):
    las = laspy.read(las_file_path)
    points = np.vstack((las.x, las.y, las.z)).transpose()
    pcd = o3d.geometry.PointCloud()
    #axis
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def visualize_point_cloud_las(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])
    vis.add_geometry(axis)
    opt = vis.get_render_option()
    opt.point_size = 2  
    vis.run()
    vis.destroy_window()

def A_obj2ply(filename):
    vertices, normals, faces, faces_normals = read_obj(filename)
    point_cloud, normal_cloud = sample_points(vertices, normals, faces, faces_normals)
    pcd = visualize_point_cloud(point_cloud, normal_cloud)
    # o3d.io.write_point_cloud("/home/roxane/PaperData/RW/oerlikon.ply", pcd)
    o3d.io.write_point_cloud("/home/roxane/Desktop/CITY/CityAndPark/CityWithPark_Winter.ply", pcd)

def B_las2ply(las_file_path):
    pcd = load_las_as_point_cloud(las_file_path)
    visualize_point_cloud_las(pcd)
    o3d.io.write_point_cloud("/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/3d_raw/LiDAR.ply", pcd)


if __name__ == '__main__':
    # A_obj2ply('/home/roxane/PaperData/RW/oerlikon.obj') #/home/roxane/elios3_2706/map_v2.obj') #/home/roxane/ros_ws/src/svo_mr_utility/maps/Oerlikon/Oerlikon_osm_mainbuilding.obj') 
    A_obj2ply('/home/roxane/Desktop/CITY/CityAndPark/CityWithPark_Winter.obj') #/home/roxane/elios3_2706/map_v2.obj') #/home/roxane/ros_ws/src/svo_mr_utility/maps/Oerlikon/Oerlikon_osm_mainbuilding.obj') 
#/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/3d_enu/Oerlikon_ENU.obj') 
    # B_las2ply('/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/3d_raw/LiDAR.las')

