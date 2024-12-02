import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib.tri import Triangulation

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

    vertices = np.array(vertices)
    normals = np.array(normals)

    print("Vertices Z-coordinate range:", vertices[:, 2].min(), vertices[:, 2].max())


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

def sample_points(vertices, normals, faces, faces_normals, point_spacing=0.2) : #25):
    point_cloud = []
    normal_cloud = []
    k=0
    for face, face_normal in zip(faces, faces_normals):
        k+=1
        if(k%1000==0):
            print("Face", k/1000, "out of", len(faces)/1000)
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
            samples_per_face = int(area / (point_spacing ** 2))

            # print(f"Processing triangle: {tri} with area: {area} and samples per face: {samples_per_face}")

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

    points = np.array(point_cloud)
    normals = np.array(normal_cloud)

    print("Sampled points Z-coordinate range:", points[:, 2].min(), points[:, 2].max())
    print("Number of sampled points:", len(points))

    return np.array(point_cloud), np.array(normal_cloud)

def visualize_point_cloud(points, normals):
    print("Points shape:", points.shape, "dtype:", points.dtype)
    print("Normals shape:", normals.shape, "dtype:", normals.dtype)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)
    pcd = force_vertical_normals_ground_plane(pcd)
    pcd.paint_uniform_color([0.5,0.5,0.5])

    o3d.visualization.draw_geometries([pcd, axis], window_name="Point Cloud Visualization with Normals", point_show_normal=False)
    o3d.io.write_point_cloud("/home/roxane/svo_gps_ws/src/svo_volocopter/svo_registration/maps/FM.ply", pcd)


def force_vertical_normals_ground_plane(point_cloud):

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])

    # Load the point cloud
    points = np.asarray(point_cloud.points)
    normals = np.asarray(point_cloud.normals)

    temp_map = point_cloud.voxel_down_sample(voxel_size=4)
    o3d.visualization.draw_geometries([temp_map, axis], point_show_normal=True, window_name="Point Cloud Before modified normals")

    condition = (points[:, 2] > -1) & (points[:, 2] < 0.01)
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

vertices, normals, faces, faces_normals = read_obj('/home/roxane/Results/Cities/city.obj')
point_cloud, normal_cloud = sample_points(vertices, normals, faces, faces_normals)

visualize_point_cloud(point_cloud, normal_cloud)

