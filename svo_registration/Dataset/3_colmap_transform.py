import open3d as o3d
import numpy as np

# Function to perform ICP
def perform_icp(source, target, initial_transform):
    result = o3d.pipelines.registration.registration_icp(
        source, target, 3, initial_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    return result

def get_source():
    source_ = o3d.io.read_point_cloud("/home/roxane/PaperData/NewOerlikonColmap/calibration_from_colmap/colmap_5Hz_new_calib/output/Reconstruction.ply") #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/colmap_txt/filtered_points.ply")
    # source_ = o3d.io.read_point_cloud("/home/roxane/PaperData/NewOerlikonColmap/calibration_from_kalibr/colmap_5Hz/output/reconstruction.ply") #/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/colmap_txt/filtered_points.ply")
    _, ind = source_.remove_statistical_outlier(nb_neighbors=10, std_ratio=0.1)
    source_ = source_.select_by_index(ind)
    source_ = source_.voxel_down_sample(voxel_size=0.05)

    return source_

target = o3d.io.read_point_cloud("/home/roxane/elios3_2706/mapv2.ply") #/home/roxane/PaperData/RW/oerlikon.ply") #/home/roxane/elios3_2706/mapv2.ply") #/home/roxane/ros_ws/src/svo_mr_utility/Elios3Utiityv2/Data/3d_enu/Oerlikon_ENU.ply")
target.paint_uniform_color([0, 1, 0])

points = np.asarray(target.points)
filtered_points = points[points[:, 2] >= 3]
filtered_point_cloud = o3d.geometry.PointCloud()
filtered_point_cloud.points = o3d.utility.Vector3dVector(filtered_points)

# Optionally, you can also copy other attributes like colors or normals if needed
filtered_point_cloud.colors = o3d.utility.Vector3dVector(np.asarray(target.colors)[points[:, 2] >= 3])
filtered_point_cloud.normals = o3d.utility.Vector3dVector(np.asarray(target.normals)[points[:, 2] >= 3])
target = filtered_point_cloud

# Save or display the filtered point cloud
# o3d.io.write_point_cloud("/path/to/save/filtered_point_cloud.ply", filtered_point_cloud)
# o3d.visualization.draw_geometries([filtered_point_cloud])

# If you just want to update the existing point cloud object
target.points = o3d.utility.Vector3dVector(filtered_points)



axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])
# Initialize the transformation guess
# translation = np.array([-26.601, 0, 484])
# rotation_euler = np.array([-90, 0, -136]) * np.pi / 180  # Convert degrees to radians
# translation = np.array([-26.601, 0, 484])
# translation = np.array([-25.0+2, 0, 484-475])
# translation = np.array([-38, 15, 484-475])
# translation = np.array([-40,10,5])
translation = np.array([-28,0.5,11])

# rotation_euler = np.array([-90,-130-90,0]) * np.pi / 180  # Convert degrees to radians
# rotation_euler = np.array([-90,-5,90]) * np.pi / 180  # Convert degrees to radians
# rotation_euler = np.array([-90,-5,90]) * np.pi / 180  # Convert degrees to radians
# rotation_euler = np.array([90,0,0]) * np.pi / 180  # Convert degrees to radians CORRESPONDS TO Z 90
# rotation_euler = np.array([0,90,0]) * np.pi / 180  # Convert degrees to radians CORRESPONDS TO Y 90
# rotation_euler = np.array([-120,-8,-90]) * np.pi / 180  # Convert degrees to radians CORRESPONDS TO Y 90
rotation_euler = np.array([44.5,185,85]) * np.pi / 180  # Convert degrees to radians CORRESPONDS TO Y 90
# R = o3d.geometry.get_rotation_matrix_from_xyz(rotation_euler)
R = o3d.geometry.get_rotation_matrix_from_zyx(rotation_euler)
T_init = np.eye(4)
T_init[:3, :3] = R
T_init[:3, 3] = translation


source = get_source()
# source.scale(5, center=(0, 0, 0))
# source.scale(0.000522, center=(0, 0, 0))
source.scale(5.984, center=(0, 0, 0))
source.paint_uniform_color([1, 0, 0])


#  [[-5.14580126e-01 -7.90454392e-02  8.53791024e-01 -4.09620585e+01]
#  [-8.49322162e-01 -8.97245267e-02 -5.20193593e-01  9.09907032e+00]
#  [ 1.17724927e-01 -9.92824923e-01 -2.09645851e-02  4.46587475e+00]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]


# T_init = np.array([[-7.40380457e-01, -1.28406248e-01,  6.59809529e-01, -2.81058186e+01],
#  [-6.70424275e-01,  7.00002621e-02, -7.38668569e-01,  4.95188088e-03],
#  [ 4.86628194e-02, -9.89248098e-01, -1.37913492e-01,  1.12066174e+01],
#  [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])  
best_rmse = float('inf')
best_scale = None
best_transformation = None
source.transform(T_init)
# #delete point with y > 250
# source.points = o3d.utility.Vector3dVector(np.array(source.points)) #[np.where(np.array(source.points)[:,1] <5)[0]])
o3d.visualization.draw_geometries([source, target], window_name="Initial guess")
# o3d.visualization.draw_geometries([source, axis], window_name="Initial guess")
# Iterate over scale values
# for scale in np.linspace(5.9, 6.0, num=51):  # 11 steps between 12 and 13
for scale in np.linspace(5.95,6, num=51):  # 11 steps between 12 and 13
    print(f"Testing scale: {scale}")

    source = get_source()
    
    # Scale the source point cloud
    source.scale(scale, center=(0, 0, 0))
    source.transform(T_init)
    source.points = o3d.utility.Vector3dVector(np.array(source.points)) #[np.where(np.array(source.points)[:,1] <5)[0]])

    source.paint_uniform_color([1, 0, 0])

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50, origin=[0, 0, 0])
    
    # o3d.visualization.draw_geometries([source, target, axis], window_name="before ICP")


    # Perform ICP
    result = perform_icp(source, target, np.eye(4))

    source.transform(result.transformation)
    source.paint_uniform_color([1, 0, 0])
    # o3d.visualization.draw_geometries([source, target], window_name="after ICP")

    print("scale : ", scale, "RMSE : ", result.inlier_rmse, "Fitness : ", result.fitness)
    # break

    
    # Check if this result is better
    if result.inlier_rmse < best_rmse:
        best_rmse = result.inlier_rmse
        best_scale = scale
        best_transformation = result.transformation


# Apply the best transformation and scale to the source point cloud
source = get_source()
source.scale(best_scale, center=(0, 0, 0))
source.transform(T_init)
source.transform(best_transformation)
source.points = o3d.utility.Vector3dVector(np.array(source.points)) #[np.where(np.array(source.points)[:,1] <5)[0]])

# Visualize final result
source.paint_uniform_color([1, 0, 0])  # Red
target.paint_uniform_color([0, 1, 0])  # Green
o3d.visualization.draw_geometries([source, target], window_name="Best Scale and ICP Alignment")
o3d.io.write_point_cloud("/home/roxane/PaperData/glomap_output/output/colmap2.ply", source) 
#/home/roxane/elios3_2706/EL300825315585_01645_214_1flight/colmap_txt/aligned_points_v2.ply", source)

# Print the best result
print("Best Scale:", best_scale)
print("Best Transformation matrix:\n", best_transformation@T_init)
print("Best Inlier RMSE:", best_rmse)
