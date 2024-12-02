import struct
import numpy as np

def read_next_bytes(fid, num_bytes, format_char_sequence, is_float=False):
    """Read and unpack the next bytes from the file."""
    data = fid.read(num_bytes)
    if is_float:
        return struct.unpack('<' + format_char_sequence, data)[0]
    else:
        return struct.unpack('<' + format_char_sequence, data)

def read_cameras_bin(file_path):
    """Read cameras from a binary file."""
    with open(file_path, "rb") as fid:
        num_cameras = read_next_bytes(fid, 8, 'Q')[0]
        cameras = {}
        for _ in range(num_cameras):
            camera_id = read_next_bytes(fid, 8, 'Q')[0]
            model_id = read_next_bytes(fid, 4, 'i')[0]
            width = read_next_bytes(fid, 4, 'i')[0]
            height = read_next_bytes(fid, 4, 'i')[0]
            num_params = read_next_bytes(fid, 8, 'Q')[0]
            params = read_next_bytes(fid, num_params * 8, 'd' * num_params, is_float=True)
            cameras[camera_id] = {
                'model': model_id,
                'width': width,
                'height': height,
                'params': params,
            }
        return cameras

def read_images_bin(file_path):
    """Read images from a binary file."""
    with open(file_path, "rb") as fid:
        num_images = read_next_bytes(fid, 8, 'Q')[0]
        images = {}
        for _ in range(num_images):
            image_id = read_next_bytes(fid, 8, 'Q')[0]
            qvec = read_next_bytes(fid, 4 * 8, 'd' * 4, is_float=True)
            tvec = read_next_bytes(fid, 3 * 8, 'd' * 3, is_float=True)
            camera_id = read_next_bytes(fid, 8, 'Q')[0]
            name_length = read_next_bytes(fid, 4, 'I')[0]
            name = fid.read(name_length).decode('utf-8')
            num_points2d = read_next_bytes(fid, 8, 'Q')[0]
            points2d = []
            for _ in range(num_points2d):
                x = read_next_bytes(fid, 8, 'd', is_float=True)
                y = read_next_bytes(fid, 8, 'd', is_float=True)
                point3d_id = read_next_bytes(fid, 8, 'Q')[0]
                points2d.append((x, y, point3d_id))
            images[image_id] = {
                'qvec': qvec,
                'tvec': tvec,
                'camera_id': camera_id,
                'name': name,
                'points2d': points2d,
            }
        return images

def extract_trajectory_and_timestamps(images_bin_path):
    """Extract the trajectory and timestamps from images.bin."""
    images = read_images_bin(images_bin_path)
    trajectory = []
    timestamps = []

    for image_id, image_data in images.items():
        qvec = image_data['qvec']
        tvec = image_data['tvec']
        name = image_data['name']
        
        timestamp = float(name.split('.')[0])  # assuming filenames are timestamps
        timestamps.append(timestamp)
        
        trajectory.append({
            'image_id': image_id,
            'qvec': qvec,
            'tvec': tvec,
            'timestamp': timestamp,
        })

    return trajectory, timestamps


# # Function to write the parsed data to the output file
# def write_converted_file(output_path, images):
#     with open(output_path, 'w') as file:
#         file.write("# timestamp tx ty tz qx qy qz qw\n")
#         for i, image in enumerate(images):
#             timestamp = 129.473392000000 + i * 0.100002
#             file.write(f"{timestamp:.12f} {image[1]} {image[2]} {image[3]} {image[4]} {image[5]} {image[6]} {image[7]}\n")


if __name__ == "__main__":
    images_bin_path = '/home/roxane/PaperData/RW/colmap/images.bin'
    trajectory, timestamps = extract_trajectory_and_timestamps(images_bin_path)
    
    # Print or save the results as needed
    print("Trajectory and Timestamps:")
    for entry in trajectory:
        print(entry)

# # Path to the input and output files
# input_file_path = '/home/roxane/PaperData/RW/colmap/images.txt'
# output_file_path = '/home/roxane/PaperData/RW/colmap/colmap_cam_estimates_2.txt'

# # Read the input file and parse the data
# images = read_colmap_file(input_file_path)

# # Write the converted data to the output file
# write_converted_file(output_file_path, images)

# print(f"Conversion complete. Output written to {output_file_path}")
