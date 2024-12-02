import re

# Function to read and parse the input file
def read_colmap_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    images = []
    for i in range(0, len(lines)):
        line = lines[i].strip()
        if line.startswith("#") or not line:
            continue
        if re.match(r'^\d+', line):
            data = line.split()
            image_id = int(data[0])
            qw, qx, qy, qz = map(float, data[1:5])
            tx, ty, tz = map(float, data[5:8])
            camera_id = int(data[8])
            name = data[9]
            images.append((name, tx, ty, tz, qx, qy, qz, qw))
    return images

# Function to write the parsed data to the output file
def write_converted_file(output_path, images):
    with open(output_path, 'w') as file:
        file.write("# timestamp tx ty tz qx qy qz qw\n")
        for i, image in enumerate(images):
            timestamp = 129.473392000000 + i * 0.100002
            file.write(f"{timestamp:.12f} {image[1]} {image[2]} {image[3]} {image[4]} {image[5]} {image[6]} {image[7]}\n")

# Path to the input and output files
input_file_path = '/home/roxane/PaperData/RW/colmap/images.txt'
output_file_path = '/home/roxane/PaperData/RW/colmap/colmap_cam_estimates_2.txt'

# Read the input file and parse the data
images = read_colmap_file(input_file_path)

# Write the converted data to the output file
write_converted_file(output_file_path, images)

print(f"Conversion complete. Output written to {output_file_path}")
