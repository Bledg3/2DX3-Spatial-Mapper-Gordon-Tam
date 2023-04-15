import serial
import math
import numpy as np
import open3d as o3d

s = serial.Serial('COM6', 115200, timeout=5)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# Speed of light in meters per second
SPEED_OF_LIGHT = 299792458

x = 0
angle = 0
angle_direction = 11.25
move_distance = 400
threshold = move_distance + 600

def read_sensor_data():

    distance_str = s.readline().decode()  # The distance value is the second element in the list

    if not distance_str:
        return None  # No distance data in the string
    
    distance = float(distance_str)
    
    return distance

def convert_to_coordinates(distance):

    if distance == None:
        return None,None
    
    # Calculate y,z coordinates for the current angle
    y = distance * math.sin(math.radians(angle))
    z = distance * math.cos(math.radians(angle))
        
    # Yield y,z coordinates
    return y, z

# Wait for user's signal to start the program
input("Press Enter to start communication...")

# Create a new file to write to
f = open("dataPoints.xyz", "w")

# Initialize a list to hold the point cloud for each plane
planes = [[] for _ in range(int(x/move_distance) + 1)]

# Main function
while True:
    
    # Convert data to coordinates
    y, z = convert_to_coordinates(read_sensor_data())

    if y is None and z is None:
        break
    
    # Display coordinates
    print("Y:", y, "Z:", z, "X:", x)
    
    # Write to file
    f.write("{0} {1} {2}\n".format(y, z, x))

    # Add the point to the appropriate plane in the planes list
    plane_index = int(x/move_distance)
    planes[plane_index].append([y, z, x])
    
    # increment angle by 11.25 degrees
    if angle >= 360:        
        angle_direction = -11.25
        
    elif angle <= 0:
        angle_direction = 11.25
        
    angle += angle_direction

    if angle % 360 == 0:
        x += move_distance
        if x/move_distance >= len(planes):
            planes.append([])
    
# Close the file
f.close()

# Create line sets for each plane
line_sets = []
for plane in planes:
    if len(plane) < 2:
        continue
    points = np.array(plane)
    lines = []
    for i in range(len(points)):
        lines.append([i, (i+1)%len(points)])
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(points), lines=o3d.utility.Vector2iVector(lines))
    line_sets.append(line_set)

more_lines = []

# Loop through all but the last plane
for i in range(len(planes)-1):
    # Get the points for the current and next plane
    points1 = np.array(planes[i])
    points2 = np.array(planes[i+1])
    
    # Create a list to hold the lines for the current pair of planes
    sticks = []
    
    # Loop through all the points in the current plane
    for j in range(len(points1)):
        # Get the current point and find the closest point in the next plane
        current_point = points1[j]
        distances = np.linalg.norm(points2 - current_point, axis=1)
        closest_point_index = np.argmin(distances)
        closest_point = points2[closest_point_index]

        # Check if the distance between the current point and closest point is below the threshold
        if distances[closest_point_index] < threshold:
            # Add a line connecting the current point to the closest point to the sticks list
            sticks.append(current_point)
            sticks.append(closest_point)

    # Create a line set from the lines
    vertex_lines = o3d.geometry.LineSet()
    vertex_lines.points = o3d.utility.Vector3dVector(np.array(sticks).reshape(-1, 3))
    vertex_lines.lines = o3d.utility.Vector2iVector(np.arange(len(sticks)*2).reshape(-1,2))

    # Append the line set to the list of line sets
    more_lines.append(vertex_lines)
    
# Read the point cloud from the file
pcd = o3d.io.read_point_cloud("dataPoints.xyz", format="xyz")

# Visualize the point cloud and lines
o3d.visualization.draw_geometries([pcd] + line_sets + more_lines)

# This will signal MCU to start the transmission
s.write('s'.encode())

# Close the port
print("Closing: " + s.name)
s.close()
