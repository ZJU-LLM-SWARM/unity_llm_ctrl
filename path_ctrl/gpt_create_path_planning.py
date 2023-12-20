# import numpy as np

# # Define a step size of 100 for the drone to cover the maximum area in each step
# step_size = 100

# # Define the altitude at which the drone should maintain during the operation
# altitude = 25

# # Create an empty list to store the path
# path = []

# # Iterate over the x and z directions from -500 to 500 with a step size of 100
# for x in range(-500, 501, step_size):
#     for z in range(-500, 501, step_size):
        
#         # Append the current position [x, altitude, z] to the path
#         # Each position is converted to float as required
#         path.append([float(x), float(altitude), float(z)])

# # Convert the path to a numpy array
# path_np = np.array(path)

# # Save the numpy array to path.txt
# np.savetxt('path.txt', path_np)

# import numpy as np

# # 定义无人机起始点
# start_point = np.array([-400.0, 25.0, -400.0])

# # 定义无人机初始飞行方向，向右为1，向左为-1
# direction = 1

# # 创建用于保存无人机路径的列表
# path = [start_point]

# # 生成路径规划（蛇形规划）
# for z in np.arange(-400.0, 500.0, 50.0):  # 修改步长为50m
#     for x in np.arange(-400.0, 500.0, 50.0)[::direction]:  # 修改步长为50m
#         path.append(np.array([x, 25.0, z]))
#     direction *= -1

# # 转化为numpy数组
# path = np.array(path)

# # 保存为.txt文件
# np.savetxt('003.txt', path, fmt='%f')

##################################################################
# 定义地图参数和无人机坐标
# Python Code
# map_details = {
#     'drone1': {
#         'position': [-400, 25, -400],
#         'map_boundaries': [[-500, 0, -500], [-500, 0, 500], [0, 0, 500], [0, 0, -500]]
#     },
#     'drone2': {
#         'position': [400, 25, 400],
#         'map_boundaries': [[0, 0, -500], [0, 0, 500], [500, 0, 500], [500, 0, -500]]
#     }
# }

# for name, data in map_details.items():
#     with open(f'{name}_map.txt', 'w') as f:
#         position_info = 'Position: ' + ', '.join(str(p) for p in data['position'])
#         boundary_info = 'Map Boundaries: ' + ', '.join(str(b) for b in data['map_boundaries'])

#         f.write(position_info + '\n')
#         f.write(boundary_info + '\n')

# print('Done, map details successfully saved to files: drone1_map.txt and drone2_map.txt')
###################################################################
# Import libraries
import numpy as np

# Initial position and step length
position = [-500, 25, -500]
step_length = 200

# Map boundaries
boundaries = [
    [-500, 0, -500], 
    [-500, 0, 500], 
    [0, 0, 500], 
    [0, 0, -500]
]

# Initialize a list to hold coordinates 
coordinates = []

# Generate coordinates for the path
for x in range(position[0], boundaries[2][0]+1, step_length):
    for z in range(position[2], boundaries[1][2]+1, step_length):
        coordinates.append([float(x), float(position[1]), float(z)])

# Convert the list to numpy array
path = np.array(coordinates)

# Save to a text file
np.savetxt('003.txt', path, fmt='%f')

import numpy as np

# 地图边界
x_min, y_min, z_min = 0, 0, -500
x_max, y_max, z_max = 500, 0, 500

# 无人机初始位置
x_start, y_start, z_start = 500, 25, -500

# 步长
step_length = 200

# 创建一个numpy数组来保存路径点
path = []

# 蛇形路径规划
x = x_start
z = z_start
while x >= x_min:
    while z <= z_max + 5:
        path.append([float(x), float(y_start), float(z)])
        z += step_length
    x -= step_length
    if x >= x_min:
        z = z_max
        while z >= z_min - 5:
            path.append([float(x), float(y_start), float(z)])
            z -= step_length
            if(z < z_min):
                z += step_length
                break
        x -= step_length

# 保存路径点为numpy数组
path = np.array(path)

# 将路径规划结果保存为txt文件
np.savetxt("004.txt", path, fmt="%.2f")