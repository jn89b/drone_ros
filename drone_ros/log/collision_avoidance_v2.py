"""
## Obstacle avoidance 
- Use KNN to find nearest n obstacle 
- Check for the following
    - if heading towards the obstacle:
        - compute ego_unit_vector
        - compute los_from ego to los
        - dot_product:
            - if positive facing towards it
    - distance/manuever check
        - compute distance and compute phi to manuever:
            - $\phi_{computed} = v_{curr}^2/(gr)$
            - if $\phi_{computed} + \phi_{computed} >= \phi_{max}$
            - Then dangerous

    - If both these conditions hold true then I need to 
    set a reference point and depending on the 
    sign convention of the aircraft (left or right) 
    set the desired phi angle to the max or min phi angle  
    
"""
import time 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from numba import jit
from scipy.spatial import distance

@jit(nopython=True)
def euclidean_distance_numba(vector1, vector2):
    '''calculate the euclidean distance,
    '''
    dist = np.linalg.norm(vector1-vector2)
    return dist


@jit(nopython=True)
def find_nearest_obstacle(obstacles:list, ego_position:np.ndarray, K:int=3, 
                          use_2d:bool=False) -> np.ndarray:
    """
    Find the nearest n obstacles from the ego_position
    """
    num_obstacles = len(obstacles)
    distances = np.zeros(num_obstacles)
    if use_2d:
        ego_position = ego_position[:2]
        obstacles = obstacles[:,:2]
        for i in range(num_obstacles):
            distances[i] = euclidean_distance_numba(ego_position, obstacles[i])
    else:
        for i in range(num_obstacles):
            distances[i] = euclidean_distance_numba(ego_position, obstacles[i])
        
    #sort the distances and return the indices of the nearest K obstacles
    nearest_indices = np.argsort(distances)[:K]
    near_obstacles = obstacles[nearest_indices]
    near_distance = distances[nearest_indices]

    return near_obstacles, near_distance, nearest_indices

def knn_obstacles(obs:np.ndarray, ego_pos:np.ndarray, 
                  K:int=3, use_2d:bool=False):
    if use_2d:
        ego_position = ego_pos[:2]
        obstacles = obs[:,:2]
    
    nearest_indices = distance.cdist([ego_position], obstacles).argsort()[:, :K]
    nearest_obstacles = obs[nearest_indices]
    return nearest_obstacles[0], nearest_indices
    
# @jit(nopython=True)
def does_right_side_have_more_obstacles(ego_heading:float,
                                    danger_zones:np.ndarray, right_array:np.ndarray,
                                    left_array:np.ndarray) -> bool:
    """
    Given position and heading we will determine which side has more obstacles by 
    computing the cross product of the ego_heading and the vector to the obstacle
    We will then sum up the right_array and return a boolean value 
    it will return true if the right side has more obstacles and 
    false if the left side has more obstacles 
    """
    ego_unit_vector = np.array([np.cos(ego_heading), np.sin(ego_heading)])
    for i in range(len(danger_zones)):
        #compute the vector to the obstacle
        obs_position = danger_zones[i][:2]
        
        #compute los from ego to obstacle
        los_vector = ego_unit_vector - obs_position
        los_vector /= np.linalg.norm(los_vector)
        cross_product = np.cross(ego_unit_vector, los_vector)
        
        print("cross product", cross_product)
        #if cross product is positive then obstacle is to the left 
        if cross_product >= 0:
            left_array[i] = 1
        #if cross product is negative then obstacle is to the right
        else:
            right_array[i] = 1 
    
    sum_right = np.sum(right_array)
    sum_left = np.sum(left_array)
    print("sum right", sum_right)
    print("sum left", sum_left)
    if sum_right > sum_left:
        return True
    else:
        return False
    
# @jit(nopython=True)
def find_driveby_direction(goal_position:np.ndarray, current_position:np.ndarray, 
                            heading_rad:float,
                            obs_radius:float,  
                            robot_radius:float, 
                            consider_obstacles:bool=False, 
                            danger_zones:np.ndarray=None,
                            use_closest_to_ref:bool=False,
                            ref_point:np.ndarray=None) -> np.ndarray:
    """
    Finds the lateral offset directions of the omnidirectional effector
    """    
    
    if goal_position.shape[0] != 2:
        goal_position = goal_position[:2]
    if current_position.shape[0] != 2:
        current_position = current_position[:2]
    
    
    range_total = obs_radius + robot_radius
    
    ego_unit_vector = np.array([np.cos(heading_rad), np.sin(heading_rad)])
    
    #swap the direction sign to get the normal vector
    drive_by_vector_one = np.array([ego_unit_vector[1], -ego_unit_vector[0]])
    drive_by_vector_two = np.array([-ego_unit_vector[1], ego_unit_vector[0]])
    
    drive_by_vector_one = drive_by_vector_one * range_total
    drive_by_vector_two = drive_by_vector_two * range_total
    
    #pick the one closer to the current position
    distance_one = np.linalg.norm(current_position - (goal_position + drive_by_vector_one))
    distance_two = np.linalg.norm(current_position - (goal_position + drive_by_vector_two))
    
    
    if consider_obstacles and use_closest_to_ref == False:
        #make a decision baased on the obstacles
        is_right = does_right_side_have_more_obstacles(heading_rad, danger_zones,
                                                         np.zeros(len(danger_zones)),
                                                         np.zeros(len(danger_zones)))
        #
        cross_product = np.cross(ego_unit_vector, drive_by_vector_one)
        if cross_product > 0:
            #if the cross product is positive then the right vector is the left vector
            left_vector = drive_by_vector_one
            right_vector = drive_by_vector_two
        else:
            left_vector = drive_by_vector_two
            right_vector = drive_by_vector_one
        
        if is_right:
            print("right side has more obstacles")
            #if right side has more obstacles then pick the left vector
            #figure out which vector is the left vector
            drive_by_vector = left_vector
        else:
            print("left side has more")
            drive_by_vector = right_vector
    
    elif consider_obstacles and use_closest_to_ref == True:
        distance_to_ref_one = np.linalg.norm(ref_point - (goal_position + drive_by_vector_one))
        distance_to_ref_two = np.linalg.norm(ref_point - (goal_position + drive_by_vector_two))
        
        if distance_to_ref_one < distance_to_ref_two:
            drive_by_vector = drive_by_vector_two
        else:
            drive_by_vector = drive_by_vector_one        
    else:  
        if distance_one < distance_two:
            drive_by_vector = drive_by_vector_two
        else:
            drive_by_vector = drive_by_vector_one
                
        
    #apply to goal position
    drive_by_position = goal_position + drive_by_vector
    
    return drive_by_position
    
current_position = np.array([-141,-80,0])
current_heading = np.deg2rad(90)
current_velocity = 15
max_velocity = 30
min_velocity = 15
phi_max = np.deg2rad(45)
g = 9.81

r_thesholdhold = (min_velocity)**2/(g*np.tan(phi_max))
print("R Threshold:", r_thesholdhold)
random_seed = 0

np.random.seed(random_seed)

#generate random obstacles
# num_obstacles = 20
# obs_x = np.random.uniform(-150, 150, num_obstacles)
# obs_y = np.random.uniform(-150, 150, num_obstacles)
# obs_z = np.random.uniform(-10, 100, num_obstacles)
# obs_radii = np.random.uniform(5, 15, num_obstacles)
# obstacles = np.array([obs_x, obs_y, obs_z, obs_radii]).T

file_name = 'obs_2'
obs_df = pd.read_csv(file_name+'.csv')
obs_x = obs_df['x'].values
obs_y = obs_df['y'].values
obs_z = obs_df['z'].values
obs_radii = obs_df['radii'].values
obstacles = np.array([obs_x, obs_y, obs_z, obs_radii]).T
goal_position = np.array([200, 200, 50])

# Find the nearest n obstacles from the ego_position
algo_start = time.time()
start_time = time.time()
nearest_obstacles,nearest_indices  = knn_obstacles(
    obstacles, current_position, K=10, use_2d=True)
print(nearest_indices)
end_time = time.time()
print("Time taken for KNN:", end_time - start_time)

#compute dot_product
#abstract this into a function have inputs as ego_position, nearest_obstacles, current_heading, threshold
inline_obstacles = []
dot_product_threshold = 0.8
start_time = time.time()
for obs in nearest_obstacles:
    obs_2d = obs[:2]
    current_position = current_position[:2]
    los_vector = obs_2d - current_position
    #cast to float
    los_vector = los_vector.astype(float)
    los_vector /= np.linalg.norm(los_vector)
    ego_unit_vector = np.array([np.cos(current_heading), np.sin(current_heading)])
    dot_product = np.dot(ego_unit_vector, los_vector)
    if dot_product > dot_product_threshold:
        inline_obstacles.append((obs, dot_product))
        
print("inline obstacles", inline_obstacles)
end_time = time.time()
print("Time taken for dot product:", end_time - start_time)

danger_zones = []
buffer_zone = 20
for obs,dot in inline_obstacles:
    obs_2d = obs[:2]
    distance_to_obstacle = np.linalg.norm(obs_2d - current_position[:2])
    print("Distance to obstacle:", distance_to_obstacle)
    delta_r = distance_to_obstacle + obs[-1] - r_thesholdhold
    if delta_r <= buffer_zone:
        danger_zones.append((obs,dot))
        
print("algo time:", time.time() - algo_start)
print("Danger Zones: ", danger_zones)
dot_product = [d[1] for d in danger_zones]
danger_zones = [d[0] for d in danger_zones]
#get the index of the max dot product
max_dot_index = np.argmax(dot_product, axis=0)
max_dot_obs = danger_zones[max_dot_index]
print("Max Dot Obs:", max_dot_obs)
#compute the drive by position
drive_by_position = find_driveby_direction(max_dot_obs[:2], current_position[:2], current_heading,
                                             max_dot_obs[-1], r_thesholdhold/2, True, danger_zones,
                                             use_closest_to_ref=True,
                                             ref_point=goal_position[:2])
print("Drive by position:", drive_by_position)

#%%
# #plot obstacles and direction vectors
fig, ax = plt.subplots(1,1, figsize=(10,10))

#draw circles
for obs in obstacles:
    ax.add_patch(plt.Circle(obs[:2], obs[-1], fill=False, color='r'))
        
#plot ego position vector
ax.quiver(current_position[0], current_position[1], 
          np.cos(current_heading), np.sin(current_heading), 
          color='b', scale=5)

for obs in danger_zones:
    ax.scatter(obs[0], obs[1], c='g', s=100)
#plot ego position vector
ax.quiver(current_position[0], current_position[1], 
          np.cos(current_heading), np.sin(current_heading), 
          color='b', scale=5)
#draw turn radius circle on the ego position offset by the heading
ax.add_patch(plt.Circle(current_position[:2], r_thesholdhold, fill=False, color='g'))

#plot the drive by position
ax.scatter(drive_by_position[0], drive_by_position[1], c='y', s=100)
plt.show()
