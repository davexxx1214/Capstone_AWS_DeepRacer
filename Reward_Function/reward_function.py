import math

def reward_function(params):
    ################## HELPER FUNCTIONS ###################
    def dist_2_points(x1, x2, y1, y2):
        return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

    def closest_2_racing_points_index(racing_coords, car_coords):
        # Calculate all distances to racing points
        distances = []
        for i in range(len(racing_coords)):
            distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                     y1=racing_coords[i][1], y2=car_coords[1])
            distances.append(distance)

        # Get index of the closest racing point
        closest_index = distances.index(min(distances))

        # Get index of the second closest racing point
        distances_no_closest = distances.copy()
        distances_no_closest[closest_index] = 999
        second_closest_index = distances_no_closest.index(min(distances_no_closest))

        return [closest_index, second_closest_index]

    def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
        # Calculate the distances between 2 closest racing points
        a = abs(dist_2_points(x1=closest_coords[0],
                              x2=second_closest_coords[0],
                              y1=closest_coords[1],
                              y2=second_closest_coords[1]))

        # Distances between car and closest and second closest racing point
        b = abs(dist_2_points(x1=car_coords[0],
                              x2=closest_coords[0],
                              y1=car_coords[1],
                              y2=closest_coords[1]))
        c = abs(dist_2_points(x1=car_coords[0],
                              x2=second_closest_coords[0],
                              y1=car_coords[1],
                              y2=second_closest_coords[1]))

        # Calculate distance between car and racing line (goes through 2 closest racing points)
        try:
            distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                           (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
        except:
            distance = b

        return distance

    def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
        # Virtually set the car more into the heading direction
        heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
        new_car_coords = [car_coords[0]+heading_vector[0],
                          car_coords[1]+heading_vector[1]]

        # Calculate distance from new car coords to 2 closest racing points
        distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                    x2=closest_coords[0],
                                                    y1=new_car_coords[1],
                                                    y2=closest_coords[1])
        distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                           x2=second_closest_coords[0],
                                                           y1=new_car_coords[1],
                                                           y2=second_closest_coords[1])

        if distance_closest_coords_new <= distance_second_closest_coords_new:
            next_point_coords = closest_coords
            prev_point_coords = second_closest_coords
        else:
            next_point_coords = second_closest_coords
            prev_point_coords = closest_coords

        return [next_point_coords, prev_point_coords]

    def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
        # Calculate the direction of the center line based on the closest waypoints
        next_point, prev_point = next_prev_racing_point(closest_coords,
                                                        second_closest_coords,
                                                        car_coords,
                                                        heading)

        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        track_direction = math.atan2(next_point[1] - prev_point[1],
                                   next_point[0] - prev_point[0])

        # Convert to degree
        track_direction = math.degrees(track_direction)

        # Calculate the difference between the track direction and the heading direction of the car
        direction_diff = abs(track_direction - heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff

        return direction_diff

    def indexes_cyclical(start, end, array_len):
        if end < start:
            end += array_len
        return [index % array_len for index in range(start, end)]

    def projected_time(first_index, closest_index, step_count, times_list):
        # Calculate how much time has passed since start
        current_actual_time = (step_count-1) / 15

        # Calculate which indexes were already passed
        indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

        # Calculate how much time should have passed if car would have followed optimals
        current_expected_time = sum([times_list[i] for i in indexes_traveled])

        # Calculate how long one entire lap takes if car follows optimals
        total_expected_time = sum(times_list)

        # Calculate how long car would take for entire lap, if it continued like it did until now
        try:
            projected_time = (current_actual_time/current_expected_time) * total_expected_time
        except:
            projected_time = 9999

        return projected_time

    #################### RACING LINE ######################

    # Optimal racing line for the invent track
    racing_track = [[2.91, 0.68319, 4.0, 0.04],
                    [3.32, 0.68334, 4.0, 0.1025],
                    [3.42, 0.68337, 4.0, 0.025],
                    [3.63, 0.68345, 4.0, 0.0525],
                    [4.19, 0.68365, 4.0, 0.14],
                    [4.5, 0.68376, 2.81041, 0.11031],
                    [4.55, 0.68378, 2.14147, 0.02335],
                    [5.32, 0.68405, 1.89927, 0.40542],
                    [5.42, 0.68409, 1.79647, 0.05566],
                    [5.78, 0.68422, 1.74631, 0.20615],
                    [6.22029, 0.69528, 1.74274, 0.25272],
                    [6.40459, 0.72226, 1.74274, 0.10688],
                    [6.55489, 0.76197, 1.74274, 0.0892],
                    [6.69764, 0.82037, 1.73564, 0.08886],
                    [6.83824, 0.90421, 1.61879, 0.10113],
                    [6.97477, 1.02121, 1.61879, 0.11107],
                    [7.08704, 1.15896, 1.61879, 0.10977],
                    [7.16979, 1.30619, 1.61879, 0.10433],
                    [7.26217, 1.69748, 1.61879, 0.24837],
                    [7.26007, 1.7961, 1.61879, 0.06094],
                    [7.24323, 1.91106, 1.66561, 0.06976],
                    [7.10688, 2.23278, 1.69642, 0.20597],
                    [6.94098, 2.42241, 1.79887, 0.14007],
                    [6.71618, 2.57805, 1.97234, 0.13862],
                    [6.44873, 2.68682, 2.21492, 0.13035],
                    [6.15711, 2.74745, 1.99187, 0.14954],
                    [5.92638, 2.76662, 1.99187, 0.11624],
                    [5.73153, 2.77005, 1.99187, 0.09784],
                    [5.67, 2.76989, 1.99187, 0.03089],
                    [5.22188, 2.76712, 1.99187, 0.22498],
                    [5.08777, 2.78503, 1.99187, 0.06793],
                    [4.95177, 2.81801, 2.01294, 0.06952],
                    [4.78882, 2.87846, 2.17449, 0.07993],
                    [4.57745, 2.99056, 2.39122, 0.10006],
                    [4.34885, 3.15577, 2.7044, 0.10429],
                    [4.14916, 3.3382, 2.04149, 0.13249],
                    [3.96942, 3.53023, 1.71851, 0.15306],
                    [3.774, 3.76141, 1.66754, 0.18153],
                    [3.68239, 3.8738, 1.66754, 0.08695],
                    [3.54352, 4.04108, 1.66754, 0.13038],
                    [3.3381, 4.24993, 1.66754, 0.17567],
                    [3.2041, 4.34699, 1.66754, 0.09922],
                    [3.08068, 4.40982, 1.66754, 0.08305],
                    [2.95991, 4.45063, 1.77239, 0.07192],
                    [2.83518, 4.4763, 2.05766, 0.06189],
                    [2.69021, 4.49161, 2.03471, 0.07165],
                    [2.49596, 4.49765, 1.96184, 0.09906],
                    [2.24938, 4.49143, 1.81066, 0.13623],
                    [1.98385, 4.4703, 1.81066, 0.14711],
                    [1.70212, 4.39882, 1.81066, 0.16052],
                    [1.41909, 4.25715, 1.81066, 0.1748],
                    [1.16364, 4.04074, 1.81066, 0.1849],
                    [0.9604, 3.73853, 1.81066, 0.20114],
                    [0.86507, 3.30778, 2.4682, 0.17874],
                    [0.87235, 2.81551, 2.83904, 0.17341],
                    [0.89401, 2.65856, 3.43482, 0.04613],
                    [0.91646, 2.53095, 2.467, 0.05252],
                    [0.93804, 2.41959, 1.53521, 0.07389],
                    [1.02121, 2.01818, 1.31212, 0.31243],
                    [1.04306, 1.91271, 1.3, 0.08286],
                    [1.09363, 1.66868, 1.3, 0.1917],
                    [1.20808, 1.22352, 1.3, 0.35356],
                    [1.249, 1.12478, 1.3, 0.08222],
                    [1.29054, 1.05152, 1.3, 0.06478],
                    [1.3427, 0.98491, 1.3, 0.06508],
                    [1.41091, 0.92173, 1.40017, 0.06639],
                    [1.50213, 0.86061, 1.62608, 0.06753],
                    [1.63657, 0.79697, 2.05673, 0.07232],
                    [2.06562, 0.69631, 2.84352, 0.15498],
                    [2.75, 0.68314, 4.0, 0.17113]]

    ################## INPUT PARAMETERS ###################

    # Read all input parameters
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    distance_from_center = params['distance_from_center']
    is_left_of_center = params['is_left_of_center']
    heading = params['heading']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    is_offtrack = params['is_offtrack']

    ############### OPTIMAL X,Y,SPEED,TIME ################

    # Get closest indexes for racing line
    closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])

    # Get optimal [x, y, speed, time] for closest and second closest index
    optimals = racing_track[closest_index]
    optimals_second = racing_track[second_closest_index]

    ################ REWARD AND PUNISHMENT ################

    ## Define the default reward ##
    reward = 1

    ## Reward if car goes close to optimal racing line ##
    DISTANCE_MULTIPLE = 1
    dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
    distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
    reward += distance_reward * DISTANCE_MULTIPLE

    ## Reward if speed is close to optimal speed ##
    SPEED_DIFF_NO_REWARD = 1
    SPEED_MULTIPLE = 2
    speed_diff = abs(optimals[2]-speed)
    if speed_diff <= SPEED_DIFF_NO_REWARD:
        speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
    else:
        speed_reward = 0
    reward += speed_reward * SPEED_MULTIPLE

    # Zero reward if obviously wrong direction (e.g. spin)
    direction_diff = racing_direction_diff(optimals[0:2], optimals_second[0:2], [x, y], heading)
    if direction_diff > 30:
        reward = 1e-3
        
    # Zero reward of obviously too slow
    speed_diff_zero = optimals[2]-speed
    if speed_diff_zero > 0.5:
        reward = 1e-3

    ## Incentive for finishing the lap in less steps ##
    REWARD_FOR_FASTEST_TIME = 1500
    STANDARD_TIME = 10
    FASTEST_TIME = 8.72
    if progress == 100:
        finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                  (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
    else:
        finish_reward = 0
    reward += finish_reward
    
    ## Zero reward if off track ##
    if all_wheels_on_track == False:
        reward = 1e-3

    #################### RETURN REWARD ####################
    
    return float(reward)