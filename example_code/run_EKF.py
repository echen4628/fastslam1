"""
Author: Andrew Q. Pham
Email: apham@g.hmc.edu
Date of Creation: 2/26/20
Description:
    Extended Kalman Filter implementation to filtering localization estimate
    This code is for teaching purposes for HMC ENGR205 System Simulation Lab 3
    Student code version with parts omitted.
"""

import csv
import time
import sys
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Ellipse
import matplotlib.transforms as transforms
import numpy as np
import math
import os.path
import pdb
from tqdm import tqdm

HEIGHT_THRESHOLD = 0.0  # meters
GROUND_HEIGHT_THRESHOLD = -.4  # meters
DT = 0.1
X_LANDMARK = 5.  # meters
Y_LANDMARK = -5.  # meters
EARTH_RADIUS = 6.3781E6  # meters


def load_data(filename):
    """Load data from the csv log

    Parameters:
    filename (str)  -- the name of the csv log

    Returns:
    data (dict)     -- the logged data with data categories as keys
                       and values list of floats
    """
    is_filtered = False
    if os.path.isfile(filename + "_filtered.csv"):
        f = open(filename + "_filtered.csv")
        is_filtered = True
    else:
        f = open(filename + ".csv")

    file_reader = csv.reader(f, delimiter=',')

    # Load data into dictionary with headers as keys
    data = {}
    header = ["X", "Y", "Z", "Time Stamp", "Latitude", "Longitude",
              "Yaw", "Pitch", "Roll", "AccelX", "AccelY", "AccelZ"]
    for h in header:
        data[h] = []

    row_num = 0
    f_log = open("bad_data_log.txt", "w")
    for row in file_reader:
        for h, element in zip(header, row):
            # If got a bad value just use the previous value
            try:
                
                data[h].append(float(element))
            except ValueError:
                data[h].append(data[h][-1])
                f_log.write(str(row_num) + "\n")

        row_num += 1
    f.close()
    f_log.close()

    return data, is_filtered


def save_data(data, filename):
    """Save data from dictionary to csv

    Parameters:
    filename (str)  -- the name of the csv log
    data (dict)     -- data to log
    """
    header = ["X", "Y", "Z", "Time Stamp", "Latitude", "Longitude",
              "Yaw", "Pitch", "Roll", "AccelX", "AccelY", "AccelZ"]
    f = open(filename, "w")
    num_rows = len(data["X"])
    for i in range(num_rows):
        for h in header:
            f.write(str(data[h][i]) + ",")

        f.write("\n")

    f.close()


def filter_data(data):
    """Filter lidar points based on height and duplicate time stamp

    Parameters:
    data (dict)             -- unfilterd data

    Returns:
    filtered_data (dict)    -- filtered data
    """

    # Remove data that is not above a height threshold to remove
    # ground measurements and remove data below a certain height
    # to remove outliers like random birds in the Linde Field (fuck you birds)
    filter_idx = [idx for idx, ele in enumerate(data["Z"])
                  if ele > GROUND_HEIGHT_THRESHOLD and ele < HEIGHT_THRESHOLD]

    filtered_data = {}
    for key in data.keys():
        filtered_data[key] = [data[key][i] for i in filter_idx]

    # Remove data that at the same time stamp
    ts = filtered_data["Time Stamp"]
    filter_idx = [idx for idx in range(1, len(ts)) if ts[idx] != ts[idx-1]]
    for key in data.keys():
        filtered_data[key] = [filtered_data[key][i] for i in filter_idx]

    return filtered_data


def convert_gps_to_xy(lat_gps, lon_gps, lat_origin, lon_origin):
    """Convert gps coordinates to cartesian with equirectangular projection

    Parameters:
    lat_gps     (float)    -- latitude coordinate
    lon_gps     (float)    -- longitude coordinate
    lat_origin  (float)    -- latitude coordinate of your chosen origin
    lon_origin  (float)    -- longitude coordinate of your chosen origin

    Returns:
    x_gps (float)          -- the converted x coordinate
    y_gps (float)          -- the converted y coordinate
    """
    x_gps = EARTH_RADIUS*(math.pi/180.)*(lon_gps - lon_origin) * \
        math.cos((math.pi/180.)*lat_origin)
    y_gps = EARTH_RADIUS*(math.pi/180.)*(lat_gps - lat_origin)

    return x_gps, y_gps


def wrap_to_pi(angle):
    """Wrap angle data in radians to [-pi, pi]

    Parameters:
    angle (float)   -- unwrapped angle

    Returns:
    angle (float)   -- wrapped angle
    """
    while angle >= math.pi:
    # while angle >= 2*math.pi:
        angle -= 2*math.pi

    while angle <= -math.pi:
    # while angle <=0:
        angle += 2*math.pi
    return angle


def propogate_state(x_t_prev, u_t):
    """Propogate/predict the state based on chosen motion model

    Parameters:
    x_t_prev (np.array)  -- the previous state estimate
    u_t (np.array)       -- the current control input

    Returns:
    x_bar_t (np.array)   -- the predicted state
    """
    """STUDENT CODE START"""
    delta_t = 0.1  # based on sampling rate of 10Hz
    # x_bar_t = np.array([])
    x_bar_x_t = x_t_prev[0] + x_t_prev[2]*delta_t
    x_bar_y_t = x_t_prev[1] + x_t_prev[3]*delta_t
    v_bar_x_t = x_t_prev[2] + u_t[0] * \
        math.cos(x_t_prev[4])*delta_t  # needs to be in radian
    v_bar_y_t = x_t_prev[3] + u_t[0]*math.sin(x_t_prev[4])*delta_t
    theta_bar_t = x_t_prev[4] + u_t[1]*delta_t
    x_bar_t = np.array(
        [x_bar_x_t, x_bar_y_t, v_bar_x_t, v_bar_y_t, theta_bar_t])
    """STUDENT CODE END"""

    return x_bar_t


def calc_prop_jacobian_x(x_t_prev, u_t):
    """Calculate the Jacobian of your motion model with respect to state

    Parameters:
    x_t_prev (np.array) -- the previous state estimate
    u_t (np.array)      -- the current control input

    Returns:
    G_x_t (np.array)    -- Jacobian of motion model wrt to x
    """
    """STUDENT CODE START"""
    delta_t = 0.1
    G_x_t = np.identity(5)  # add shape of matrix
    G_x_t[0][2] = delta_t
    G_x_t[1][3] = delta_t
    G_x_t[2][4] = -1*u_t[0]*math.sin(x_t_prev[4])*delta_t
    G_x_t[3][4] = u_t[0]*math.cos(x_t_prev[4])*delta_t

    """STUDENT CODE END"""

    return G_x_t


def calc_prop_jacobian_u(x_t_prev, u_t):
    """Calculate the Jacobian of motion model with respect to control input

    Parameters:
    x_t_prev (np.array)     -- the previous state estimate
    u_t (np.array)          -- the current control input

    Returns:
    G_u_t (np.array)        -- Jacobian of motion model wrt to u
    """

    """STUDENT CODE START"""
    delta_t = 0.1
    G_u_t = np.zeros((5, 2))  # add shape of matrix
    G_u_t[2][0] = math.cos(x_t_prev[4])*delta_t
    G_u_t[3][0] = math.sin(x_t_prev[4])*delta_t
    G_u_t[4][1] = delta_t
    """STUDENT CODE END"""

    return G_u_t


def prediction_step(x_t_prev, u_t, sigma_x_t_prev):
    """Compute the prediction of EKF

    Parameters:
    x_t_prev (np.array)         -- the previous state estimate
    u_t (np.array)              -- the control input
    sigma_x_t_prev (np.array)   -- the previous variance estimate

    Returns:
    x_bar_t (np.array)          -- the predicted state estimate of time t
    sigma_x_bar_t (np.array)    -- the predicted variance estimate of time t
    """

    """STUDENT CODE START"""
    # Covariance matrix of control input
    sigma_u_t = np.identity(2)  # add shape of matrix, changed to identity matrix

    x_bar_t = propogate_state(x_t_prev, u_t)
    # call functions to get needed matrices
    G_x_t = calc_prop_jacobian_x(x_t_prev, u_t)
    G_u_t = calc_prop_jacobian_u(x_t_prev, u_t)
    # sigma_x_prev = np.identity(5) # bad

    # compute sigma_x_bar_t
    # pdb.set_trace()
    sigma_x_bar_t = G_x_t@sigma_x_prev@G_x_t.T + G_u_t@sigma_u_t@G_u_t.T
    """STUDENT CODE END"""

    return x_bar_t, sigma_x_bar_t # changed to np array format


def calc_meas_jacobian(x_bar_t):
    """Calculate the Jacobian of your measurment model with respect to state

    Parameters:
    x_bar_t (np.array)  -- the predicted state

    Returns:
    H_t (np.array)      -- Jacobian of measurment model
    """
    """STUDENT CODE START"""
    H_t = np.zeros((3, 5))
    # good because h(x_bar_t) = [x,y,theta], so jacobian is trivial
    H_t[0][0] = 1
    H_t[1][1] = 1
    H_t[2][4] = 1
    """STUDENT CODE END"""

    return H_t


def calc_kalman_gain(sigma_x_bar_t, H_t):
    """Calculate the Kalman Gain

    Parameters:
    sigma_x_bar_t (np.array)  -- the predicted state covariance matrix
    H_t (np.array)            -- the measurement Jacobian

    Returns:
    K_t (np.array)            -- Kalman Gain
    """
    """STUDENT CODE START"""
    # Covariance matrix of measurments
    sigma_z_t = np.identity(3) # changed to identity matrix
    inverse_term = np.linalg.inv(H_t@sigma_x_bar_t@H_t.T + sigma_z_t)
    K_t = sigma_x_bar_t@H_t.T@inverse_term
    
    """STUDENT CODE END"""

    return K_t


def calc_meas_prediction(x_bar_t):
    """Calculate predicted measurement based on the predicted state

    Parameters:
    x_bar_t (np.array)  -- the predicted state

    Returns:
    z_bar_t (np.array)  -- the predicted measurement
    """

    """STUDENT CODE START"""
    z_bar_t = np.array([x_bar_t[0],x_bar_t[1],x_bar_t[4]])
    """STUDENT CODE END"""

    return z_bar_t


def correction_step(x_bar_t, z_t, sigma_x_bar_t):
    """Compute the correction of EKF

    Parameters:
    x_bar_t       (np.array)    -- the predicted state estimate of time t
    z_t           (np.array)    -- the measured state of time t
    sigma_x_bar_t (np.array)    -- the predicted variance of time t

    Returns:
    x_est_t       (np.array)    -- the filtered state estimate of time t
    sigma_x_est_t (np.array)    -- the filtered variance estimate of time t
    """

    """STUDENT CODE START"""
    # get kalman gain and H
    H_t = calc_meas_jacobian(x_bar_t)
    K_t = calc_kalman_gain(sigma_x_bar_t, H_t)
    

    # define estimated meas matrix h(u_bar_t)
    h_u_bar_t = calc_meas_prediction(x_bar_t)

    # calculate outputs
    x_est_t = x_bar_t + K_t@(z_t - h_u_bar_t)
    sigma_x_est_t = (np.identity(5) - (K_t@H_t))@sigma_x_bar_t
    """STUDENT CODE END"""

    return [x_est_t, sigma_x_est_t]

def moving_average(imu, window_size):
    smoothed_imu = []
    for i in range(len(imu)):
        if i < window_size-1:
            smoothed_imu.append(np.average(imu[:i+1]))
        else:
            smoothed_imu.append(np.average(imu[i-window_size+1:i+1]))
    return np.array(smoothed_imu)


def plot_cov_at_point_t(t, covariance_estimates, x, y, ax, n_std=3):
    current_cov = covariance_estimates[:2,:2,t]
    current_x = x[t]
    current_y = y[t]
    pearson = current_cov[0, 1]/np.sqrt(current_cov[0, 0] * current_cov[1, 1])
    ellipse_rx = np.sqrt(1 + pearson)
    ellipse_ry = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width = ellipse_rx * 2, height = ellipse_ry * 2, alpha=0.1, edgecolor = "k", lw=3)
    scale_x = np.sqrt(current_cov[0, 0]) * n_std
    mean_x = np.mean(x)

    # calculating the standard deviation of y ...
    scale_y = np.sqrt(current_cov[1, 1]) * n_std
    mean_y = np.mean(y)

    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(current_x, current_y)

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)

# def moving_window(yaw, )

# def main():
#     """Run a EKF on logged data from IMU and LiDAR moving in a box formation around a landmark"""

#     filepath = "c:/Users/echen/cs/e205lab3/"
#     filename = "2020_2_26__17_21_59_filtered"
#     # data, is_filtered = load_data("2020_2_26__17_21_59_filtered")
#     data, is_filtered = load_data(filepath + filename)

#     # Save filtered data so don't have to process unfiltered data everytime
#     # if not is_filtered:
#     #     f_data = filter_data(data)
#     #     save_data(f_data, filepath+filename+"_filtered.csv")

#     print(data)
#     # Load data into variables
#     x_lidar = data["X"]
#     y_lidar = data["Y"]
#     z_lidar = data["Z"]
#     time_stamps = data["Time Stamp"]
#     lat_gps = data["Latitude"]
#     lon_gps = data["Longitude"]
#     yaw_lidar = data["Yaw"]
#     yaw_lidar = np.deg2rad(data["Yaw"])
#     pitch_lidar = data["Pitch"]
#     roll_lidar = data["Roll"]
#     x_ddot = data["AccelX"]
#     y_ddot = data["AccelY"]
#     shifted_yaw = np.concatenate([[0], yaw_lidar[:-1]])
#     yaw_velocity = wrap_to_pi(shifted_yaw - yaw_lidar)
#     print(sum(yaw_velocity > 3.16))
#     yaw_velocity = yaw_velocity/ DT

    
#     print(len(lat_gps))

#     lat_origin = lat_gps[0]
#     lon_origin = lon_gps[0]

#     #  Initialize filter
#     """STUDENT CODE START"""
#     N =  5 # number of states
#     state_est_t_prev = np.array([0,0,0,0,0]) # set everything initally as 0
#     var_est_t_prev = np.identity(N)

#     state_estimates = np.empty((N, len(time_stamps)))
#     covariance_estimates = np.empty((N, N, len(time_stamps)))
#     gps_estimates = np.empty((2, len(time_stamps)))
#     """STUDENT CODE END"""

#     #  Run filter over data
#     for t, _ in enumerate(time_stamps):
#         # Get control input
#         """STUDENT CODE START"""
#         u_t = np.array([x_ddot[t], yaw_velocity[t]]) 
#         """STUDENT CODE END"""

#         # Prediction Step
#         state_pred_t, var_pred_t = prediction_step(
#             state_est_t_prev, u_t, var_est_t_prev)

#         # Get measurement
#         """STUDENT CODE START"""
#         # print("hi")
#         z_x_t = X_LANDMARK - (y_lidar[t]*math.cos(yaw_lidar[t]) + x_lidar[t]*math.sin(yaw_lidar[t]))
#         z_y_t = Y_LANDMARK - (y_lidar[t]*math.sin(yaw_lidar[t]) - x_lidar[t]*math.cos(yaw_lidar[t]))
#         z_theta_t = yaw_lidar[t]
#         # print("got past 409")
#         z_t = np.array([z_x_t,z_y_t,z_theta_t])
#         """STUDENT CODE END"""

#         # Correction Step
#         state_est_t, var_est_t = correction_step(state_pred_t,
#                                                  z_t,
#                                                  var_pred_t)

#         #  For clarity sake/teaching purposes, we explicitly update t->(t-1)
#         state_est_t_prev = state_est_t
#         var_est_t_prev = var_est_t

#         # Log Data
#         state_estimates[:, t] = state_est_t
#         covariance_estimates[:, :, t] = var_est_t

#         x_gps, y_gps = convert_gps_to_xy(lat_gps=lat_gps[t],
#                                          lon_gps=lon_gps[t],
#                                          lat_origin=lat_origin,
#                                          lon_origin=lon_origin)
#         gps_estimates[:, t] = np.array([x_gps, y_gps])

#     """STUDENT CODE START"""
#     # Plot or print results here
#     # plt.scatter(state_estimates)
#     """STUDENT CODE END"""
#     return 0


if __name__ == "__main__":
    # main()
        # """Run a EKF on logged data from IMU and LiDAR moving in a box formation around a landmark"""
    CORRECTION_ONLY_ON_EVERY_OTHER = False
    correct_counter = 0
    filepath = "c:/Users/echen/cs/e205lab3/"
    filename = "2020_2_26__17_21_59_filtered"
    # filename = "2020_2_26__16_59_7_filtered"
    # data, is_filtered = load_data("2020_2_26__17_21_59_filtered")
    data, is_filtered = load_data(filepath + filename)

    # Save filtered data so don't have to process unfiltered data everytime
    # if not is_filtered:
    #     f_data = filter_data(data)
    #     save_data(f_data, filepath+filename+"_filtered.csv")

    print(data)
    # Load data into variables
    x_lidar = data["X"]
    y_lidar = data["Y"]
    z_lidar = data["Z"]
    time_stamps = data["Time Stamp"]
    lat_gps = data["Latitude"]
    lon_gps = data["Longitude"]
    yaw_lidar = data["Yaw"]
    yaw_lidar = -1*np.deg2rad(data["Yaw"])

    pitch_lidar = data["Pitch"]
    roll_lidar = data["Roll"]
    x_ddot = data["AccelX"]
    x_ddot = moving_average(x_ddot, 3)
    y_ddot = data["AccelY"]
    shifted_yaw = np.concatenate([[0], yaw_lidar[:-1]])
    yaw_velocity = (shifted_yaw - yaw_lidar)

    for i in range(0, len(yaw_velocity)):
        yaw_velocity[i] = wrap_to_pi(yaw_velocity[i])
    # yaw_velocity = wrap_to_pi(shifted_yaw - yaw_lidar)
    # print(sum(yaw_velocity > 3.16))
    yaw_velocity = yaw_velocity/DT
    
    print(len(lat_gps))

    lat_origin = lat_gps[0]
    lon_origin = lon_gps[0]

    #  Initialize filter
    """STUDENT CODE START"""
    N =  5 # number of states
    state_est_t_prev = np.array([0,0,0,0,0]) # set everything initally as 0
    var_est_t_prev = np.identity(N)

    state_estimates = np.empty((N, len(time_stamps)))
    covariance_estimates = np.empty((N, N, len(time_stamps)))
    gps_estimates = np.empty((2, len(time_stamps)))
    """STUDENT CODE END"""

    #  Run filter over data
    for t, _ in enumerate(time_stamps):
        # Get control input
        """STUDENT CODE START"""
        u_t = np.array([x_ddot[t], yaw_velocity[t]]) 
        """STUDENT CODE END"""

        # Prediction Step
        state_pred_t, var_pred_t = prediction_step(
            state_est_t_prev, u_t, var_est_t_prev)
        # Get measurement
        """STUDENT CODE START"""
        # print("hi")
        z_x_t = X_LANDMARK - (y_lidar[t]*math.cos(yaw_lidar[t]) + x_lidar[t]*math.sin(yaw_lidar[t]))
        z_y_t = Y_LANDMARK - (y_lidar[t]*math.sin(yaw_lidar[t]) - x_lidar[t]*math.cos(yaw_lidar[t]))
        z_theta_t = yaw_lidar[t]
        # print("got past 409")
        z_t = np.array([z_x_t,z_y_t,z_theta_t])
        """STUDENT CODE END"""

        # Correction Step
        if(CORRECTION_ONLY_ON_EVERY_OTHER and correct_counter%2 != 0):
            state_est_t, var_est_t = state_pred_t, var_pred_t
        else:
            state_est_t, var_est_t = correction_step(state_pred_t,
                                                    z_t,
                                                    var_pred_t)
        
        correct_counter += 1

        #  For clarity sake/teaching purposes, we explicitly update t->(t-1)
        state_est_t_prev = state_est_t
        var_est_t_prev = var_est_t

        # Log Data
        state_estimates[:, t] = state_est_t
        covariance_estimates[:, :, t] = var_est_t
        # pdb.set_trace()
        x_gps, y_gps = convert_gps_to_xy(lat_gps=lat_gps[t],
                                         lon_gps=lon_gps[t],
                                         lat_origin=lat_origin,
                                         lon_origin=lon_origin)
        gps_estimates[:, t] = np.array([x_gps, y_gps])

    """STUDENT CODE START"""
    # Plot or print results here
    x = state_estimates[0, :]
    y = state_estimates[1, :]
    gps_x = gps_estimates[0, :]
    gps_y = gps_estimates[1, :]
    map_fig, map_ax = plt.subplots()
    square_x = np.linspace(0,10, 1000)
    square_y = np.linspace(-10, 0, 1000)
    square_top_side = np.zeros((1000,2))
    square_right_side = np.zeros((1000,2))
    square_bot_side = np.zeros((1000,2))
    square_left_side = np.zeros((1000,2))

    square_top_side[:,0] = square_x
    square_right_side[:,0] = 10
    square_right_side[:,1] = square_y
    square_bot_side[:,0] = square_x
    square_bot_side[:,1] = -10
    square_left_side[:,1] = square_y

    rms = []
    for i in tqdm(range(len(x))):
        current_x = x[i]
        current_y = y[i]
        current_min = float("inf")
        for j in range(1000):
            current_min = min(current_min,(current_x - square_top_side[j][0])**2 + (current_y - square_top_side[j][1])**2)
            current_min = min(current_min,(current_x - square_right_side[j][0])**2 + (current_y - square_right_side[j][1])**2)
            current_min = min(current_min,(current_x - square_bot_side[j][0])**2 + (current_y - square_bot_side[j][1])**2)
            current_min = min(current_min,(current_x - square_left_side[j][0])**2 + (current_y - square_left_side[j][1])**2)
        rms.append((current_min)**(1/2))
    rms = np.array(rms)
    map_ax.add_patch(Rectangle((0,-10), 10,10, fill=False, lw=3))
    map_ax.scatter(gps_x, gps_y)
    map_ax.scatter(x, y)
    sampled_t = np.linspace(0, len(x)-1, 10, dtype=np.int32)
    for t in sampled_t:
        plot_cov_at_point_t(t, covariance_estimates, x, y, map_ax)
    #     plot_cov_at_point_t(t, covariance_estimates, x, y, map_ax)

    map_ax.set_title("Vehicle Position")
    map_ax.set_xlabel("x position")
    map_ax.set_ylabel("y position")
    map_ax.legend(["Expected", "GPS", "Estimate"])
    map_fig.show()

    rms_fig, rms_ax = plt.subplots()
    t = 0.1*np.arange(0, len(rms))
    rms_ax.plot(t, rms)
    rms_ax.set_title("Path Tracking Error vs Time")
    rms_ax.set_xlabel("time (s)")
    rms_ax.set_ylabel("path tracking error (m)")
    rms_fig.show()


    yaw_fig, yaw_ax = plt.subplots()
    yaw = state_estimates[4, :]
    for i in range(0, len(yaw)):
        yaw[i] = wrap_to_pi(yaw[i])
        # if yaw[i] < 0:
        #     yaw[i] += math.pi
    t = 0.1*np.arange(0, len(yaw))
    yaw_ax.plot(t, yaw)
    yaw_fig.show()

    cov_fig, cov_ax = plt.subplots(5,5)
    for row in range(5):
        for col in range(5):
            # pdb.set_trace()
            cov_ax[row, col].scatter(t, covariance_estimates[row, col, :])
            # cov_fig.show()
    # cov_ax[0,0] = plt.scatter([0.4,0.5], [0.4, 0.5])
    cov_fig.show()
    # cov_fig.show()


    """STUDENT CODE END"""
