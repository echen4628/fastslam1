"""
Author: Andrew Q. Pham, Victor Shia
Email: apham@g.hmc.edu, vshia@g.hmc.edu
Date of Creation: 2/26/20
Description:
    Particle filter implementation to filtering localization estimate
    This code is for teaching purposes for HMC ENGR205 System Simulation Lab 4
    Student code version with parts omitted.
"""


import matplotlib.pyplot as plt
import numpy as np
import shelve
from utils import *
from prediction import prediction_and_correction_step
from resample import resample_step
import pdb
from tqdm import tqdm
from matplotlib.patches import Rectangle


def moving_average(x, window = 10):
    return np.convolve(x, 1.0 * np.ones(window) / window, 'full')


def main():
    """Run a PF on logged data from IMU and LiDAR moving in a box formation around a landmark"""

    filename="./shelve.out"
    my_shelf = shelve.open(filename, "n") # 'n' for new

    filepath = "c:/Users/echen/cs/e205_lab4/"
    filename = "2020_2_26__17_21_59_filtered"
    data, is_filtered = load_data(filepath+filename)

    # Save filtered data so don't have to process unfiltered data everytime
    # if not is_filtered:
    #     data = filter_data(data)
    #     save_data(data, filepath+filename+"_filtered.csv")

    # Load data into variables
    x_lidar = data["X"]
    y_lidar = data["Y"]
    z_lidar = data["Z"]
    time_stamps = data["Time Stamp"]
    lat_gps = data["Latitude"]
    lon_gps = data["Longitude"]
    yaw_lidar = -1*np.deg2rad(data["Yaw"])
    pitch_lidar = data["Pitch"]
    roll_lidar = data["Roll"]
    x_ddot = data["AccelX"]
    y_ddot = data["AccelY"]

    x_ddot = moving_average(x_ddot)
    y_ddot = moving_average(y_ddot)

    lat_origin = lat_gps[0]
    lon_origin = lon_gps[0]

    #  Initialize filter
    """STUDENT CODE START"""
    rms_fig, rms_ax = plt.subplots()
    position_state = "Known Start Position"
    plt_color = 'b'
    for k in range(2):
        if k == 0:
            state_est_t_prev = np.zeros((PARTICLES, N))
        else:
            state_est_t_prev = np.random.rand(PARTICLES, N)*20
            position_state = "Random Start Position"
            plt_color = 'orange'
        # state_est_t_prev = np.zeros((PARTICLES, N))
        # state_est_t_prev = np.random.rand(PARTICLES, N)*10
        state_est_t_prev[:, -1] = 1/PARTICLES
        state_estimates = np.empty((PARTICLES, N, len(time_stamps)))
        gps_estimates = np.empty((2, len(time_stamps)))
        lidar_pos = np.empty((2, len(time_stamps)))
        average_x = np.zeros((1, len(time_stamps)))
        average_y = np.zeros((1, len(time_stamps)))
        """STUDENT CODE END"""
        c = "red"
        #  Run filter over data
        for t, _ in tqdm(enumerate(time_stamps)):
            
            # print(t)
            # Get control input
            """STUDENT CODE START"""
            # kidnap!!!
            if t == 500:
                state_est_t_prev = np.random.rand(PARTICLES, N)*10
                state_est_t_prev[:, -1] = 1/PARTICLES

            if (t == 0):
                yaw_velocity = 0
            else:
                yaw_velocity = wrap_to_pi(yaw_lidar[t] - yaw_lidar[t-1])/DT  

            u_t = np.array([x_ddot[t], yaw_velocity])
            # print(u_t)
            
            z_x_t = X_LANDMARK - (y_lidar[t]*math.cos(yaw_lidar[t]) + x_lidar[t]*math.sin(yaw_lidar[t]))
            z_y_t = Y_LANDMARK - (y_lidar[t]*math.sin(yaw_lidar[t]) - x_lidar[t]*math.cos(yaw_lidar[t]))
            z_theta_t = yaw_lidar[t]

            z_t = np.array([z_x_t,z_y_t,z_theta_t])
            """STUDENT CODE END"""

            # Prediction Step
            # if t == 222:
            #     pdb.set_trace()
            state_pred_t = prediction_and_correction_step(state_est_t_prev, u_t, z_t)
            average_x[0, t] = np.average(state_pred_t[:, 0], weights = state_pred_t[:,5])
            average_y[0, t] = np.average(state_pred_t[:, 1], weights = state_pred_t[:,5])
            state_estimates[:, :, t] = state_pred_t

            state_est_t = resample_step(state_pred_t)
            #  For clarity sake/teaching purposes, we explicitly update t->(t-1)
            state_est_t_prev = state_est_t

            # Log Data
            # state_estimates[:, :, t] = state_est_t
            # average_x[0, t] = np.mean(state_est_t, axis=0)[0]
            # average_y[0, t] = np.mean(state_est_t, axis=0)[1]

            x_gps, y_gps = convert_gps_to_xy(lat_gps=lat_gps[t],
                                                lon_gps=lon_gps[t],
                                                lat_origin=lat_origin,
                                                lon_origin=lon_origin)
            gps_estimates[:, t] = np.array([x_gps, y_gps])
            # lidar_pos[:,t] = np.array([z_x, z_y])
            lidar_pos[:,t] = np.array([z_x_t, z_y_t])

        for key in dir():
            try:
                my_shelf[key] = eval(key)
            except Exception:
                #
                # __builtins__, my_shelf, and imported modules can not be shelved.
                #
                print('ERROR shelving: {0}'.format(key))
        my_shelf.close()


        """STUDENT CODE START"""
        # RMS
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
        for i in tqdm(range(average_x.shape[1])):
            current_x = average_x[0, i]
            current_y = average_y[0, i]
            current_min = float("inf")
            # pdb.set_trace()
            for j in range(1000):
                current_min = min(current_min,(current_x - square_top_side[j][0])**2 + (current_y - square_top_side[j][1])**2)
                current_min = min(current_min,(current_x - square_right_side[j][0])**2 + (current_y - square_right_side[j][1])**2)
                current_min = min(current_min,(current_x - square_bot_side[j][0])**2 + (current_y - square_bot_side[j][1])**2)
                current_min = min(current_min,(current_x - square_left_side[j][0])**2 + (current_y - square_left_side[j][1])**2)
            rms.append((current_min)**(1/2))
        rms = np.array(rms)

        # Plot here
        map_fig, ax = plt.subplots()
        ax.scatter(state_estimates[:, 0, 0], state_estimates[:, 1, 100], alpha=0.5)
        ax.scatter(state_estimates[:, 0, 200], state_estimates[:, 1, 200], alpha=0.5)
        ax.scatter(state_estimates[:, 0, 400], state_estimates[:, 1, 400], alpha=0.5)
        ax.scatter(state_estimates[:, 0, 500], state_estimates[:, 1, 500], alpha=0.5)
        ax.scatter(average_x, average_y)
        ax.scatter(gps_estimates[0,:], gps_estimates[1,:], c="lawngreen")
        ax.add_patch(Rectangle((0,-10), 10,10, fill=False, lw=3))

        ax.set_title("Estimated Position of Robot with " + position_state)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(["10s", "20s", "40s", "50s","Particle Filter", "GPS", "Expected"])
        map_fig.show()
        # ax.plot(state_estimates[0])

        # RMS
        #rms_fig, rms_ax = plt.subplots()
        t = 0.1*np.arange(0, len(rms))
        rms_ax.plot(t, rms, color = plt_color)

    rms_ax.set_title("Path Tracking Error vs Time")
    rms_ax.set_xlabel("time (s)")
    rms_ax.set_ylabel("path tracking error (m)")
    rms_ax.legend(["Known Start Position", "Random Start Position"])
    rms_fig.show()
    plt.show()
    


    
    """STUDENT CODE END"""
    return 0


if __name__ == "__main__":
    main()
