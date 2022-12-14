import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import Dataloader
import FASTSlam
import pdb
import os
from tqdm import tqdm
from utils import *

ROBOT_NUM = 2

# load in the csv with pandas
groundtruth_data = pd.read_csv(f"./data/Cleaned_Robot{ROBOT_NUM}_Groundtruth.csv")
# landmark_data = pd.read_csv("./data/Landmark_Groundtruth copy.csv")
landmark_data = pd.read_csv("./data/Landmark_Groundtruth copy.csv")

odometry_data = pd.read_csv(f"./data/Cleaned_Robot{ROBOT_NUM}_Odometry.csv")
groundtruth_x = groundtruth_data["X"]
groundtruth_y = groundtruth_data["Y"]
groundtruth_yaw = groundtruth_data["Orientation"]

landmark_subject = landmark_data["Subject"]
landmark_x = landmark_data["X"]
landmark_y = landmark_data["Y"]
landmark_x_cov = landmark_data["X-std-dev"]
landmark_y_cov = landmark_data["Y-std-dev"]
dt = 1

# define landmark state groundtruth and covariance
landmark_pos = []
landmark_cov = []
for i in range(len(landmark_x)):
    landmark_pos.append([landmark_x[i], landmark_y[i]])
    landmark_cov.append([[landmark_x_cov[i], 0], [0, landmark_y_cov[i]]])
landmark_pos = np.array(landmark_pos)
landmark_cov = np.array(landmark_cov)
  

# FastSlam
starting_state = np.array([groundtruth_x[0], groundtruth_y[0], groundtruth_yaw[0]])
filter = FASTSlam.Fastslam(100, starting_state)
dataloader = Dataloader.Dataloader(f"./data/Cleaned_Robot{ROBOT_NUM}_Odometry.csv", f"./data/Cleaned_Robot{ROBOT_NUM}_Measurement.csv", f"./data/Cleaned_Robot{ROBOT_NUM}_Groundtruth.csv")

# resulting x list
# resulitng y list
combined_x = []
combined_y = []
current_est_landmark_x = []
current_est_landmark_y = []
final_landmark = None

# static plot
static_fig, static_ax = plt.subplots()
static_ax.scatter(groundtruth_x, groundtruth_y)
static_ax.scatter(landmark_x, landmark_y)
for i, landmark in enumerate(zip(landmark_x, landmark_y)):
    x, y = landmark
    static_ax.text(x, y, str(i+6), color="red", fontsize=12)
static_ax.set_title("Robot Path and Landmark")
static_ax.set_xlabel("X (m)")
static_ax.set_ylabel("Y (m)")
# plt.show()
# pdb.set_trace()
for i in tqdm(range(dataloader.len)):
    # print(i)
    # if i == 500:
    #     filter.set_all_particle_state(np.array([5,5,0]))
        # fake_map = np.ones((15,2))*2
        # filter.set_all_particle_landmark(fake_map)


    current_odometry, all_measurements = dataloader.get_next(0)
    u_t_noiseless = np.array([current_odometry["Forward-velocity"], current_odometry["Angular-velocity"]])
    filter.propagate_all_states(u_t_noiseless, dt, True)
    # filter.set_position_to_groundtruth(np.array([groundtruth_x[i+10], groundtruth_y[i+10], groundtruth_yaw[i+10]]))
    # filter.set_landmark_to_groundtruth(landmark_pos, landmark_cov)
    # print(filter.particles[0].state)
    filter.reweight_and_update(all_measurements)
    combined_state, combined_landmark= filter.combine_particles()
    # print(filter.particles)
    filter.resample()
    # print(filter.particles)
    static_ax.scatter(combined_state[0], combined_state[1], c="g")

    # for animation
    combined_x.append(combined_state[0])
    combined_y.append(combined_state[1])
    current_est_landmark_x.append(combined_landmark[:,0])
    current_est_landmark_y.append(combined_landmark[:,1])
    final_landmark = combined_landmark

static_ax.scatter(final_landmark[:, 0], final_landmark[:,1])
for i, landmark in enumerate(zip(final_landmark[:,0], final_landmark[:,1])):
    x, y = landmark
    static_ax.text(x, y, str(i+6), color="red", fontsize=12)

# Plot RMS of robot position x, y
rms_robot = []
for i in range(len(combined_x)):
    robot_distance_sq = (combined_x[i] - groundtruth_x[i+10])**2 + (combined_y[i] - groundtruth_y[i+10])**2
    rms_robot.append(robot_distance_sq**(0.5))

shifted_groundtruth_times = np.arange(odometry_data["Time"][0], odometry_data["Time"][len(odometry_data)-1], 1)

rms_fig, rms_ax = plt.subplots()
rms_ax.plot(shifted_groundtruth_times, rms_robot)
rms_ax.set_title("Robot Path Tracking Error vs Time")
rms_ax.set_xlabel("time (s)")
rms_ax.set_ylabel("path tracking error (m)")
rms_fig.show()


# plot covariance for landmark 6 (generalize to all later)
# cov_fig, cov_ax = plt.subplots(2, 1)
# landmark6_xx_cov = 1 # get xx cov values
# landmark6_yy_cov = 1 # get yy cov values


# rms_landmark_fig, rms_landmark_ax = plt.subplots()
# pdb.set_trace()
rms_landmark = ((final_landmark[:,0] - np.array(landmark_x))**2 + (final_landmark[:,1] - np.array(landmark_y))**2)**(1/2)
# rms_ax.plot(shifted_groundtruth_times, rms_landmark)
# rms_ax.set_title("Mapping Error vs Time")
# rms_ax.set_xlabel("time (s)")
# rms_ax.set_ylabel("mapping error (m)")
# rms_fig.show()
print(rms_landmark)

ani = animate_path(combined_x, combined_y, current_est_landmark_x, current_est_landmark_y, f"./data/Cleaned_Robot{ROBOT_NUM}_Groundtruth.csv", "./data/Landmark_Groundtruth.csv")
# save_file(combined_x, "combined_x_1.txt")
save_results(combined_x, combined_y, current_est_landmark_x, current_est_landmark_y, os.path.join("results", f"robot{ROBOT_NUM}"))

