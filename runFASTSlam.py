import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import Dataloader
import FASTSlam
import pdb

# load in the csv with pandas
groundtruth_data = pd.read_csv("./data/Cleaned_Robot1_Groundtruth.csv")
landmark_data = pd.read_csv("./data/Landmark_Groundtruth.csv")
groundtruth_x = groundtruth_data["X"]
groundtruth_y = groundtruth_data["Y"]
landmark_x = landmark_data["X"]
landmark_y = landmark_data["Y"]
dt = 1

# FastSlam
dataloader = Dataloader.Dataloader("./data/Cleaned_Robot1_Odometry.csv", "./data/Cleaned_Robot1_Measurement.csv", "./data/Cleaned_Robot1_Groundtruth.csv")

filter = FASTSlam.Fastslam(5)
for i in range(dataloader.len):
    pdb.set_trace()
    current_odometry, all_measurements = dataloader.get_next(0)
    u_t_noiseless = np.array([current_odometry["Forward-velocity"], current_odometry["Angular-velocity"]])
    filter.propagate_all_states(u_t_noiseless, dt)
    combined_state = filter.combine_particles()

# plot all points
# static_fig, static_ax = plt.subplots()
# static_ax.scatter(groundtruth_x, groundtruth_y)
# static_ax.scatter(landmark_x, landmark_y)
# plt.show()

# # example code
fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = ax.plot([], [], 'ro')

def init():
    ax.set_xlim(-1, 6)
    ax.set_ylim(-6, 6)
    ax.scatter(groundtruth_x, groundtruth_y)
    ax.scatter(landmark_x, landmark_y)
    return ln,

def update(frame):
    xdata.append(groundtruth_x[frame])
    ydata.append(groundtruth_y[frame])
    ln.set_data(xdata, ydata)
    return ln,

ani = FuncAnimation(fig, update, frames=np.arange(1501),
                    init_func=init, blit=True, interval = 10)
plt.show()




