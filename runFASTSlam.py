import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import Dataloader
import FASTSlam
import pdb
from tqdm import tqdm

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

# resulting x list
# resulitng y list
filter = FASTSlam.Fastslam(1000)
combined_x = []
combined_y = []
current_est_landmark_x = []
current_est_landmark_y = []

static_fig, static_ax = plt.subplots()
static_ax.scatter(groundtruth_x, groundtruth_y)
static_ax.scatter(landmark_x, landmark_y)
static_ax.scatter(combined_x, combined_y)
final_landmark = None
# plt.show()
for i in tqdm(range(dataloader.len)):
    # pdb.set_trace()
    # print(i)
    current_odometry, all_measurements = dataloader.get_next(0)
    u_t_noiseless = np.array([current_odometry["Forward-velocity"], current_odometry["Angular-velocity"]])
    filter.propagate_all_states(u_t_noiseless, dt)
    # print(filter.particles[0].state)
    filter.reweight_and_update(all_measurements)
    # filter.resample()
    combined_state, combined_landmark= filter.combine_particles()
    # print(filter.particles)
    filter.resample()
    # print(filter.particles)
    static_ax.scatter(combined_state[0], combined_state[1], c="g")
    #resample
    # combined_state is a 3x1 array
    # append to that list

    combined_x.append(combined_state[0])
    combined_y.append(combined_state[1])
    current_est_landmark_x.append(combined_landmark[:,0])
    current_est_landmark_y.append(combined_landmark[:,1])
    final_landmark = combined_landmark


static_ax.scatter(combined_landmark[:, 0], combined_landmark[:,1])
# plot all points
static_fig, static_ax = plt.subplots()
static_ax.scatter(groundtruth_x, groundtruth_y)
static_ax.scatter(landmark_x, landmark_y)
static_ax.scatter(combined_x, combined_y)
static_ax.set_title("Ground Truth Robot Path and Landmark")
static_ax.set_xlabel("X (m)")
static_ax.set_ylabel("Y (m)")
plt.show()

# Plot RMS of robot position x, y
rms_robot = []
for i in range(len(combined_x)):
    robot_distance_sq = (combined_x[i] - groundtruth_x[i+10])**2 + (combined_y[i] - groundtruth_y[i+10])**2
    rms_robot.append(robot_distance_sq**(0.5))
shifted_groundtruth_times = np.arange(1248272273, 1248272273+1491, 1)

rms_fig, rms_ax = plt.subplots()
rms_ax.plot(shifted_groundtruth_times, rms_robot)
rms_ax.set_title("Robot Path Tracking Error vs Time")
rms_ax.set_xlabel("time (s)")
rms_ax.set_ylabel("path tracking error (m)")
rms_fig.show()

def animate_path(x_positions, y_positions, est_landmark_x, est_landmark_y):
    groundtruth_data = pd.read_csv("./data/Cleaned_Robot1_Groundtruth.csv")
    landmark_data = pd.read_csv("./data/Landmark_Groundtruth.csv")
    groundtruth_x = groundtruth_data["X"]
    groundtruth_y = groundtruth_data["Y"]
    landmark_x = landmark_data["X"]
    landmark_y = landmark_data["Y"]

    # figure
    fig, ax = plt.subplots()
    x_groundtruth_data, y_groundtruth_data = [], []
    x_path_data, y_path_data = [], []
    est_landmark_x_data, est_landmark_y_data = [], []
    ln_groundtruth, = ax.plot([], [], 'ro')
    ln_path, = ax.plot([], [], 'bo')
    ln_est_landmark, = ax.plot([], [], 'go')

    def init():
        ax.set_xlim(-1, 6)
        ax.set_ylim(-6, 6)
        ax.set_xlabel("x position [m]")
        ax.set_ylabel("y position [m]")
        ax.scatter(groundtruth_x, groundtruth_y)
        ax.scatter(landmark_x, landmark_y)
        return ln_groundtruth, ln_path, ln_est_landmark

    def update(frame):
        x_groundtruth_data.append(groundtruth_x[frame + 10])
        y_groundtruth_data.append(groundtruth_y[frame + 10])
        ln_groundtruth.set_data(x_groundtruth_data, y_groundtruth_data)
        x_path_data.append(x_positions[frame])
        y_path_data.append(y_positions[frame])
        ln_path.set_data(x_path_data, y_path_data)
        est_landmark_x_data = [est_landmark_x[frame]]
        est_landmark_y_data = [est_landmark_y[frame]]
        ln_est_landmark.set_data(est_landmark_x_data, est_landmark_y_data)
        return ln_groundtruth, ln_path, ln_est_landmark

    ani = FuncAnimation(fig, update, frames=np.arange(len(x_positions)),
                    init_func=init, blit=True, interval = 10)
    ax.legend(["Animated Groundtruth", "Animated Prediction Step"])
    plt.show()

    return ani


ani = animate_path(combined_x, combined_y, current_est_landmark_x, current_est_landmark_y)
# FFwrite = animation.PillowWriter(fps = 10)
# ani.save("animation.gif")


# # # example code
# fig, ax = plt.subplots()
# xdata, ydata = [], []
# ln, = ax.plot([], [], 'ro')

# def init():
#     ax.set_xlim(-1, 6)
#     ax.set_ylim(-6, 6)
#     ax.scatter(groundtruth_x, groundtruth_y)
#     ax.scatter(landmark_x, landmark_y)
#     return ln,

# def update(frame):
#     xdata.append(combined_x[frame])
#     ydata.append(combined_y[frame])
#     ln.set_data(xdata, ydata)
#     return ln,

# ani = FuncAnimation(fig, update, frames=np.arange(1501),
#                     init_func=init, blit=True, interval = 10)
# plt.show()




