from utils import *
import os
import pdb
import re

ROBOT_NUM = 2
def read_file(path, type):
    results = ""
    with open(path, 'r') as f:
        results = f.read()
    results = results.split("\n")[:-1]
    if type == "robot":
        results = [float(num) for num in results]
    elif type == "landmark":
        pdb.set_trace()
        results = list(map(lambda x: [float(i) for i in x[1:-2].split(".")], results))
        # results = [float(x) for x in results[0][1:-2].split(".")]
    print(f"successfully loaded {path}")
    return results
def read_landmark(path):
    results = ""
    with open(path, 'r') as f:
        results = f.read()
    results = re.sub('\n', ' ', results).split("] [")
    results = [re.sub('[][]', "", x) for x in results]
    results = list(map(lambda x: x.split(" "), results))
    # results = ["" in x for x in results]
    for i in range(len(results)):
        if "" in results[i]:
            results[i] = [float(x) for x in results[i] if x != ""]
    return results
def load_results(robot_num, experiment_num):
    combined_x = read_file(os.path.join("results", f"robot{robot_num}", str(experiment_num), "combined_x.txt"), "robot")
    combined_y = read_file(os.path.join("results", f"robot{robot_num}", str(experiment_num), "combined_y.txt"), "robot")
    landmark_x = read_landmark(os.path.join("results", f"robot{robot_num}", str(experiment_num), "landmark_x.txt"))
    landmark_y = read_landmark(os.path.join("results", f"robot{robot_num}", str(experiment_num), "landmark_y.txt"))
    landmark_cov = np.load(os.path.join("results", f"robot{robot_num}", str(experiment_num), "landmark_cov.npy"))
    return combined_x, combined_y, landmark_x, landmark_y, landmark_cov

# results = read_file(os.path.join("results", f"robot{ROBOT_NUM}", "2", "combined_x.txt"))
# results = results.split("\n")[:-1]
# results = [float(num) for num in results]
# print(results)
combined_x, combined_y, landmark_x, landmark_y, landmark_cov = load_results(robot_num = 2, experiment_num = 25)
animate_path(combined_x, combined_y, landmark_x, landmark_y, f"./data/Cleaned_Robot{ROBOT_NUM}_Groundtruth.csv", "./data/Landmark_Groundtruth.csv")

# load in the csv with pandas
groundtruth_data = pd.read_csv(f"./data/Cleaned_Robot{ROBOT_NUM}_Groundtruth.csv")
# landmark_data = pd.read_csv("./data/Landmark_Groundtruth copy.csv") # use this for robot 1
landmark_data = pd.read_csv("./data/Landmark_Groundtruth.csv") # use this for robot 2

odometry_data = pd.read_csv(f"./data/Cleaned_Robot{ROBOT_NUM}_Odometry.csv")
groundtruth_x = groundtruth_data["X"]
groundtruth_y = groundtruth_data["Y"]
groundtruth_yaw = groundtruth_data["Orientation"]

groundtruth_landmark_subject = landmark_data["Subject"]
groundtruth_landmark_x = landmark_data["X"]
groundtruth_landmark_y = landmark_data["Y"]
groundtruth_landmark_x_cov = landmark_data["X-std-dev"]
groundtruth_landmark_y_cov = landmark_data["Y-std-dev"]
groundtruth_landmark_cov = np.zeros((15,2,2))
for i in range(groundtruth_landmark_cov.shape[0]):
    groundtruth_landmark_cov[i] = np.array([[groundtruth_landmark_x_cov[i], 0],
                                            [0, groundtruth_landmark_y_cov[i]]])


# Calculate Mapping RMS
landmark_x_np = np.array(landmark_x[-1]).astype(np.float64)
landmark_y_np = np.array(landmark_y[-1]).astype(np.float64)
rms_landmark = ((landmark_x_np - np.array(groundtruth_landmark_x))**2 + (landmark_y_np - np.array(groundtruth_landmark_y))**2)**(1/2)
print(rms_landmark)
print(f"mean: {np.mean(rms_landmark)}")

# Calculate Localization RMS
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

# Plot static plot with legend
static_fig, static_ax = plt.subplots()
static_ax.scatter(groundtruth_x, groundtruth_y, color="green")
static_ax.scatter(combined_x, combined_y, color="blue")

static_ax.scatter(groundtruth_landmark_x, groundtruth_landmark_y, color="orange")
for i, landmark in enumerate(zip(groundtruth_landmark_x, groundtruth_landmark_y)):
    x, y = landmark
    static_ax.text(x, y, str(i+6), color="red", fontsize=12)
    plot_cov_at_point_t(i, groundtruth_landmark_cov, groundtruth_landmark_x, groundtruth_landmark_y, static_ax, 3)


static_ax.scatter(landmark_x_np, landmark_y_np, color="purple")
for i, landmark in enumerate(zip(landmark_x_np, landmark_y_np)):
    x, y = landmark
    static_ax.text(x, y, str(i+6), color="red", fontsize=12)
    plot_cov_at_point_t(i, landmark_cov[-1], landmark_x_np, landmark_y_np, static_ax, 3)


# plot_cov_at_point_t(0, landmark_cov, landmark_x_np, landmark_y_np, static_ax, 3)
# static_ax = plot_cov_at_point_t(0, landmark_cov, landmark_x_np, landmark_y_np, static_ax)

static_ax.set_title("Robot Path and Landmark")
static_ax.set_xlabel("X (m)")
static_ax.set_ylabel("Y (m)")
static_ax.legend(["Groundtruth Path", "Estimated Path", "Grountruth Landmark", "Estimated Landmark"])
plt.show()
# print(landmark_cov[-1])
# print(landmark_cov[-1].shape)