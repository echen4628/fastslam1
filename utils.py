import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import pdb
def animate_path(x_positions, y_positions, est_landmark_x, est_landmark_y, groundtruth_data_path, landmark_data_path):
    groundtruth_data = pd.read_csv(groundtruth_data_path)
    landmark_data = pd.read_csv(landmark_data_path)
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
        # pdb.set_trace()
        # est_landmark_x_data = est_landmark_x_data + est_landmark_x[frame].tolist()
        # est_landmark_y_data = est_landmark_y_data + est_landmark_y[frame].tolist()
        ln_path.set_data(x_path_data, y_path_data)
        est_landmark_x_data = [est_landmark_x[frame]]
        est_landmark_y_data = [est_landmark_y[frame]]
        ln_est_landmark.set_data(est_landmark_x_data, est_landmark_y_data)
        return ln_groundtruth, ln_path, ln_est_landmark
    ax.legend(["Animated Groundtruth", "Animated Prediction Step"])
    ani = FuncAnimation(fig, update, frames=np.arange(len(x_positions)),
                    init_func=init, blit=True, interval = 10)
    plt.show()
    return ani


def save_file(data, file_name):
    saving_path = os.path.join(file_name)
    with open(saving_path, 'w') as f:
        for num in data:
            f.write(f"{num}\n")
    print(f"sucessfully wrote to {saving_path}")

def save_results(combined_x, combined_y, current_est_landmark_x, current_est_landmark_y, landmark_cov, base_path=os.path.join("results", "robot1")):
    result_files = os.listdir(base_path)
    result_files = [int(f) for f in result_files]
    if len(result_files) == 0:
        new_experiment_num = 1
    else:
        new_experiment_num = max(result_files)+1
    os.mkdir(os.path.join(base_path, str(new_experiment_num)))
    save_file(combined_x, os.path.join(base_path, str(new_experiment_num), "combined_x.txt"))
    save_file(combined_y, os.path.join(base_path, str(new_experiment_num), "combined_y.txt"))
    save_file(current_est_landmark_x, os.path.join(base_path, str(new_experiment_num), "landmark_x.txt"))
    save_file(current_est_landmark_y, os.path.join(base_path, str(new_experiment_num), "landmark_y.txt"))
    np.save(os.path.join(base_path, str(new_experiment_num), "landmark_cov.npy"), np.array(landmark_cov))


def plot_cov_at_point_t(i, covariance_estimates, x, y, ax, n_std=1):
    current_cov = covariance_estimates[i]
    current_x = x[i]
    current_y = y[i]
    pearson = current_cov[0, 1]/np.sqrt(current_cov[0, 0] * current_cov[1, 1])
    ellipse_rx = np.sqrt(1 + pearson)
    ellipse_ry = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width = ellipse_rx * 2, height = ellipse_ry * 2, alpha=0.1, edgecolor = "k", lw=3)
    scale_x = np.sqrt(current_cov[0, 0]) * n_std
    # mean_x = np.mean(x)

    # calculating the standard deviation of y ...
    scale_y = np.sqrt(current_cov[1, 1]) * n_std
    # mean_y = np.mean(y)

    # pdb.set_trace()
    transf = transforms.Affine2D() \
        .rotate_deg(45) \
        .scale(scale_x, scale_y) \
        .translate(np.array(current_x), np.array(current_y))

    ellipse.set_transform(transf + ax.transData)
    return ax.add_patch(ellipse)