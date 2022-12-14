import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
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
