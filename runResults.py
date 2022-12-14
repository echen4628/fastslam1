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
combined_x, combined_y, landmark_x, landmark_y, landmark_cov = load_results(robot_num = 2, experiment_num = 24)
animate_path(combined_x, combined_y, landmark_x, landmark_y, f"./data/Cleaned_Robot{ROBOT_NUM}_Groundtruth.csv", "./data/Landmark_Groundtruth.csv")
# print(landmark_cov[-1])
# print(landmark_cov[-1].shape)