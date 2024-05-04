"""
class

index in Robot1_Measurement
index in Robot1_Odometry

get next datapoint function
 - return the index at Robot1_odometry
 - check if the index in Robot1_measurement is close enough, else reject

initialization (long)

"""
import pandas as pd
class Dataloader():
    def __init__(self, odometry_data_path, measurement_data_path, robot_groundtruth_data_path=None):
        self.measurement_data_path = measurement_data_path
        self.odometry_data_path = odometry_data_path
        self.groundtruth_data_path = robot_groundtruth_data_path
        self.measurement_df = pd.read_csv(self.measurement_data_path)
        self.odometry_df = pd.read_csv(self.odometry_data_path)
        if self.groundtruth_data_path:
            self.groundtruth_df = pd.read_csv(self.groundtruth_data_path)
        self.odometry_idx = 0
        self.measurement_idx = 0
        self.groundtruth_idx = 0
        self.len = len(self.odometry_df)-1
    
    def get_next(self, threshold):
        current_odometry = self.odometry_df.loc[self.odometry_idx]
        self.odometry_idx += 1
        time = current_odometry["Time"]
        next_measurement = self.measurement_df.loc[self.measurement_idx]
        measurement_time = next_measurement["Time"]
        all_measurements = []
        while measurement_time <= time:
            if measurement_time-time <= threshold:
                all_measurements.append(next_measurement)
            self.measurement_idx += 1
            if self.measurement_idx >= len(self.measurement_df):
                break
            next_measurement = self.measurement_df.loc[self.measurement_idx]
            measurement_time = next_measurement["Time"]
        return current_odometry, all_measurements
    
    def get_next_groundtruth(self):
        if self.groundtruth_data_path:
            current_groundtruth = self.groundtruth_df.loc[self.groundtruth_idx]
            self.groundtruth_idx += 1
            return current_groundtruth
        else:
            print("Missing groundtruth data, check if path is supplied")
            return None
    
    def reset_idx(self):
        self.odometry_idx = 0
        self.measurement_idx = 0

if __name__ == "__main__":
    # loader = Dataloader("./data/Robot1_Odometry.csv", "./data/Robot1_Measurement.csv")
    loader = Dataloader("./data/Cleaned_Robot1_Odometry.csv", "./data/Cleaned_Robot1_Measurement.csv")
