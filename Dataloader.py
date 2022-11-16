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
    def __init__(self, odometry_data_path, measurement_data_path):
        self.measurement_data_path = measurement_data_path
        self.odometry_data_path = odometry_data_path
        self.measurement_df = pd.read_csv(self.measurement_data_path)
        self.odometry_df = pd.read_csv(self.odometry_data_path)
        self.odometry_idx = 0
        self.measurement_idx = 0
    
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
            next_measurement = self.measurement_df.loc[self.measurement_idx]
            measurement_time = next_measurement["Time"]
        return current_odometry, all_measurements
    
    def reset_idx(self):
        self.odometry_idx = 0
        self.measurement_idx = 0

if __name__ == "__main__":
    # loader = Dataloader("./data/Robot1_Odometry.csv", "./data/Robot1_Measurement.csv")
    loader = Dataloader("./Cleaned_Robot1_Odometry.csv", "./data/Robot1_Measurement.csv")
