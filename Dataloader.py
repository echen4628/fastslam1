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
        self.odometry_data_df = pd.read_csv(self.odometry_data_path)
        self.odometry_idx = 0
        self.measurement_idx = 0

if __name__ == "__main__":
    loader = Dataloader("./data/Robot1_Odometry.csv", "./data/Robot1_Measurement.csv")
    