import csv
import pandas as pd

def clean_odometry():
    filepath = "/Users/Erina/e205labs/fastslam1/fastslam1/data/"
    filename = "Robot1_Odometry_Copy"

    data = pd.read_csv(filepath + filename + '.csv')

    timestamp = data["# Time [s]"]
    forward_v = data["forward velocity [m/s]"]
    angular_v = data["angular velocity[rad/s]"]

    # establish csv file for filtered data [TODO, may not be needed]
    header = ["# Time [s]", "forward velocity [m/s]", "angular velocity[rad/s]"]

    # loop through time stamps
    filtered_data = []
    for i in range(len(timestamp)):
        matching_timestamp_list = data[timestamp == timestamp[i]]
        if (i == 0):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            timestamp_avg_fv = matching_timestamp_list["forward velocity [m/s]"].mean()
            timestamp_avg_av = matching_timestamp_list["angular velocity[rad/s]"].mean()
            timestamp_avg_data = [timestamp[i], timestamp_avg_fv, timestamp_avg_av]
            filtered_data.append(timestamp_avg_data)
        elif(timestamp[i] != timestamp[i-1]):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            timestamp_avg_fv = matching_timestamp_list["forward velocity [m/s]"].mean()
            timestamp_avg_av = matching_timestamp_list["angular velocity[rad/s]"].mean()
            timestamp_avg_data = [timestamp[i], timestamp_avg_fv, timestamp_avg_av]
            filtered_data.append(timestamp_avg_data)
        else:
            # go to next timestamp, indexed point should already have been accounted for
            pass

    # print(filtered_data) 
  
        
    csv_filename = 'Cleaned_Robot1_Odometry.csv'
    print("Beginning transfer")
    fieldnames = ["# Time [s]","forward velocity [m/s]","angular velocity[rad/s]"]
    with open(csv_filename, mode='w') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for i in filtered_data:
            writer.writerow({fieldnames[0]: i[0], fieldnames[1]: i[1], fieldnames[2]: i[2]})
    

if __name__ == "__main__":
    # main()
    clean_odometry()