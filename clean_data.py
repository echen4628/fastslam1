import csv
import pandas as pd

def clean_odometry():
    filepath = "./data/"
    filename = "Robot2_Odometry"

    data = pd.read_csv(filepath + filename + '.csv')

    timestamp = data["Time"]
    forward_v = data["Forward-velocity"]
    angular_v = data["Angular-velocity"]

    # loop through time stamps
    filtered_data = []
    for i in range(len(timestamp)):
        matching_timestamp_list = data[timestamp == timestamp[i]]
        if (i == 0):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            timestamp_avg_fv = matching_timestamp_list["Forward-velocity"].mean()
            timestamp_avg_av = matching_timestamp_list["Angular-velocity"].mean()
            timestamp_avg_data = [timestamp[i], timestamp_avg_fv, timestamp_avg_av]
            filtered_data.append(timestamp_avg_data)
        elif(timestamp[i] != timestamp[i-1]):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            timestamp_avg_fv = matching_timestamp_list["Forward-velocity"].mean()
            timestamp_avg_av = matching_timestamp_list["Angular-velocity"].mean()
            timestamp_avg_data = [timestamp[i], timestamp_avg_fv, timestamp_avg_av]
            filtered_data.append(timestamp_avg_data)
        else:
            # go to next timestamp, indexed point should already have been accounted for
            pass  
        
    csv_filename = 'Cleaned_Robot2_Odometry.csv'
    print("Beginning transfer")
    fieldnames = ["Time","Forward-velocity","Angular-velocity"]
    with open(csv_filename, mode='w') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for i in filtered_data:
            writer.writerow({fieldnames[0]: i[0], fieldnames[1]: i[1], fieldnames[2]: i[2]})


def clean_measurement():
    # robots are subjects 1-5, landmarks are subjects 16-20. Robot 1 is subject 1 (barcode 5)   
    # barcodes to ignore: 5, 14, 41, 32, 23
    filepath = "./data/"
    filename = "Robot2_Measurement"

    data = pd.read_csv(filepath + filename + '.csv')
    # sort by time and barcode to simplify iteration process
    sorted_data = data.sort_values(by=["Time", "Subject"])
    sorted_data = sorted_data.reset_index()

    timestamp = sorted_data["Time"]
    subject = sorted_data["Subject"]

    robot_barcodes = [5, 14, 41, 32, 23]
    landmark_barcodes = [72, 27, 54, 70, 36, 18, 25, 9, 81, 16, 90, 61, 45, 7, 63]

    filtered_data = []
    for i in range(len(timestamp)):
        matching_timestamp_list = sorted_data[timestamp == timestamp[i]]
        sub_subject_list = matching_timestamp_list["Subject"]

        if (i == 0):
            landmark_index = landmark_barcodes.index(sub_subject_list[i])
            landmark_num = landmark_index + 6
            sub_barcode_data = matching_timestamp_list[subject == sub_subject_list[i]]
            sub_timestamp_avg_range = sub_barcode_data["Range"].mean()
            sub_timestamp_avg_bearing = sub_barcode_data["Bearing"].mean()
            sub_timestamp_avg_data = [timestamp[i], landmark_num,
                                      sub_timestamp_avg_range, sub_timestamp_avg_bearing]
            filtered_data.append(sub_timestamp_avg_data)
            pass
        elif timestamp[i] != timestamp[i - 1]:
            for j in range(len(sub_subject_list)):
                if (j == 0) and (sub_subject_list[i+j] not in robot_barcodes):
                    sub_barcode_data = matching_timestamp_list[subject == sub_subject_list[i+j]]
                    landmark_index = landmark_barcodes.index(sub_subject_list[i+j])
                    landmark_num = landmark_index + 6
                    # print(sub_barcode_data)
                    sub_timestamp_avg_range = sub_barcode_data["Range"].mean()
                    sub_timestamp_avg_bearing = sub_barcode_data["Bearing"].mean()
                    sub_timestamp_avg_data = [timestamp[i], landmark_num,
                                              sub_timestamp_avg_range, sub_timestamp_avg_bearing]
                    filtered_data.append(sub_timestamp_avg_data)
                    # print(filtered_data)
                elif (j > 0) and\
                        (sub_subject_list[i+j] != sub_subject_list[i+j-1]) and \
                        (sub_subject_list[i+j] not in robot_barcodes):
                    sub_barcode_data = matching_timestamp_list[subject == sub_subject_list[i+j]]
                    landmark_index = landmark_barcodes.index(sub_subject_list[i + j])
                    landmark_num = landmark_index + 6
                    sub_timestamp_avg_range = sub_barcode_data["Range"].mean()
                    sub_timestamp_avg_bearing = sub_barcode_data["Bearing"].mean()
                    sub_timestamp_avg_data = [timestamp[i], landmark_num,
                                              sub_timestamp_avg_range, sub_timestamp_avg_bearing]
                    filtered_data.append(sub_timestamp_avg_data)
                else:
                    pass
        else:
            # go to next timestamp, indexed point should already have been accounted for
            pass

    csv_filename = 'Cleaned_Robot2_Measurement.csv'
    fieldnames = ["Time", "Subject", "Range", "Bearing"]
    with open(csv_filename, mode='w') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for i in filtered_data:
            writer.writerow({fieldnames[0]: i[0], fieldnames[1]: i[1],
                             fieldnames[2]: i[2], fieldnames[3]: i[3]})

def clean_robot_groundtruth():
    # filepath = "/Users/Erina/e205labs/fastslam1/fastslam1/data/"
    # filename = "Robot2_Groundtruth"

    filepath = "./data/Robot2_Groundtruth.csv"

    #data = pd.read_csv(filepath + filename + '.csv')
    data = pd.read_csv(filepath)

    timestamp = data["Time"]

    # loop through time stamps
    filtered_data = []
    for i in range(len(timestamp)):
        matching_timestamp_list = data[timestamp == timestamp[i]]
        if (i == 0):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            timestamp_avg_x = matching_timestamp_list["X"].mean()
            timestamp_avg_y = matching_timestamp_list["Y"].mean()
            timestamp_avg_or = matching_timestamp_list["Orientation"].mean()
            timestamp_avg_data = [timestamp[i], timestamp_avg_x, timestamp_avg_y, timestamp_avg_or]
            filtered_data.append(timestamp_avg_data)
        elif(timestamp[i] != timestamp[i-1]):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            timestamp_avg_x = matching_timestamp_list["X"].mean()
            timestamp_avg_y = matching_timestamp_list["Y"].mean()
            timestamp_avg_or = matching_timestamp_list["Orientation"].mean()
            timestamp_avg_data = [timestamp[i], timestamp_avg_x, timestamp_avg_y, timestamp_avg_or]
            filtered_data.append(timestamp_avg_data)
        else:
            # go to next timestamp, indexed point should already have been accounted for
            pass  
        
    csv_filename = 'Cleaned_Robot2_Groundtruth.csv'
    print("Beginning transfer")
    fieldnames = ["Time","X","Y","Orientation"]
    with open(csv_filename, mode='w') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for i in filtered_data:
            writer.writerow({fieldnames[0]: i[0], fieldnames[1]: i[1], fieldnames[2]: i[2], fieldnames[3]: i[3]})

if __name__ == "__main__":
    clean_odometry()
    clean_measurement()
    clean_robot_groundtruth()