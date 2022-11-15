import csv

def load_data(filename):
    """Load data from the csv log

    Parameters:
    filename (str)  -- the name of the csv log

    Returns:
    data (dict)     -- the logged data with data categories as keys
                       and values list of floats
    """
    f = open(filename + ".csv")

    file_reader = csv.reader(f, delimiter=',')

    # Load data into dictionary with headers as keys
    data = {}
    # header = ["# Time [s]", "forward velocity [m/s]", "angular velocity[rad/s]"]
    # for h in header:
    #     data[h] = []

    row_num = 0
    f_log = open("bad_data_log.txt", "w")
    for row in file_reader:
        for h, element in zip(header, row):
            # If got a bad value just use the previous value
            try:
                
                data[h].append(float(element))
            except ValueError:
                data[h].append(data[h][-1])
                f_log.write(str(row_num) + "\n")

        row_num += 1
    f.close()
    f_log.close()

    return data

def clean_odometry():
    #filepath = "c:/Users/echen/cs/e205lab3/"
    filename = "Robot1_Odometry"
    data = load_data(filename)
    timestamp = data["# Time [s]"]
    forward_v = data["forward velocity [m/s]"]
    angular_v = data["angular velocity[rad/s]"]

    # test load
    print(data)

    # establish csv file for filtered data [TODO, may not be needed]
    header = ["# Time [s]", "forward velocity [m/s]", "angular velocity[rad/s]"]

    # loop through time stamps
    for i in range(len(timestamp)):
        if (i == 0):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            num_matching_time = 1
            while (timestamp[i] != timestamp[i + num_matching_time]):
                # compute average
                num_matching_time += 1
            pass
        elif(timestamp[i] != timestamp[i-1]):
            # look at upcoming timestamp indices
            # stop when timestamps do not match, get averages, define as 1x3
            pass
        else:
            # go to next timestamp, indexed point should already have been accounted for
            pass
