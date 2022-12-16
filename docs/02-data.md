---
layout: page
title: Data
permalink: /data/
---

# Data Source
We are using the Multi-robot Cooperative Localization and Mapping [citation here] dataset collected by Leung et. al at the University of Toronto Institute for Aerospace Studies. The dataset consists of nine sub-datasets, which differ in experimental runtime and setup. Data was produced using 5 moving robots and 15 static landmarks, which were each assigned a unique barcode. 
<br>
<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Robot_and_Landmarks.png" alt="Robots and Landmarks" width="100" />
</div>
##### Fig. 1, Caption Here, Will upload photo at end

<br>

During each experiment, the robots drive to random waypoints in the workspace, logging their odometry at a frequency of 1 Hz. Whenever another robot or landmark is in its field of view, a robot records a range and bearing measurement. The robots are also capable of logging multiple measurements at the same time in the event that more than one object is in view. In addition, groundtruth data for all robot and landmark positions is simultaneously recorded using an external 10-camera Vicon motion capture system.

<br>
 
# Data Description
In each sub-dataset, there are seventeen text files. The following table summarizes the five types of files and their respective fields:
<br>
<style type="text/css">
.tg  {border-collapse:collapse;border-spacing:0;}
.tg td{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  overflow:hidden;padding:10px 5px;word-break:normal;}
.tg th{border-color:black;border-style:solid;border-width:1px;font-family:Arial, sans-serif;font-size:14px;
  font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg .tg-e5wy{background-color:#8584b6;color:#000000;font-weight:bold;text-align:left;vertical-align:top}
.tg .tg-u4ra{background-color:#ffffff;border-color:#efefef;color:#000000;text-align:left;vertical-align:top}
</style>
<table class="tg">
<thead>
  <tr>
    <th class="tg-e5wy">Text File</th>
    <th class="tg-e5wy">Field #1</th>
    <th class="tg-e5wy">Field #2</th>
    <th class="tg-e5wy">Field #3</th>
    <th class="tg-e5wy">Field #4</th>
    <th class="tg-e5wy">Field #5</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-u4ra">Barcode Identification</td>
    <td class="tg-u4ra">subject #</td>
    <td class="tg-u4ra">barcode #</td>
    <td class="tg-u4ra"></td>
    <td class="tg-u4ra"></td>
    <td class="tg-u4ra"></td>
  </tr>
  <tr>
    <td class="tg-u4ra">Landmark Groundtruth</td>
    <td class="tg-u4ra">subject #</td>
    <td class="tg-u4ra">x [m]</td>
    <td class="tg-u4ra">y [m]</td>
    <td class="tg-u4ra">x std-dev [m]</td>
    <td class="tg-u4ra">y std-dev [m]</td>
  </tr>
  <tr>
    <td class="tg-u4ra">Robot Groundtruth (one for each robot)</td>
    <td class="tg-u4ra">time [s]</td>
    <td class="tg-u4ra">x [m]</td>
    <td class="tg-u4ra">y [m]</td>
    <td class="tg-u4ra">θ [rad]</td>
    <td class="tg-u4ra"></td>
  </tr>
  <tr>
    <td class="tg-u4ra">Robot Odometry (one for each robot)</td>
    <td class="tg-u4ra">time [s]</td>
    <td class="tg-u4ra">v [m/s]</td>
    <td class="tg-u4ra">ω [rad/s]</td>
    <td class="tg-u4ra"></td>
    <td class="tg-u4ra"></td>
  </tr>
  <tr>
    <td class="tg-u4ra">Measurement Files (one for each robot)</td>
    <td class="tg-u4ra">time [s]</td>
    <td class="tg-u4ra">barcode #</td>
    <td class="tg-u4ra">r [m]</td>
    <td class="tg-u4ra">φ [rad]</td>
    <td class="tg-u4ra"></td>
  </tr>
</tbody>
</table>
<br>

### <u>Barcode Identification</u>
It is important to note that subject numbers and barcode numbers are different. Robots are assigned a subject number from 1-5 and landmarks are assigned a subject number from 6-20. The barcode numbers are more random, as each barcode encodes two digits that and ranges from 5 to 90. The two digits sum sum to five for a robot's barcode, and seven or nine for a landmark's barcode. This was done in order to mitigate the risk of misreading a barcode. By using the barcode identification file's information, we are able to convert the barcode numbers recorded in the measurement files to subject numbers. 

### <u>Landmark Groundtruth</u>
The landmark groundtruth file contains the average Vicon x and y position measurements of all landmarks. It also contains the standard deviations for each measurement. This data is useful for measuring our algorithm's mapping performance.

### <u>Robot Groundtruth</u>
There are five robot groundtruth files, with each filename corresponding to a robot's subject number. Each robot's groundtruth file contains timestamped robot position information (x,y,θ). This data is useful for measuring our algorithm's localization performance.

### <u>Robot Odometry</u>
There are five robot odometry files, with each filename corresponding to a robot's subject number. Each robot's odometry file contains timestamped forward and angular velocity commands(v,ω). This data is used in our algorithm to propagate our particle states.

### <u>Measurement Files</u>
here are five measurement files, with each filename corresponding to a robot's subject number. As mentioned earlier, the measurement files contain timestamped range and bearing measurements (r,φ) to specific subjects. The range indicates a robot's recorded distance from a barcoded subject, and the bearing represents to angle between the robot's forward view and the subject. It is important to note that the bearing measurements are recorded with respect to the robot body's frame, which is depicted in Figure 2. The measurement data is used in our algorithm's reweight and update steps, which are explained in more detail in the <b>Methods</b> section.

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Bearing_Frames.png" alt="Bearing Frames" width="100" />
</div>
#### Fig. 2, Caption here

<br>

# Data Cleaning
Before we started our algorithm, we decided to clean the odometry, measurement, and groundtruth data. This was done for two reasons: 

(1) Many timestamps had multiple sets of data for the same subject, resulting in significantly more lines of data than timestamps. For example, for 1491 timestamps, Robot 1's original odometry file contained 97890 entries.

(2) For our algorithm, we were interested in a single robot's ability to map the static landmarks and its own position. As a result, we chose not to use the measurement data that corresponded to other robots.

A general overview of how we cleaned the data is described below. The data was saved as new files to preserve the original data and verify the cleaned data's accuracy. 

### <u>Cleaning the Odometry and Groundtruth Files</u>
To clean the data, we looped through each file and averaged any data that shared the same timestamp. For example, if a single timestamp had three sets of data in the odometry file, we would return a single line containing the timestamp, average forward velocity, and average angular velocity.

### <u>Cleaning the Measurement Files</u>
We looped through each file and averaged any data that shared the same timestamp and barcode number. In addition, we removed any data that had a barcode number corresponding to a robot subject. Finally, using the barcode identification file, we converted the barcode numbers to their respective subject numbers for ease of future testing.