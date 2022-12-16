---
layout: page
title: Conclusion
permalink: /conclusion/
---

# Conclusion
1. Both the robot and landmark positions update according to new data. Also, only landmarks within the robot's view appear to update, which matches our expected behavior. <br>
2. The estimated path is within a reasonable range of the actual path (<1m). <br>
3. The algorithm appears to be lagging when the robot is expected to turn. We believe this is partially due to a low IMU sampling rate of 1 Hz. <br>
4. If the robot's position deviates too far from the actual path, the landmark estimations are negatively impacted. Once this occurs, since the correction step depends on an accurate map, the robot can never fully recover. <br>


# Future Improvements
1. We are currently estimating the sensor covariance values, and observed that changing the values had significant effects on the results. An improvement would be to characterize the sensor more accurately.<br>
2. Test out different data cleaning methods to preserve more data. <br>
3. Run the algorithm on a simpler path such as a square to identify where we see the most error.<br>
4. Validate the performance of FastSLAM 1.0 by comparing it to at least one other algorithm, such as EKF SLAM or FastSLAM 2.0.<br>
5. Find a way to incorporate yaw more directly into the sensor update step. 