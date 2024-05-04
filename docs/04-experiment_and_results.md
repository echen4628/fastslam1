---
layout: page
title: Experiments and Results
permalink: /experiment_and_results/
---

The goal of our project is to localize a robot in an unknown environment. Along the way, we also characterized the performance of FastSLAM 1.0 and its robustness. We quantified performance using the euclidean distance between the estimated and actual positions of the robot and landmarks. We expected the errors to be higher at the start than at the end because localization should benefit from a more complete map of the environment. 

$\begin{equation}
\text{Euclidean Distance at t } = \sqrt{(x_t-\hat{x_t})^2}
\end{equation}$

, where $x_t$ is the groundtruth value and $\hat{x}_t$ is the estimated value at time $t$.

# Landmark 
For our experiment, we used robot 2 of the first dataset. Subsequent data reads are spaced 1 second apart as explained in the Data section. The FastSLAM filter uses 100 particles. We add gaussian noise centered around 0 with a standard deviation of 0.05 m independently to the forward velocity and angular velocity inputs. We set the sensor covariance to 0.02 m. The MR.CLAM dataset did not provide a sensor covariance, so this value was picked based on the observation that measurement data appears to be relatively high quality. This is supported by observing the difference between the actual landmark and estimated landmark positions when the robot position is known.

Before estimating both robot and landmark positions simultaneously, we ran two experiments to verify that our implementation of FastSLAM 1.0 is bug-free. The first experiment is attempting to localize the robot using groundtruth landmark data. In this experiment, at every timestep, the landmarks estimations of every particle will be set to their ground truth positions. This experiment tests whether our implementation of FastSLAM 1.0 can localize well in a known environment. Below is the result.


<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Localize_with_GT_Landmark.png" alt="Localize with Groundtruth Landmark" width="100%" />
</div>

###### <b>Fig. 1</b> Localization using Groundtruth Landmark Positions

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Localize_with_GT_Landmark_Tracking_Error.png" alt="Error of Localization with Groundtruth Landmark" width="100%" />
</div>

###### <b>Fig. 2</b> Euclidean distance between estimated and groundtruth paths after localization using groundtruth landmarks

We observe that with known landmarks, the estimated robot path followed the groundtruth robot path closely. The maximum error was around 0.53 m, but most errors are around or below 0.3 m. This suggests that our algorithm is capable of localization.

Our second experiment attempts to map the robot's environment using ground truth position data. In this experiment, at every timestep, the robot state of every particle is set to be the groundtruth location. This experiment tests the ability of our implementation to map the environment given the actual robot position. Below is the result.

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Mapping_with_GT_Positions.png" alt="Mapping with Groundtruth Positions" width="100%" />
</div>

###### <b>Fig. 3</b> Euclidean distance between estimated and groundtruth paths after localization using groundtruth landmarks

| Landmark | Euclidean Distance (m)| Landmark | Distance| Landmark | Distance
|----| -----| -----| ----|---|---|
|6 | 0.07256|10| 0.01907|16| 0.03206|
|7 | 0.12077|11| 0.07136|17| 0.03464|
|8 | 0.06043|13| 0.01165|18| 0.01223|
|9 | 0.03167|14| 0.01049|19| 0.01158|
|10| 0.01907|15| 0.01102|20| 0.02700|

###### <b>Table. 1</b> Euclidean distance between estimated and groundtruth landmarks after mapping using groundtruth positions. Average error is 0.03667 m.

We observe that most landmark estimations were distinctly close to the groundtruth locations. The average error is 0.03667 m. The area of the testing site was close to about 60 m^2, so our error is relatively small. This promising result suggests our algorithm is also capable of accurate mapping when the localization is accurate. This also motivated us to use a low sensor covariance value.

Finally, we attempted to localize the robot and map its environment simultaneously. The result is as follows.
<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/SLAM.png" alt="SLAM" width="100%" />
</div>

###### <b>Fig. 4</b> Simultaneous Localization and Mapping

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/SLAM_Tracking_Error.png" alt="SLAM Tracking Error" width="100%" />
</div>

###### <b>Fig. 5</b> Simultaneous Localization and Mapping Tracking Error

| Landmark | Euclidean Distance (m)| Landmark | Distance| Landmark | Distance
|----| -----| -----| ----|---|---|
|6 | 0.39581|10| 0.63810|16| 0.48963|
|7 | 0.47139|11| 0.38798|17| 0.45038|
|8 | 0.47292|13| 0.39505|18| 0.62538|
|9 | 0.53963|14| 0.49539|19| 0.49196|
|10| 0.68457|15| 0.33944|20| 0.52820|

###### <b>Table. 2</b> Euclidean distance between estimated and groundtruth landmarks using FastSLAM 1.0. Average error is 0.4937 m.

We notice that the localization and mapping have higher errors than when we conducted our separate experiments. We also do not notice a decrease of localization errors as we predicted. In fact, the error was lowest at the start. Notably, the entire scene appears to be shifted to the right. The scene does appear to be mostly internally consistent. We believe this error is a result of a low IMU update frequency. Initially, the robot moves towards the right and then rotates counterclockwise. When the robot makes this turn, our estimated path lagged behind. The extra distance in the estimated robot path before the associated turn likely cause all subsequent landmark initializations and updates to be shifted to the right. This result suggests that FastSLAM 1.0's mapping capabilities is dependent on accurate localization. 

# Robustness
To test the robustness of the robot, we manually either set the estimated pose of the robot to be [5m, 5m, 0 rad] at the 500th timestep. The entire dataset spans about 1500 timesteps, placing this disturbance at about one third of the way into the run.

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Teleport_Robot.png" alt="SLAM" width="100%" />
</div>

###### <b>Fig. 6</b> Robot is teleported to $x=5$, $y=5$, and $\theta = 0$ at t = 500s.

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Teleport_Robot_Tracking_Error.png" alt="SLAM Tracking Error" width="100%" />
</div>

###### <b>Fig. 7</b> Tracking error in the teleporting robot experiment. The error increased dramatically about 500s after the start of the run.


| Landmark | Euclidean Distance (m)| Landmark | Distance| Landmark | Distance
|----| -----| -----| ----|---|---|
|6 | 6.20589|10| 9.76596|16| 6.69041|
|7 | 5.09170|11| 4.50479|17| 6.65484|
|8 | 7.29407|13| 5.90074|18| 9.02313|
|9 | 5.63583|14| 5.66122|19| 6.17277|
|10| 7.44631|15| 4.93662|20| 7.58258|

###### <b>Table. 3</b> Euclidean distance between estimated and groundtruth landmarks in the teleporting robot experiment. Average error is 6.5711 m.

After teleporting, the robot did not return back to the groundtruth path. The landmark positions, while relatively accurate at first, moved towards the the robot. The correction step was not suffice to fix this error and in fact moved the landmarks in the wrong direction. The robot's failure to correct itself reinforces the idea that localization and mapping depend heavily on each other's performance. Large errors in one aspect can lead to irrecoverable errors in the other side of SLAM. 