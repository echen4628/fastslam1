---
layout: page
title: Experiments and Results
permalink: /experiment_and_results/
---

The goal of our project is to localize a robot in an unknown environment. Along the way, we hope to characterize the performance of FastSLAM 1.0 and its robustness. We will quantify performance using the euclidean distance between the estimated and actual path and landmarks. We expect the errors to be higher at the start than at the end because localization should benefit from a more complete map of the environment. 

$\begin{equation}
\text{Euclidean Distance at t } = \sqrt{(x_t-\hat{x_t})^2}
\end{equation}$

, where $x_t$ is the groundtruth value and $\hat{x}_t$ is the estimated value at time $t$.

# Landmark 
For our experiment, we used robot 2 of the first dataset. Subsequent data reads are spaced 1 second apart as explained in the Data section. The FastSLAM filter uses 100 particles. We add gaussian noise centered around 0 with a standard deviation of 0.05 m independently to the forward velocity and angular velocity inputs. We set the sensor covariance to 0.02 m. The MR.CLAM dataset did not provide a sensor covariance, so this value was picked based on observation that measurement data appears to be relatively high quality. This is supported by observing the difference between the actual landmark and estimated landmark positions when the robot position is known.

Before estimating both robot and landmark positions simultaneously, we ran two experiments to verify that our implementation of FastSLAM 1.0 is bug free. The first experiment is attempting to localize the robot using groundtruth landmark data. In this experiment, at every timestep, the landmarks estimations of every particle will be set to their ground truth positions. This experiment tests whether our implementation of FastSLAM 1.0 can localize well in a known environment. Below is the result.


<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Localize_with_GT_Landmark.png" alt="Localize with Groundtruth Landmark" width="100%" />
</div>

###### <b>Fig. 1</b> Localization using Groundtruth Landmark Positions

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Localize_with_GT_Landmark_Tracking_Error.png" alt="Error of Localization with Groundtruth Landmark" width="100%" />
</div>

###### <b>Fig. 2</b> Euclidean distance between estimated and groundtruth paths after localization using groundtruth landmarks

We observe that with known landmarks, the estimated robot path followed the groundtruth robot path closely. The maximum error was around 0.53 m, but most errors are around or below 0.3 m. This suggests that our algorithm are capable of localization.

Our second experiment is attempting to map the robot’s environment using ground truth position data. In this experiment, at every timestep, the robot state of every particle is set to be the groundtruth location. This experiment tests the ability of our implementation to map the environment given the actual robot position. Below is the result.

<div>
  <img src="https://echen4628.github.io/fastslam1/assets/img/Mapping_with_GT_Positions.png" alt="Mapping with Groundtruth Positions" width="100%" />
</div>

###### <b>Fig. 3</b> Euclidean distance between estimated and groundtruth paths after localization using groundtruth landmarks

| Landmark | Euclidean Distance | Landmark | Distance | Landmark | Distance
|----| -----| -----| ----|---|---|
|6 | 0.07256|10| 0.01907|16| 0.03206|
|7 | 0.12077|11| 0.07136|17| 0.03464|
|8 | 0.06043|13| 0.01165|18| 0.01223|
|9 | 0.03167|14| 0.01049|19| 0.01158|
|10| 0.01907|15| 0.01102|20| 0.02700|


# Robustness
To test the robustness of the robot, we manually either set the estimated pose of the robot to be 5,5,0, or set the landmark estimations to be 2,2 at the 500th timestep. The entire dataset spans about 1500 timesteps, placing this disturbance at about one third of the way into the run.