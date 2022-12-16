---
layout: page
title: Method
permalink: /method/
---
Simultaneous Localization and Mapping (SLAM) algorithms fuse various sensor readings to track an agent’s location and map its surroundings. They are perfect for our goal of localizing a robot in an unknown environment of the MR.CLAM dataset. While there are many SLAM algorithms, we choose to use FastSLAM 1.0 because it fits our problem well and combines familiar algorithms in creative ways. Note that although FastSLAM 1.0 is just one algorithm in a family of FastSLAM algorithms, we will use the term FastSLAM 1.0 and FastSLAM interchangeably. 

Obstacles in the MR.CLAM dataset are stationary and tagged by a barcode. Therefore, FastSLAM 1.0, which is a landmark based SLAM, is perfect for the task. We also considered another landmark based SLAM - the Extended Kalman Filter SLAM (EKF SLAM). However, FastSLAM 1.0 was chosen due to its improve efficiency. The creater of FastSLAM Michael Montemerlo designed FastSLAM as a response to the EKF SLAM's time consuming sensor update step. The EKF SLAM inverts several large covariance matrices, causing its time complexity to be $O(K^2)$, where $K$ represents the number of landmarks. In contrast, the time complexity of FastSLAM 1.0 can be as low as $O(M log K)$, where $M$ and $K$ represent the number of particles and landmarks respectively. Our implementation has a time complexity of $O(M*K)$. 

We also chose FastSLAM 1.0 due to its technical ingenuity. FastSLAM draws inpirations from the Particle Filter and the Extended Kalman Filter. Implementing FastSLAM allows us to review and utilize a large portion of E205's material. Since we are more familiar with the components of FastSLAM than many other SLAM algorithms, we have more freedom to make design decisions. 

Our implementation of FastSLAM can be found [here](https://github.com/echen4628/fastslam1/blob/main/FASTSlam.py) in the $\texttt{FASTSlam.py}$ file. Like the original algorithm outlined in "FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem" by Michael Montemerlo et al., our implementation has two main components - the $\texttt{Particle}$ and $\texttt{FastSLAM}$ classes. 

The $\texttt{Particle}$ class encodes a single FastSLAM particle. In FastSLAM, each particle contains an extimated state of the robot in the global frame. The state vector $X$ of a robot in the MR.CLAM dataset is [$x$, $y$, $\theta$], representing the x,y coordinate of the robot and its angle. Unlike a particle in a Particle Filter, FastSLAM particles also record estimated landmark positions and covariances. Each landmark state vector $L$ is represented by [$x$, $y$]. Each landmark covariance, $\texttt{landmark\_cov}$, is a typical 2X2 matrix representing the covariance in the corresponding landmark state. These covariance matrices will be used later in an EKF style update step. On initialization, the robot state is set to be the groundtruth starting pose and the landmark covariance to the identity. Landmark positions are initialized later in the sensor update stage of the FastSLAM algorithm. Unlike the typical FastSLAM particle, our particles do not carry a weight. We include the particle weight in the $\texttt{Fastslam}$ class for direct access and avoid an unnecessary loop to collect weights from each particle when needed. 

The $\texttt{Fastslam}$ class implements are the remaining parts of the FastSLAM algorithm. It keeps track of the number of particles, a list of particles, and the weights for each particle. It includes four main functions: 1. $\texttt{propagate\_all\_states}$, 2. $\texttt{reweight\_and\_update}$, 3. $\texttt{combine\_particles}$, and 4. $\texttt{resample}$.

## $\texttt{propagate\_all\_states}$
The $\texttt{propagate\_all\_states}$ function implements the prediction step of FastSLAM. Its arguments are the inputs to the robot, time difference between inputs, and a boolean indicating whether noise will be added to the system before progating or after. As mentioned in the Data section, the MR.CLAM dataset provides each robot’s forward velocity $v$ and angular velocity $\omega$. We update each particle’s robot state according to the following motion model.

$\begin{equation}
x_t = x_{t-1} + v_t * cos(\theta)*\Delta t 
\end{equation}$

$\begin{equation}
y_t = y_{t-1} + v_t * sin(\theta)*\Delta t 
\end{equation}$

$\begin{equation}
\theta_t = \theta_{t-1} + \omega_t *\Delta t 
\end{equation}$

If noise is to be added before motion model propagation, then gaussian noise centered around 0 is added to $v_t$ and $\omega_t$ first. Otherwise, gaussian noise centered around 0 is added to $x_t$, $y_t$, and $\theta_t$. 

## $\texttt{reweight\_and\_update}$
The next stage of the algorithm is reweight and update. The $\texttt{reweight\_and\_update}$ function takes in the measured range $r$ and bearing $\phi$ to all observed landmarks. The function loops through each particle and then loops through the measurements for each landmark to reweight the particle and update the associated landmark position. For particles that have never seen the landmark before now, we simply initialize the landmark according to the measurement without changing the particle weight. The correction step is also not needed for this landmark. Because the measurements are $r$ and $\phi$, we need translate them to landmark $L_x$ and $L_y$ positions using the following equations

$\begin{equation}
L_x = x_t + r*cos(\phi + \theta_t)
\end{equation}$

$\begin{equation}
L_y = y_t + r*sin(\phi + \theta_t)
\end{equation}$

If the landmark has been observed previously (in other words, the particle already carries an estimation of the landmark’s position), then the measurement will be used for reweighting the particle and to update the landmark position. We reweight each particle by the using the following equations,

$\begin{equation}
p_w = \Pi_{j} w_j
\end{equation}$

$\begin{equation}
w_j = 0 \text{ if landmark j is previously unobserved}
\end{equation}$

$\begin{equation}
w_j =  |2\pi Q|^{-\frac{1}{2}} e^{-\frac{1}{2} (z_j - \hat{z}_j)^{T}Q^{-1}(z_j - \hat{z}_j)}  \text{ otherwise}
\end{equation}$


where $j$ is the landmark index, $z_j$ is the measured landmark (which has already been converted from $r$ and $\phi$ to $L$), $\hat{z}_j$ is the estimated landmark (which is obtained from the particle).