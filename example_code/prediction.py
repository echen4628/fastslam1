import numpy as np
from utils import *
import math
import scipy.stats as stats
import pdb


def propogate_state(x_t_prev, u_t_noiseless):
    """Propagate/predict the state based on chosen motion model

    Parameters:
    x_t_prev (np.array)  -- the previous state estimate
    u_t (np.array)       -- the current control input

    Returns:
    x_bar_t (np.array)   -- the predicted state
    """
    """STUDENT CODE START"""
    # x_bar_t = x_t_prev   WAS INCLUDED IN CODE STUB?!
    x_bar_t = np.zeros((x_t_prev.shape[0], 6))
    delta_t = 0.1  # based on sampling rate of 10Hz
    # x_bar_t = np.array([])
    x_bar_t[0:, 0] = x_t_prev[0:, 0] + x_t_prev[0:, 2]*delta_t
    x_bar_t[0:, 1] = x_t_prev[0:, 1] + x_t_prev[0:, 3]*delta_t
    x_bar_t[0:, 2] = x_t_prev[0:, 2] + u_t_noiseless[0] * \
        np.cos(x_t_prev[0:, 4])*delta_t  # needs to be in radian
    x_bar_t[:, 3] = x_t_prev[:, 3] + u_t_noiseless[0]*  \
        np.sin(x_t_prev[:, 4])*delta_t
    x_bar_t[:, 4] = x_t_prev[:, 4] + u_t_noiseless[1]*delta_t
    x_bar_t[0:, 5] = x_t_prev[0:, 5]
    
    # treating input as just one particle
    # x_bar_x_t = x_t_prev[0] + x_t_prev[2]*delta_t
    # x_bar_y_t = x_t_prev[1] + x_t_prev[3]*delta_t
    # v_bar_x_t = x_t_prev[2] + u_t_noiseless[0] * \
    #     math.cos(x_t_prev[4])*delta_t  # needs to be in radian
    # v_bar_y_t = x_t_prev[3] + u_t_noiseless[0]*math.sin(x_t_prev[4])*delta_t
    # theta_bar_t = x_t_prev[4] + u_t_noiseless[1]*delta_t
    # w_t = x_t_prev[5]
    # x_bar_t = np.array(
    #     [x_bar_x_t, x_bar_y_t, v_bar_x_t, v_bar_y_t, theta_bar_t, w_t])
    """STUDENT CODE END"""
    return x_bar_t

def correction_step(x_bar_t, z_t):
    """Compute the correction step
    
    Parameters:
    x_bar_t (np.array)         -- all particles
    """
    new_weights = []
    sigma = np.array([[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.2]])
    for i in range(x_bar_t.shape[0]):
        x_i = x_bar_t[i]
        mu = np.array([x_i[0], x_i[1], x_i[4]])
        p_z_given_x_i = stats.multivariate_normal.pdf(z_t, mu, sigma)
        new_weights.append(p_z_given_x_i)
    new_weights = np.array(new_weights)

    new_weights_sum = np.sum(new_weights)
    # print("HI: " , new_weights_sum)

    new_weights = new_weights/new_weights_sum

    return new_weights

def prediction_and_correction_step(x_t_prev, u_t, z_t):
    # print("Hi from prediciotn")
    """Compute the prediction and correction (re-weighting) for the PF

    Parameters:
    x_t_prev (np.array)         -- the previous state estimate
    u_t (np.array)              -- the control input
    z_t (np.array)              -- the current measurement

    Returns:
    x_bar_t (np.array)          -- the predicted state estimate of time t
    """

    """STUDENT CODE START"""
    # pdb.set_trace()
    num_particles = x_t_prev.shape[0]

    # Prediction step
    x_bar_t = propogate_state(x_t_prev, u_t)

    # create noise
    # x_t_prev_var = np.var(x_t_prev, axis=0)
    x_t_prev_var = np.array([1,1,1,1,1])
    x_t_prev_var = x_t_prev_var
    x_t_prev_noise_x = np.random.normal(0, x_t_prev_var[0], num_particles)
    x_t_prev_noise_y = np.random.normal(0, x_t_prev_var[1], num_particles)
    x_t_prev_noise_v_x = np.random.normal(0, x_t_prev_var[2], num_particles)
    x_t_prev_noise_v_y = np.random.normal(0, x_t_prev_var[3], num_particles)
    x_t_prev_noise_theta = np.random.normal(0, x_t_prev_var[4], num_particles)
    x_t_noise = np.zeros((num_particles, 6))
    x_t_noise[:,0] = x_t_prev_noise_x
    x_t_noise[:,1] = x_t_prev_noise_y
    x_t_noise[:,2] = x_t_prev_noise_v_x
    x_t_noise[:,3] = x_t_prev_noise_v_y
    x_t_noise[:,4] = x_t_prev_noise_theta
    
    # add noise
    x_bar_t = x_bar_t + x_t_noise

    # correction step
    new_weights = correction_step(x_bar_t, z_t)
    x_bar_t[:,5] = new_weights.T
    
    """STUDENT CODE END"""

    return x_bar_t

if __name__ == "__main__":
    x_t_prev = np.zeros((10, 6))
    x_t_prev[:, 5] = 1/10
    u_t = np.array([0.5, 0.2])
    z_t = np.array([0.5, 0.5, 0.2])
    prediction_and_correction_step(x_t_prev, u_t, z_t)
