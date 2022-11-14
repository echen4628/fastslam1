import numpy as np
from utils import *
import pdb

def resample_step(x_bar_t):
    """Ressampling step for the PF

    Parameters:
    x_bar_t       (np.array)    -- the predicted state estimate of time t
    """

    """STUDENT CODE START"""
    x_bar_t_weights = x_bar_t[:,-1]
    rng = np.random.default_rng()
    x_est_t = rng.choice(x_bar_t, x_bar_t.shape[0], p=x_bar_t_weights)
    # x_est_t = x_bar_t
    """STUDENT CODE END"""

    return x_est_t

if __name__ == "__main__":
    x_t_prev = np.random.rand(10, 6)
    x_t_prev[:, 5] = [0.6, 0.25, 0.15,0,0,0,0,0,0,0]
    u_t = np.array([0.5, 0.2])
    z_t = np.array([0.5, 0.5, 0.2])
    x_bar_t = x_t_prev
    x_est_t = resample_step(x_bar_t)
    print(x_est_t)
