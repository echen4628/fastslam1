import numpy as np
class Particle():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.yaw = 0 
        self.landmark = np.zeros((15,2))
        #[[1,2],
        #  [2,3]]
        self.landmark_cov = np.zeros((15, 2, 2))
        #[ [[1,2],
        #    [3,4]],
        #   [[]]]
        
class FASTSlam():
    def __init__(self, num_particles):
        # self.particles = lists of particles (not numpy list)
        # self.weights = 
        # self.next_particles = lists of particles
        # self.next_weights
        # weighted_particle = particle
        pass
    
    def prediction(self):
        # propagate and add noise using the self.particles
        #  and store in self.next_particles

        # self.next_particles will be updated
        pass
    
    def propagate_state(self):
        pass

#    def propogate_state(x_t_prev, u_t_noiseless):
#     """Propagate/predict the state based on chosen motion model

#     Parameters:
#     x_t_prev (np.array)  -- the previous state estimate
#     u_t (np.array)       -- the current control input

#     Returns:
#     x_bar_t (np.array)   -- the predicted state
#     """
#     """STUDENT CODE START"""
#     x_bar_t = np.zeros((x_t_prev.shape[0], 6))
#     delta_t = 0.1  # based on sampling rate of 10Hz
#     x_bar_t[0:, 0] = x_t_prev[0:, 0] + x_t_prev[0:, 2]*delta_t
#     x_bar_t[0:, 1] = x_t_prev[0:, 1] + x_t_prev[0:, 3]*delta_t
#     x_bar_t[0:, 2] = x_t_prev[0:, 2] + u_t_noiseless[0] * \
#         np.cos(x_t_prev[0:, 4])*delta_t  # needs to be in radian
#     x_bar_t[:, 3] = x_t_prev[:, 3] + u_t_noiseless[0]*  \
#         np.sin(x_t_prev[:, 4])*delta_t
#     x_bar_t[:, 4] = x_t_prev[:, 4] + u_t_noiseless[1]*delta_t
#     x_bar_t[0:, 5] = x_t_prev[0:, 5]
# """STUDENT CODE END"""
#      return x_bar_t


    
    def reweight(self, z_t, sigma_z_t):
        # reweight this self.next_particles
        pass

    def correction(self, z_t, sigma_z_t):
        # use self.next_particles and perform the correction step on all of them
        pass

    def combine_particles(self):
        # average the self.next_particles and store value
        pass

    def resample(self):
        # resample from self.next_particles and then reassign self.particles
        pass
