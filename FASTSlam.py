import numpy as np
import math

class Particle():
    def __init__(self):
        self.state = np.zeros(3)
        # self.x = 0
        # self.y = 0
        # self.yaw = 0 
        self.landmark = np.zeros((15,2))
        #[[1,2],
        #  [2,3]]
        self.landmark_cov = np.zeros((15, 2, 2))
        #[ [[1,2],
        #    [3,4]],
        #   [[]]]
    def get_x(self):
        return self.state[0]
    def get_y(self):
        return self.state[1]
    def get_yaw(self):
        return self.state[2]
    def set_x(self, x):
        self.state[0] = x
    def set_y(self, y):
        self.state[1] = y
    def set_yaw(self, yaw):
        self.state[2] = yaw

    def __repr__(self):
        return f"X: {self.get_x()} \n \
                 Y: {self.get_y()} \n \
                 yaw: {self.get_yaw()} \n \
                 landmark: {self.landmark} \n \
                 landmark_cov: {self.landmark_cov} \n \
                ------------------------"
        
class Fastslam():
    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.particles = [Particle() for i in range(num_particles)]
        self.weights = np.ones(num_particles)*(1/num_particles)
        self.next_particles = [Particle() for i in range(num_particles)]
        self.next_weights = np.ones(num_particles)*(1/num_particles)
        self.weighted_particle = Particle()
    
    def __repr__(self):
        return f"num_particles: {self.num_particles}\n \
                 particles: {self.particles[:3]}\n \
                 weights: {self.weights[:3]}\n \
                 weighted_particle: {self.weighted_particle}"

    def prediction(self):
        # propagate and add noise using the self.particles
        #  and store in self.next_particles

        # self.next_particles will be updated
        pass
    
    def propagate_all_states(self, u_t_noiseless, dt):
        for i in range(len(self.particles)):
            self.particles[i].state = self.propogate_single_particle_state(self.particles[i].state, u_t_noiseless, dt)
            x_t_var = np.array([1,1,1])
            x_t_noise_x = np.random.normal(0, x_t_var[0])
            x_t_noise_y = np.random.normal(0, x_t_var[1])
            x_t_noise_z = np.random.normal(0, x_t_var[2])
            self.particles[i].state += np.array([x_t_noise_x, x_t_noise_y, x_t_noise_z])

    def propogate_single_particle_state(self, x_t_prev, u_t_noiseless, delta_t):
        """
        Propagate/predict the state based on chosen motion model

        Parameters:
        x_t_prev (np.array)  -- the previous state estimate
        u_t (np.array)       -- the current control input (odometry [x_dot, yaw_dot])

        Returns:
        x_bar_t (np.array)   -- the predicted state
        """
        x_bar_t = np.zeros((3))
        
        x_bar_t[0] = x_t_prev[0] + u_t_noiseless[0]*np.cos(x_t_prev[2])*delta_t
        x_bar_t[1] = x_t_prev[1] + u_t_noiseless[0]*np.sin(x_t_prev[2])*delta_t
        x_bar_t[2] = x_t_prev[2] + u_t_noiseless[1]*delta_t

        return x_bar_t
        

#    def propogate_state(x_t_prev, u_t_noiseless):
#     """Propagate/predict the state based on chosen motion model

#     Parameters:
#     x_t_prev (np.array)  -- the previous state estimate
#     u_t (np.array)       -- the current control input (odometry)

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
        # Q = np.identity(2)
        
        
        # need to loop through the z_t

        # calc the weight contribution from each measurement

        # multiple with the original weight

        # call correction step on the measurements while at it.
        
        pass

    def correction(self, z_t, sigma_z_t):
        # use self.next_particles and perform the correction step on all of them

        # to do the correction step, we can compare the measured range and theta
        # with the calcuated range and theta using the distance formula and atan2
        pass

    def combine_particles(self):
        # average the self.next_particles and store value
        combined_state = np.zeros(3)
        for i in range(len(self.weights)):
            combined_state[0] = self.particles[i].get_x()*self.weights[i]
            combined_state[1] = self.particles[i].get_y()*self.weights[i]
            combined_state[2] = self.particles[i].get_yaw()*self.weights[i]
        return combined_state

    def resample(self):
        # resample from self.next_particles and then reassign self.particles
        pass

if __name__ == "__main__":
    num_particles = 3
    test = Fastslam(num_particles)
    test.propagate_all_states(np.array([3, 1.5]) , 1)
    print(test.particles[0].state)
    test.propagate_all_states(np.array([3, 1.5]) , 1)
    print(test.particles[0].state)