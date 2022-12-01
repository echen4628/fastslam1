import numpy as np
import math

class Particle():
    def __init__(self):
        self.state = np.zeros(3)
        self.landmark = np.zeros((15,2))
        #[[1,2],
        #  [2,3]]
        self.landmark_cov = np.zeros((15, 2, 2))
        #[ [[1,2],
        #    [3,4]],
        #   [[]]]

        self.set_x(3.5718479294117658)
        self.set_y(-3.3314256499999995)
        self.set_yaw(2.3551147058823525)

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
        # self.next_particles = [Particle() for i in range(num_particles)]
        # self.next_weights = np.ones(num_particles)*(1/num_particles)
        # self.weighted_particle = Particle()
    
    def __repr__(self):
        return f"num_particles: {self.num_particles}\n \
                 particles: {self.particles[:3]}\n \
                 weights: {self.weights[:3]}\n \
                 weighted_particle: {self.weighted_particle}"

    # def prediction(self):
    #     # propagate and add noise using the self.particles
    #     #  and store in self.next_particles

    #     # self.next_particles will be updated
    #     pass
    
    def wrap_to_pi(self, angle):
        """Wrap angle data in radians to [-pi, pi]

        Parameters:
        angle (float)   -- unwrapped angle

        Returns:
        angle (float)   -- wrapped angle
        """
        while angle >= math.pi:
            angle -= 2*math.pi

        while angle <= -math.pi:
            angle += 2*math.pi
        return angle
    
    def propagate_all_states(self, u_t_noiseless, dt):
        for i in range(len(self.particles)):
            self.particles[i].state = self.propogate_single_particle_state(self.particles[i].state, u_t_noiseless, dt)
            # x_t_var = np.array([1,1,1])
            # x_t_noise_x = np.random.normal(0, x_t_var[0])
            # x_t_noise_y = np.random.normal(0, x_t_var[1])
            # x_t_noise_z = np.random.normal(0, x_t_var[2])
            # self.particles[i].state += np.array([x_t_noise_x, x_t_noise_y, x_t_noise_z])

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
        
    def reweight_and_update(self, measurements):
        """
        measurements: [[tag, r, theta], [tag, r, theta]]
        """
        for idx, p in enumerate(self.particles):
            for z in measurements:
                subject_tag = int(z["Subject"])
                sensor = [z["Range"], z["Bearing"]]
                robot_states = [p.get_x(), p.get_y(), p.get_yaw()]
                # go from z[tag] to self.particle.landmark[tag-6] (1x2)
                current_landmark =  p.landmark[subject_tag-6].reshape((2))
                    
                # get covariance index self.landmark_cov[tag-6] (1x2x2)
                current_landmark_cov =  p.landmark_cov[subject_tag-6].reshape((2, 2))
                
                # change measurement to become a measurement of where the robot is
                    # things we get from measurement: measurement_theta, measurement_range
                    # things we get from our particle: landmark x, y, yaw
                measured_landmark_x, measured_landmark_y = self.sensor_to_measured_robot_state(sensor, robot_states)
                if np.all(current_landmark == np.array([0,0])): # never seen this landmark before
                    # intialize landmark
                    p.landmark[subject_tag-6][0] = measured_landmark_x
                    p.landmark[subject_tag-6][1] = measured_landmark_y
                else:
                    # find difference between estimation/pred and measurement z - z hat
                    z_diff = np.zeros((2,1))
                    H_t = np.identity(2)
                    z_diff[0,0] = current_landmark[0] - measured_landmark_x
                    z_diff[1,0] = current_landmark[1] - measured_landmark_y
                    # find Modified_ measurement covariance H*Cov_landmark*H^T + Q <-identity.
                    sensor_cov = np.identity(2)
                    modified_measurement_cov = self.calc_measurement_cov(current_landmark_cov, sensor_cov)
                    # find w*
                    self.weights[idx] =  self.weights[idx]* self.calc_weight(modified_measurement_cov, z_diff)
                    # call the update function to update landmark of particle p
                    updated_landmark, updated_landmark_cov = self.update(modified_measurement_cov, current_landmark_cov, H_t, current_landmark, z_diff)
                    p.landmark[subject_tag-6] = updated_landmark
                    p.landmark_cov[subject_tag-6] = updated_landmark_cov

    def update(self, modified_measurment_cov, landmark_cov, H_t, current_landmark, z_diff):
        K_t = landmark_cov@H_t.T@np.linalg.inv(modified_measurment_cov)
        updated_landmark = current_landmark + (K_t@z_diff).reshape(2) # 2
        updated_landmark_cov = (np.identity(2) - K_t@H_t)@landmark_cov # 2x2
        return updated_landmark, updated_landmark_cov
    
    def calc_weight(self, modified_measurement_cov, z_diff):
        inverted_cov_term = np.linalg.inv(modified_measurement_cov)
        exp_term = -0.5*z_diff.T@inverted_cov_term@z_diff
        weight_coefficient = np.linalg.det(2*np.pi*modified_measurement_cov)**(-0.5)
        return weight_coefficient*np.e**(exp_term)

    def sensor_to_measured_robot_state(self, sensor, robot_states):
        """
        sensor = [r, theta]
        robot_states = [x,y,yaw]
        """
        robot_x = robot_states[0]
        robot_y = robot_states[1]
        robot_yaw = robot_states[2]
        r = sensor[0]
        theta = sensor[1]

        measured_landmark_x = robot_x + r*np.cos(theta + robot_yaw)
        measured_landmark_y = robot_y + r*np.sin(theta + robot_yaw)

        return measured_landmark_x, measured_landmark_y
    
    def calc_measurement_cov(self, landmark_cov, Q):
        H = np.identity(2)
        return H @ landmark_cov @ H.T + Q

    def reweight(self, z_t, sigma_z_t):
        # reweight this self.next_particles
        Q = np.identity(2)
        
        # need to loop through the z_t

        # calc the weight contribution from each measurement

        # multiple with the original weight

        # call correction step on the measurements while at it.
        
        pass

    def calc_single_particle_jacobian(self, particle_states, landmark_state, landmark_cov, Q):
        """
        particle_states 1D length 3
        landmark_state 1D length 2
        landmark_cov 2D length 2x2
        Q is 2x2
        """
        dx = landmark_state[0] - particle_states[0]
        dy = landmark_state[1] - particle_states[1]
        d2 = dx**2 + dy**2
        d = math.sqrt(dx)
        dyaw = self.wrap_to_pi(math.atan2(dy, dx) - particle_states[2])

        zp = np.array(d, dyaw)
        H_t = np.array([[dx/d, dy/d], [-dy/d2, dx/d2]])
        
        return zp, H_t

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