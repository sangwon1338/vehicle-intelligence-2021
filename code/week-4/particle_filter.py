import numpy as np
from helpers import distance

class ParticleFilter:
    def __init__(self, num_particles):
        self.initialized = False
        self.num_particles = num_particles

    # Set the number of particles.
    # Initialize all the particles to the initial position
    #   (based on esimates of x, y, theta and their uncertainties from GPS)
    #   and all weights to 1.0.
    # Add Gaussian noise to each particle.
    def initialize(self, x, y, theta, std_x, std_y, std_theta):
        self.particles = []
        for i in range(self.num_particles):
            self.particles.append({
                'x': np.random.normal(x, std_x),
                'y': np.random.normal(y, std_y),
                't': np.random.normal(theta, std_theta),
                'w': 1.0,
                'assoc': [],
            })
        self.initialized = True

    # Add measurements to each particle and add random Gaussian noise.
    def predict(self, dt, velocity, yawrate, std_x, std_y, std_theta):
        # Be careful not to divide by zero.
        v_yr = velocity / yawrate if yawrate else 0
        yr_dt = yawrate * dt
        for p in self.particles:
            # We have to take care of very small yaw rates;
            #   apply formula for constant yaw.
            if np.fabs(yawrate) < 0.0001:
                xf = p['x'] + velocity * dt * np.cos(p['t'])
                yf = p['y'] + velocity * dt * np.sin(p['t'])
                tf = p['t']
            # Nonzero yaw rate - apply integrated formula.
            else:
                xf = p['x'] + v_yr * (np.sin(p['t'] + yr_dt) - np.sin(p['t']))
                yf = p['y'] + v_yr * (np.cos(p['t']) - np.cos(p['t'] + yr_dt))
                tf = p['t'] + yr_dt
            p['x'] = np.random.normal(xf, std_x)
            p['y'] = np.random.normal(yf, std_y)
            p['t'] = np.random.normal(tf, std_theta)

    # Find the predicted measurement that is closest to each observed
    #   measurement and assign the observed measurement to this
    #   particular landmark.
    def associate(self, predicted, observations):
        associations = []
        # For each observation, find the nearest landmark and associate it.
        #   You might want to devise and implement a more efficient algorithm.
        for o in observations:
            min_dist = -1.0
            for p in predicted:
                dist = distance(o, p)
                if min_dist < 0.0 or dist < min_dist:
                    min_dist = dist
                    min_id = p['id']
                    min_x = p['x']
                    min_y = p['y']
            association = {
                'id': min_id,
                'x': min_x,
                'y': min_y,
            }
            associations.append(association)
        # Return a list of associated landmarks that corresponds to
        #   the list of (coordinates transformed) predictions.
        return associations

    # Update the weights of each particle using a multi-variate
    #   Gaussian distribution.
    def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        # TODO: For each particle, do the following:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates.
        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
        #    association algorithm.
        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
        # 5. Update the particle's weight by the calculated probability.
        w_s = 0.0
        transformed_observations = []
        for p in self.particles:
            w_t = 1.0
            landmarks =[]
            
            for i,val in map_landmarks.item():
                d = distance(p,val)
                if d < sensor_range:
                    landmarks.append({'i':i,'x':val['x'],'y':val['y']})
            if len(landmarks) == 0:
                continue
         
         linear_trans_x = [np.cos(p['t']), -np.sin(p['t']), p['x']]
         linear_trans_y = [np.sin(p['t']), np.cos(p['t']), p['y']]
         for i,val in enumerate(observations):
                cur_cordi = [val['x'], val['y'], 1]
                
                transformed_x = np.dot(cur_cordi, linear_trans_x)
                transformed_y = np.dot(cur_cordi, linear_trans_y)
                transformed_observations.append({'x': transformed_x, 'y': transformed_y})
          
          a = self.associate(landmarks, transformed_observations) 
          temp_id = []
          for i in range(len(a)):
                temp_id.append(a[i]['id'])
                
                      p['assoc'] = temp_id
                         
         
          for i, val in enumerate(transformed_observations):
              
                transformed_obs = {'x': transformed_x, 'y': transformed_y}
                
                single_landmark_k = []
                single_landmark = {}
 
                for i, val in enumerate(landmarks):
                    d = distance(transformed_obs, val)
                    single_landmark_k.append({'d': d, 'x': val['x'], 'y': val['y']})
                    

                distance_min = single_landmark_k[0]['dist']
                single_landmark = single_landmark_k[0]

                for j in range(1, len(single_landmark_k)):
                    if distance_min >= single_landmark_k[j]['dist']:
                        single_landmark = single_landmark_k[j]


                normalizer = 1./ 2. * np.pi * std_landmark_x * std_landmark_y

                exponent = pow((transformed_obs['x'] - single_landmark['x']), 2) / pow(std_landmark_x, 2) + pow((transformed_obs['y'] - single_landmark['y']), 2) /pow(std_landmark_y, 2)

                obs_w = normalizer * math.exp((-0.5 * exponent))  
                obs_w +=  1e-25

                wt *= obs_w

            p['w'] = wt
                
        pass

    # Resample particles with replacement with probability proportional to
    #   their weights.
    def resample(self):
        import copy 
        from scipy import stats

        w = [p['w'] for p in self.particles]

        
        pk = weights/np.sum(w)



        xk = np.arange(len(self.particles))

       

        cus = stats.rv_discrete(name="paticles", values=(xk, pk))

        resampled_idx = cus.rvs(size=self.num_particles)

       
        r_p = []

        for i in resampled_idx:
             resampled_particles.append(copy.deepcopy(self.particles[i]))

        self.particles = r_p
          
   

        pass

    # Choose the particle with the highest weight (probability)
    def get_best_particle(self):
        highest_weight = -1.0
        for p in self.particles:
            if p['w'] > highest_weight:
                highest_weight = p['w']
                best_particle = p
        return best_particle
