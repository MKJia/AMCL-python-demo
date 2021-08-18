import numpy as np #
import scipy.stats #
from numpy.random import uniform,randn,random
import matplotlib.pyplot as plt
 
 
def create_uniform_particles(x_range, y_range, hdg_range, N):
	#set params of particles:
	#x_range = [0,20]
	#y_range = [0,20]
	#heading_degree_range = [0,2*np.pi]

    particles = np.empty((N, 3)) 	#create an empty matrix for N particles and each particle has 3 dimensions
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N) 	#dimension 1 is the x coordinate of the particles
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N) 	#dimension 2 is the y coordinate of the particles
    particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=N) 	#dimension 3 is the heading degree of the particles
    particles[:, 2] %= 2 * np.pi 	#mapping the heading degree to [0, 1]
    return particles 	#return the created particles matrix
 
     
def predict_ptc_state(particles, step_displacement, noise, N):
    """ move according to control input step_displacement (palstance, velocity)
    with noise Q (noise heading change, noise velocity)"""
 
    # update_ptc_weight heading
    particles[:, 2] += step_displacement[0] + (randn(N) * noise[0])
    particles[:, 2] %= 2 * np.pi
 
    # move in the (noisy) commanded direction
    dist = step_displacement[1] + (randn(N) * noise[1])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist   
     
     
def update_ptc_weight(particles, weights, rd, err, landmarks):
    weights.fill(1.)
    for i, landmark in enumerate(landmarks): #i for the index and landmark for the coordinate
        particles_distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)
        weights *= scipy.stats.norm(particles_distance, err).pdf(rd[i])
        weights *= scipy.stats.norm(particles[:,2], err*5).pdf(0.125)

	weights += 1.e-300      # avoid round-off to zero
    weights /= sum(weights)	# normalize   
    
     
def estimate(particles, weights):
    """returns mean and variance of the weighted particles"""
 
    pos = particles[:, 0:2]
    mean = np.average(pos, weights=weights, axis=0)
    var  = np.average((pos - mean)**2, weights=weights, axis=0)
    return mean, var   
     
     
def num_of_effected_particles(weights):
    return 1. / np.sum(np.square(weights)) 
     
     
def simple_resample(particles, weights):
    N = len(particles)

    cumulative_sum = np.cumsum(weights)
  
    # print cumulative_sum
    cumulative_sum[-1] = 1. 	# avoid round-off error
    indexes = np.searchsorted(cumulative_sum, random(N))
    for i in range(5000):
    	print weights[i]
    	print cumulative_sum[i]  
    # resample according to indexes
    particles[:] = particles[indexes]
    weights[:] = weights[indexes]
    weights /= np.sum(weights) 	# normalize   

def all_plot(predict_particles, moveSteps, landmarks):
    predict_particles = np.array(predict_particles)
    plt.plot(np.arange(moveSteps+1),'k+')
    plt.plot(predict_particles[:, 0], predict_particles[:, 1],'r.')
    plt.scatter(landmarks[:,0],landmarks[:,1],alpha=0.4,marker='o',c=randn(6),s=100) # plot landmarks
    plt.legend( ['Actual','PF'], loc=6, numpoints=1)
    plt.xlim([-2,20])
    plt.ylim([0,22])
    plt.show()

def particles_plot(particles):
    plt.xlim([-2,22])
    plt.ylim([-2,22])
    plt.plot(particles[0],particles[1],'r.')
    #r. - red , b. - blue , k. - black
    plt.show()

def run_pf(N, moveSteps=18, sensor_noise_err=0.05, xlim=(0, 20), ylim=(0, 20)):   
    landmarks = np.array([[-1, 2], [3, 9], [5, 15] ,[9, 13], [12, 18], [18,21]]) 	#set 6 landmarks
    number_of_landmarks = len(landmarks) 
     
    # create particles and weights
    particles = create_uniform_particles((0,20), (0,20), (0, 2*np.pi), N)
    weights = np.zeros(N) 	#create the weight of the particles(initialized with 0)

    predict_particles = []   # estimated values								
    robot_pos = np.array([0., 0.]) 	# create positon array as [x, y]
     
    #a map of [20, 20] and step by [1, 1] for 18 steps
    for x in range(moveSteps):
        robot_pos += (1, 1) 
         
        # distance from robot to each landmark
        real_distance = np.linalg.norm(landmarks - robot_pos, axis=1) #the real distance
        real_distance += randn(number_of_landmarks) * sensor_noise_err #add gaussian noise
         
        # move particles forward to (x+1, x+1)
        predict_ptc_state(particles, step_displacement=(0.00, 1.414), noise=(.2, .05), N=N)
         
        # incorporate measurements
        update_ptc_weight(particles, weights, rd=real_distance, err=sensor_noise_err, landmarks=landmarks)
         
        # resample if too few effective particles
        noep = num_of_effected_particles(weights)
        if num_of_effected_particles(weights) < N/2:
            simple_resample(particles, weights)
         
        # Computing the State Estimate
        mu, var = estimate(particles, weights)
        print 'estimated position and variance:\n\t', mu, var
        predict_particles.append(mu)
    
    all_plot(predict_particles, moveSteps, landmarks)
     
     
     
if __name__ == '__main__':   
    run_pf(N=5000)
