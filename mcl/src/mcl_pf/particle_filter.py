import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from scipy.stats import norm


class ParticleFilter(object):
    def __init__(self, num_particles=100):
        self.num_particles = num_particles
        self.particles_x = None
        self.particles_y = None
        self.particles_t = None
        self.dx = 0.
        self.dy = 0.
        self.da = 0.
        self.weights = None

    def initialise(self, pose, pose_sigma=.1, rot_sigma=.2):
        """Intialise the particle filter using a normal distribution.

        :param pose: The geometry_msgs/PoseStamped that forms the centre of the normal distribution
        :param pose_sigma: The std dev of the normal distribution for the x, y coordinates (default=0.1)
        :param pose_sigma: The std dev of the normal distribution for the rotation in radians (default=0.2)
        :returns: A triple of lists that contain the generate x, y coordinates and the angles theta
        """
        rospy.loginfo("Initialising particle filter at ({}, {}, {}) - ({}, {}, {}, {})".format(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ))
        self.particles_x, self.particles_y, self.particles_t = self.random_sample_pose(pose,self.num_particles, pose_sigma, rot_sigma)
        return self.particles_x, self.particles_y, self.particles_t

    def random_sample_pose(self, pose, n, pose_sigma=.1, rot_sigma=.2):
        """Generates random samples from a given pose using a normal distribution.

        :param pose: The geometry_msgs/PoseStamped that forms the centre of the normal distribution
        :param n: The number of particles to generate
        :param pose_sigma: The std dev of the normal distribution for the x, y coordinates (default=0.1)
        :param pose_sigma: The std dev of the normal distribution for the rotation in radians (default=0.2)
        :returns: A triple of lists that contain the generate x, y coordinates and the angles theta
        """
        x = pose.pose.position.x
        y = pose.pose.position.y
        _, _, yaw = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w))
        return self.random_sample(x, y, yaw, n, pose_sigma, rot_sigma)

    def random_sample(self, x, y, a, n, pose_sigma=.1, rot_sigma=.2):
        """Generates random samples from a given 2D position and angle using a normal distribution.

        :param x: The x-coordinate of the centre of the normal destribution
        :param y: The y-coordinate of the centre of the normal destribution
        :param a: The rotation
        :param n: The number of particles to generate
        :param pose_sigma: The std dev of the normal distribution for the x, y coordinates (default=0.1)
        :param pose_sigma: The std dev of the normal distribution for the rotation in radians (default=0.2)
        :returns: A triple of lists that contain the generate x, y coordinates and the angles theta
        """

        # Generating a 1D normal distribution for each value with a given sigma and sampling n particles from it
        x = np.random.normal(x, pose_sigma, n)
        y = np.random.normal(y, pose_sigma, n)
        t = np.random.normal(a, rot_sigma, n)
        return x, y, t

    def update(self, dx, dy, da, dt):
        """Update to constant motion model.

        :param dx: The change in x direction since the last time step
        :param dy: The change in y direction since the last time step
        :param da: The change in rotation since the last time step
        :param dt: The length of the interval over which this change in position happened
        """

        # Normalising the given deltas for x, y, and a to meters and radians per second
        factor = 1./dt
        self.dx = dx * factor
        self.dy = dy * factor
        self.da = da * factor
        rospy.logdebug("Update: {}, {}, {}".format(self.dx, self.dy, self.da))

    def predict(self, dt):
        """Predicting the new position at the given time based on the constant velocity model.

        :param dt: The time since the last update
        :returns: A triple of lists that contain the generate x, y coordinates and the angles theta
        """

        # Assuming a constant velocity, we just multiply the last observed speeds by the time 
        # increment given to update the particles
        self.particles_x += (self.dx * dt)
        self.particles_y += (self.dy * dt)
        self.particles_t += (self.da * dt)
        return self.particles_x, self.particles_y, self.particles_t

    def observe(self, x, y, t, pose_sigma=.1, rot_sigma=.2):
        """Making a new observation and changing the particle weights based on distance using a normal distribution.

        :param x: The x-coordinate of the new observation.
        :param y: The y-coordinate of the new observation.
        :param t: The rotation of the new observation.
        :param pose_sigma: The std dev of the normal distribution for the x, y coordinates (default=0.1)
        :param pose_sigma: The std dev of the normal distribution for the rotation in radians (default=0.2)
        :returns: The updated list of weights.
        """

        # The Normal Probability Density Function is used to calculate weights for each particle based on
        # the observed values for x, y, and angle t
        xw = norm.pdf(self.particles_x, x, pose_sigma)
        yw = norm.pdf(self.particles_y, y, pose_sigma)
        tw = norm.pdf(self.particles_t, t, rot_sigma)

        #Summing and normalising the weights
        self.weights = xw + yw + tw
        self.weights /= np.sum(self.weights)

        return self.weights

    def resample(self, pose_sigma=.1, rot_sigma=.2, starvation_factor=0.2):
        """Resamples the current particles based on the weights calculated during the last observation.

        If the starvation factor is > 0, not all particles will be resampled based on weights but a percentage
        will be randomly sampled

        :param pose_sigma: The std dev of the normal distribution for the x, y coordinates (default=0.1)
        :param pose_sigma: The std dev of the normal distribution for the rotation in radians (default=0.2)
        :param starvation_factor: The percentage of particles that should be sampled randomly instead of based on their weights (default=0.2)
        :returns: A triple of lists that contain the generate x, y coordinates and the angles theta
        """
        # Num particles to be sampled based on weight
        n = int(np.round(self.num_particles*(1.-starvation_factor)))
        # Creating indices for new particles based on weights
        new_particles = np.random.choice(range(self.num_particles), replace=True, size=n, p=self.weights)

        # Sampling random particle according to starvation factor
        x, y, t = self.random_sample(np.mean(self.particles_x), np.mean(self.particles_y), np.mean(self.particles_t), self.num_particles-n, pose_sigma, rot_sigma)

        # Updating particles with new sample
        self.particles_x = np.concatenate([self.particles_x[new_particles], x])
        self.particles_y = np.concatenate([self.particles_y[new_particles], y])
        self.particles_t = np.concatenate([self.particles_t[new_particles], t])
        return self.particles_x, self.particles_y, self.particles_t


