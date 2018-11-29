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
        x = pose.pose.position.x
        y = pose.pose.position.y
        _, _, yaw = euler_from_quaternion((pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w))
        return self.random_sample(x, y, yaw, n, pose_sigma, rot_sigma)

    def random_sample(self, x, y, a, n, pose_sigma=.1, rot_sigma=.2):
        x = np.random.normal(x, pose_sigma, n)
        y = np.random.normal(y, pose_sigma, n)
        t = np.random.normal(a, rot_sigma, n)
        return x, y, t

    def update(self, dx, dy, da, dt):
        factor = 1./dt
        self.dx = dx * factor
        self.dy = dy * factor
        self.da = da * factor
        rospy.logdebug("Update: {}, {}, {}".format(self.dx, self.dy, self.da))

    def predict(self, dt):
        self.particles_x += (self.dx * dt)
        self.particles_y += (self.dy * dt)
        self.particles_t += (self.da * dt)
        return self.particles_x, self.particles_y, self.particles_t

    def observe(self, x, y, t, pose_sigma=.1, rot_sigma=.2):
        xw = norm.pdf(self.particles_x, x, pose_sigma)
        yw = norm.pdf(self.particles_y, y, pose_sigma)
        tw = norm.pdf(self.particles_t, t, rot_sigma)

        self.weights = xw + yw + tw
        self.weights /= np.sum(self.weights)

        return self.weights

    def resample(self, pose, pose_sigma=.1, rot_sigma=.2, starvation_factor=0.2):
        n = int(np.round(self.num_particles*(1.-starvation_factor)))
        new_particles = np.random.choice(range(self.num_particles), replace=True, size=n, p=self.weights)
        x, y, t = self.random_sample(np.mean(self.particles_x), np.mean(self.particles_y), np.mean(self.particles_t), self.num_particles-n, pose_sigma, rot_sigma)
        self.particles_x = np.concatenate([self.particles_x[new_particles], x])
        self.particles_y = np.concatenate([self.particles_y[new_particles], y])
        self.particles_t = np.concatenate([self.particles_t[new_particles], t])
        return self.particles_x, self.particles_y, self.particles_t


