#!/usr/bin/env python3

"""This is the starter code for the robot localization project"""

import rclpy
from threading import Thread
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from nav2_msgs.msg import ParticleCloud, Particle
from nav2_msgs.msg import Particle as Nav2Particle
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import math
import time
import numpy as np
from occupancy_field import OccupancyField
from helper_functions import TFHelper, draw_random_sample
from rclpy.qos import qos_profile_sensor_data
from angle_helpers import quaternion_from_euler


class Particle(object):
    """Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
    Attributes:
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of the hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
    """

    def __init__(self, x=0.0, y=0.0, theta=0.0, w=1.0):
        """Construct a new Particle
        x: the x-coordinate of the hypothesis relative to the map frame
        y: the y-coordinate of the hypothesis relative ot the map frame
        theta: the yaw of KeyboardInterruptthe hypothesis relative to the map frame
        w: the particle weight (the class does not ensure that particle weights are normalized
        """
        self.w = w
        self.theta = theta
        self.x = x
        self.y = y

    def as_pose(self):
        """A helper function to convert a particle to a geometry_msgs/Pose message"""
        q = quaternion_from_euler(0, 0, self.theta)
        return Pose(
            position=Point(x=self.x, y=self.y, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
        )

    # TODO: define additional helper functions if needed


class ParticleFilter(Node):
    """The class that represents a Particle Filter ROS Node
    Attributes list:
        base_frame: the name of the robot base coordinate frame (should be "base_footprint" for most robots)
        map_frame: the name of the map coordinate frame (should be "map" in most cases)
        odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
        scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
        n_particles: the number of particles in the filter
        d_thresh: the amount of linear movement before triggering a filter update
        a_thresh: the amount of angular movement before triggering a filter update
        pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
        particle_pub: a publisher for the particle cloud
        last_scan_timestamp: this is used to keep track of the clock when using bags
        scan_to_process: the scan that our run_loop should process next
        occupancy_field: this helper class allows you to query the map for distance to closest obstacle
        transform_helper: this helps with various transform operations (abstracting away the tf2 module)
        particle_cloud: a list of particles representing a probability distribution over robot poses
        current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                               The pose is expressed as a list [x,y,theta] (where theta is the yaw)
        thread: this thread runs your main loop
    """

    def __init__(self):
        super().__init__("pf")
        self.base_frame = "base_footprint"  # the frame of the robot base
        self.map_frame = "map"  # the name of the map coordinate frame
        self.odom_frame = "odom"  # the name of the odometry coordinate frame
        self.scan_topic = "scan"  # the topic where we will get laser scans from
        # the width and height of bounding box, set in initialize_particle_cloud
        self.width = 0
        self.height = 0
        self.x_low = 0 #left bound
        self.x_up = 0 #right bound
        self.y_low = 0 #bottom bound
        self.y_up = 0 #top bound

        self.n_particles = 1000  # the number of particles to use

        self.d_thresh = 0.2  # the amount of linear movement before performing an update
        self.a_thresh = (
            math.pi / 6
        )  # the amount of angular movement before performing an update

        # TODO: define additional constants if needed

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.update_initial_pose, 10
        )

        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = self.create_publisher(
            ParticleCloud, "particle_cloud", qos_profile_sensor_data
        )

        # laser_subscriber listens for data from the lidar
        self.create_subscription(LaserScan, self.scan_topic, self.scan_received, 10)

        # this is used to keep track of the timestamps coming from bag files
        # knowing this information helps us set the timestamp of our map -> odom
        # transform correctly
        self.last_scan_timestamp = None
        # this is the current scan that our run_loop should process
        self.scan_to_process = None
        # your particle cloud will go here
        self.particle_cloud = []

        self.current_odom_xy_theta = []
        self.occupancy_field = OccupancyField(self)
        self.transform_helper = TFHelper(self)

        # we are using a thread to work around single threaded execution bottleneck
        thread = Thread(target=self.loop_wrapper)
        thread.start()
        self.transform_update_timer = self.create_timer(0.05, self.pub_latest_transform)

    def pub_latest_transform(self):
        """This function takes care of sending out the map to odom transform"""
        if self.last_scan_timestamp is None:
            return
        postdated_timestamp = Time.from_msg(self.last_scan_timestamp) + Duration(
            seconds=0.1
        )
        self.transform_helper.send_last_map_to_odom_transform(
            self.map_frame, self.odom_frame, postdated_timestamp
        )

    def loop_wrapper(self):
        """This function takes care of calling the run_loop function repeatedly.
        We are using a separate thread to run the loop_wrapper to work around
        issues with single threaded executors in ROS2"""
        while True:
            self.run_loop()
            time.sleep(0.1)

    def run_loop(self):
        """This is the main run_loop of our particle filter.  It checks to see if
        any scans are ready and to be processed and will call several helper
        functions to complete the processing.

        You do not need to modify this function, but it is helpful to understand it.
        """
        if self.scan_to_process is None:
            return
        msg = self.scan_to_process

        (new_pose, delta_t) = self.transform_helper.get_matching_odom_pose(
            self.odom_frame, self.base_frame, msg.header.stamp
        )
        if new_pose is None:
            # we were unable to get the pose of the robot corresponding to the scan timestamp
            if delta_t is not None and delta_t < Duration(seconds=0.0):
                # we will never get this transform, since it is before our oldest one
                self.scan_to_process = None
            return

        (r, theta) = self.transform_helper.convert_scan_to_polar_in_robot_frame(
            msg, self.base_frame
        )
        print("r[0]={0}, theta[0]={1}".format(r[0], theta[0]))
        # clear the current scan so that we can process the next one
        self.scan_to_process = None

        self.odom_pose = new_pose
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )
        print("x: {0}, y: {1}, yaw: {2}".format(*new_odom_xy_theta))

        if not self.current_odom_xy_theta:
            self.current_odom_xy_theta = new_odom_xy_theta
        elif not self.particle_cloud:
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud(msg.header.stamp)
        elif self.moved_far_enough_to_update(new_odom_xy_theta):
            # we have moved far enough to do an update!
            self.update_particles_with_odom()  # update based on odometry
            self.update_particles_with_laser(r, theta)  # update based on laser scan
            self.update_robot_pose()  # update robot's pose based on particles
            self.resample_particles()  # resample particles to focus on areas of high density
        # publish particles (so things like rviz can see them)
        self.publish_particles(msg.header.stamp)

    def moved_far_enough_to_update(self, new_odom_xy_theta):
        return (
            math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1])
            > self.d_thresh
            or math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            > self.a_thresh
        )

    def update_robot_pose(self):
        """Update the estimate of the robot's pose given the updated particles.
        There are two logical methods for this:
            (1): compute the mean pose
            (2): compute the most likely pose (i.e. the mode of the distribution)
        """
        # option 2

        # first make sure that the particle weights are normalized
        self.normalize_particles()

        # get the best particle
        best_particle = max(self.particle_cloud, key=lambda p: p.w)

        x = best_particle.x
        y = best_particle.y
        theta = best_particle.theta

        new_pose = quaternion_from_euler(0, 0, theta)
        self.robot_pose = Pose(
            position=Point(x=x, y=y),
            orientation=Quaternion(
                x=new_pose[0], y=new_pose[1], z=new_pose[2], w=new_pose[3]
            ),
        )

        if hasattr(self, "odom_pose"):
            self.transform_helper.fix_map_to_odom_transform(
                self.robot_pose, self.odom_pose
            )
        else:
            self.get_logger().warn(
                "Can't set map->odom transform since no odom data received"
            )

    def update_particles_with_odom(self):
        """Update the particles using the newly given odometry pose.
        The function computes the value delta which is a tuple (x,y,theta)
        that indicates the change in position and angle between the odometry
        when the particles were last updated and the current odometry.
        """
        delta = (0,0,0)
        new_odom_xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
            self.odom_pose
        )
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (
                new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                new_odom_xy_theta[2] - self.current_odom_xy_theta[2],
            )

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        odom_noise = .2
        heading_noise = 0.05
        for p in self.particle_cloud:
            ti = p.theta; xi = p.x; yi = p.y;
            td = delta[2]; xd = delta[0]; yd = delta[1];
            Ti = np.array([[np.cos(ti), -np.sin(ti), xi],
                          [np.sin(ti), np.cos(ti), yi],
                          [0, 0, 1]])
            T_del = np.array([[np.cos(td), -np.sin(td), xd],
                          [np.sin(td), np.cos(td), yd],
                          [0, 0, 1]])
            T_new = Ti @ T_del
            p.x = T_new[0, 2]
            p.y = T_new[1, 2]
            p.theta = float(np.arctan2(T_new[1,0], T_new[0, 0]))
            #p.x += delta[0] + np.random.randn() * odom_noise 
            #p.y += delta[1] + np.random.randn() * odom_noise
            #p.theta += delta[2] + np.random.randn() * heading_noise

    def random_p(self):
        """function to get a random particle within the map"""
        x = self.x_low + np.random.rand() * self.width
        y = self.y_low + np.random.rand() * self.height
        t = rand_theta = np.random.rand() * np.pi * 2  # radians
        return Particle(x=x, y=y, theta=t)


    def resample_particles(self):
        """Resample the particles according to the new particle weights.
        The weights stored with each particle should define the probability that a particular
        particle is selected in the resampling step.  You may want to make use of the given helper
        function draw_random_sample in helper_functions.py.
        """
        # make sure the distribution is normalized
        self.normalize_particles()
        # Create a dictionary to store particles with keys representing the weights of each
        ######################

        particles = {}
        for particle in self.particle_cloud:
            particles[particle.w] = particle
        sorted_p_keys = sorted(list(particles.keys()), reverse=True)
        center_percent = .15
        redist_percent = .7
        random_percent = 1-center_percent-redist_percent
        
        center_keys = sorted_p_keys[:round(self.n_particles * center_percent)]
        dist_num = int(np.floor(self.n_particles * (center_percent + redist_percent))) 
        redist_num = int(np.floor(dist_num / len(center_keys)))
        lin_sd = .5
        ang_sd = .02
        self.particle_cloud = []
        for ik, key in enumerate(center_keys):
            for i in range(redist_num):
                new_x = np.random.normal(particles[key].x, lin_sd)
                new_y = np.random.normal(particles[key].y, lin_sd)
                new_theta = np.random.normal(particles[key].y, ang_sd)
                self.particle_cloud.append(Particle(x=new_x, y=new_y, theta=new_theta)) 

        num_filled = len(center_keys) * redist_num
        num_extra = self.n_particles - num_filled
        for e in range(num_extra):
            x_pos = self.x_low + np.random.rand() * self.width
            y_pos = self.y_low + np.random.rand() * self.height
            rand_theta = np.random.rand() * np.pi * 2  # radians
            self.particle_cloud.append(Particle(
                x=x_pos, y=y_pos, theta=rand_theta
            ))
        ######################

    def update_particles_with_laser(self, r, theta):
        """Updates the particle weights in response to the scan data
        r: the distance readings to obstacles
        theta: the angle relative to the robot frame for each corresponding reading
        """
        # TODO: implement this
        #   Julian
        ######################
        print("running update")
        for p in self.particle_cloud:
            particle_ang = p.theta  # radians
            x_list = r * np.sin(theta + particle_ang) + p.x
            y_list = r * np.cos(theta + particle_ang) + p.y
            weights = self.occupancy_field.get_closest_obstacle_distance(x_list, y_list)
            tot_weight = 0
            for w in weights:
                #print(f"w: {w}")
                #print(f"w type: {type(w)}")
                if np.isnan(w):
                    tot_weight += 200
                else:
                    tot_weight += w

            p.w = 1/tot_weight
        ######################

    def update_initial_pose(self, msg):
        """Callback function to handle re-initializing the particle filter based on a pose estimate.
        These pose estimates could be generated by another ROS Node or could come from the rviz GUI
        """
        xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(msg.header.stamp, xy_theta)

    def initialize_particle_cloud(self, timestamp, xy_theta=None):
        """Initialize the particle cloud.
        Arguments
        xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                  particle cloud around.  If this input is omitted, the odometry will be used
        """
        if xy_theta is None:
            xy_theta = self.transform_helper.convert_pose_to_xy_and_theta(
                self.odom_pose
            )
        self.particle_cloud = []
        # TODO create particles
        #   Julian
        ######################
        ((xl, xu), (yl, yu)) = self.occupancy_field.get_obstacle_bounding_box()
        self.x_up = xu
        self.x_low = xl
        self.y_up = yu
        self.y_low = yl
        self.width = xu - xl  # directionless
        self.height = yu - yl  # directionless
        print(f"width: {self.width}, height: {self.height}")
        print(f"x_low: {self.x_low}, y_low: {self.y_low}")
        print(f"x_up: {self.x_up}, y_up: {self.y_up}")

        grid_size = int(np.sqrt(self.n_particles))  # smallest square grid of particles
        width_increment = self.width / (grid_size + 1)
        height_increment = self.height / (grid_size + 1)
        for i in range(grid_size):
            for j in range(grid_size):
                x_pos = self.x_low + (width_increment * (i + 1))
                y_pos = self.y_low + (height_increment * (j + 1))
                rand_theta = np.random.rand() * np.pi * 2  # radians
                self.particle_cloud.append(Particle(
                    x=x_pos, y=y_pos, theta=rand_theta
                ))

        # randomly distribute extra
        grid_num = grid_size**2
        extra = self.n_particles - grid_num

        for e in range(extra):
            x_pos = self.x_low + np.random.rand() * self.width
            y_pos = self.y_low + np.random.rand() * self.height
            rand_theta = np.random.rand() * np.pi * 2  # radians
            self.particle_cloud.append(Particle(
                x=x_pos, y=y_pos, theta=rand_theta
            ))
        ######################
        self.normalize_particles()
        self.update_robot_pose()

    def normalize_particles(self):
        """Make sure the particle weights define a valid distribution (i.e. sum to 1.0)"""
        total_weight = sum(particle.w for particle in self.particle_cloud)

        for p in self.particle_cloud:
            p.w /= total_weight

    def publish_particles(self, timestamp):
        msg = ParticleCloud()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = timestamp
        for p in self.particle_cloud:
            msg.particles.append(Nav2Particle(pose=p.as_pose(), weight=p.w))
        self.particle_pub.publish(msg)

    def scan_received(self, msg):
        self.last_scan_timestamp = msg.header.stamp
        # we throw away scans until we are done processing the previous scan
        # self.scan_to_process is set to None in the run_loop
        if self.scan_to_process is None:
            self.scan_to_process = msg


def main(args=None):
    rclpy.init()
    n = ParticleFilter()
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
