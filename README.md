# Comprobo Robot Localization Project
*ENGR3590: A Computational Introduction to Robotics*
*Ansel Harris Crowne, Brenna O'Donnell, Julian Shah*

-----------------------------------------------------------------------------------------------------------
### Project Goals: 
This project focuses solely on a particular form of bayesian optimization called a particle filter to localize a neato in a given map. The filter creates a set of random position guesses called particles, and adjusts them using the robot's LiDAR scan data until it converges upon the robot's actual position. 

## Code Architecture
[ map thing ]
We mostly followed the skeleton code provided for the project (by Paul Ruvolo) which has both the Particle class and ParticleFilter class containing all of the logic in one file called pf.py which some additional framing, math, and map functions in their own .py files (helper_functions, angle_helpers, occupancy_field). pf.py follows a very linear path, running mainly out of run_loop() which checks to see if the robot has moved enough to do anything. It then calls all of the filtering functions in order, publishes the new set of particles, and repeats.

### initialize_particle_cloud()
run_loop() begins by setting up all the necessary transforms to update a particle could, then initializes said particle could. To generate an initial set of particles, we used the method of evenly distributing most particles and evenly distributing a few "extra". We get the width and height of the map using the bounding box obtained with the prewritten function get_obstacle_bounding box from occupancy_field.py. We then use a nested for loop to generate particles that are evenly spaced in a grid on the map each with a random theta. The particles designated to be random are then randomized in both their x,y,theta. The weights of the particles are then normalized to sum to 0, in this case all being assigned the same weight.
[ image of the particles nicely spaced at the start of the visualization ]

### update_particles_with_odom()
This function gets the difference in the robots past and current pose and updates the particles using this change. (explain better once complete).

### update_particles_with_laser()


### update_robot_pose()
### resample_particles()

## Design Decisions
words

## Challenges
words

##

### Improvements
words

### Lessons for the Future
Our team had a lot of difficulty with the code working to different degrees on each team member's device. One of these inconsistencies was the the numpy version that each person had. Some of the mathematics were written for numpy 1.21, which uses some variable types that are incompatible with newer numpy versions. Figuring out which numpy to use, what was wrong in the first place, then rewritting affected code took up time and contributed to getting the team very out of sync.
