# Comprobo Robot Localization Project

_ENGR3590: A Computational Introduction to Robotics_
_Ansel Harris Crowne, Brenna O'Donnell, Julian Shah_

---

<p align="center">
<img src="media/finished_filter.gif" alt="drawing" width="70%"/>
</p>

### Project Goals:

This project focuses solely on a particular form of bayesian optimization called a particle filter to localize a robot in a given map. The filter creates a set of random position guesses called particles, and adjusts them using the robot's LiDAR scan data until it converges upon the robot's actual position.

## Code Architecture

The code is built around ROS2. This allowed for easy testing (replay bag files), visualisation/debugging (RViz/RQT), and the ability to easily adapt this implementation onto many ROS2 compatible robots. Every ROS2 node in Python needs to be structured in a class, which was practical since there were many shared variables within the code. The Bayesian filter components run independently within the class through a main loop. This loop intializes the particles then determines when the robot has moved a sufficient degree to run the next iteration of the filter: update particle positions from odometry data, assign weights to particles, and resamples particles.

The vast majority of advanced math was done using numpy and helper functions from the helper_functions.py and occupancy_field.py. The occupancy field in particular not only saved the map as an occupancy_field but also precomputes a likelihood field for the map making particle weighting much faster and more accurate.

### initialize_particle_cloud()

run_loop() begins by setting up all the necessary transforms to update a particle could, then initializes said particle could. To generate an initial set of particles, we used the method of evenly distributing most particles and evenly distributing a few "extra". We get the width and height of the map using the bounding box obtained with the prewritten function get_obstacle_bounding box from occupancy_field.py. We then use a nested for loop to generate particles that are evenly spaced in a grid on the map each with a random theta. The particles designated to be random are then randomized in both their x,y,theta. The weights of the particles are then normalized to sum to 0, in this case all being assigned the same weight. [ image of the particles nicely spaced at the start of the visualization ]

### update_particles_with_odom()

This function gets the difference in the robots past and current pose and updates the particles using this change. (explain better once complete).

### update_particles_with_laser()

Maps the laserscan data from the robot onto the particles relative to their orientation. Then uses a likelihood field (OccupancyField.get_closest_obstacle_distance()) to find the error between the mapped scan the map. Each scan point distance is then summed which is then inversed to get the weight. The inversion is important because the higher the sum the less likely it is to be close to the true robot pose.

Some minor optimization here from doing vector math isntead of iterating through a for loop for each scan point. Though undocumented, the get_closest_obstacle_distance() function can take an array and solve for the distances all at once so this feature was made use of.

### resample_particles()

This method sorts the particles and chooses where are the best spots to add more particles in order to approach the true robot position. Our method takes a percentage of the highest weighted particles (p_high) and distributes a percentage of the rest (p_rest) of the particles nearby their pose and orientation in the hopes of finding a better match for the scan data. This is done through a gaussian distribution so that most of the particles are near the particle, however some amount of them will be farther away. This baked in error helps compensate for the cumulative error from things like the scanner noise or the discrete abstraction of the map (among other things).

[image of gaussian distribution]

The rest of the unallocated particles are sampled randomly across the map bounding box in order to reduce the likelihood of converging into an incorrect position. This is another element of controlled, baked in, error.

The last layer of complexity that helps the particles converge properly is a scaling of p_high and p_rest as the filter progresses. The idea here is that initially the model will be very wrong so lots of random particles are good to compensate. However, as the model progresses, the model will begin converging on the highest weight areas until it has nearly all the particles at the highest weight area which should be very close to where the robot actually is. Our implementation implements this linearly, but could be easily adapted for an asymptotic function of some sort.

In practise, it is mostly good with some bad. It helps the model more consistently converge on the target location (where before it was apprx 1 in 10 runs converging now it's 3 in 5 times convergin). It also makes complex movements, like the neato turning, stay in the true particle location.

[image of turning in run 1]

### update_robot_pose()

Updates the robot position based on the best particle in the particle cloud.

## Design Decisions

We chose to generate our particles uniformly accross the map and converge using using normal distributions that localize around the best particles. We define the "best particles" based on a percentage of the total number that grows smaller with each iteration. Hence, as the particle filter converges on the location of the robot, it sample from fewer areas. To fine tune the convergence, we defined the standard deviation of the normal distribution at each "best particle" based on some constant divded by the weight of the "best particle". This constant controls our confidence in a given weight. To lower the odds of false convergence, a proportion of particles are also randomly resampled. Like the determination of the "best particles", the proportion of random particles is controlled by a percentage that decreases with each iteration. The randomness, thus, decreases as the filter becomes more confident (hopefully) in the robot's location.

## Challenges

One of our greatest challenges ended up being working with the skeleton code. We spent a lot of time trying to figure out what functionality was already provided and what needed to be added. That is, not to mention resolving resulting errors with types and data storage. On top of that, because the skeleton code was written awhile ago, we ran into some version issues which further complicated our debugging efforts. Another learning curve was visualization. Tuning and debugging the particle filter rested heavily on being able to visualize the results using rviz.

### Improvements

There are a number of ways we would continue to iterate on our filter given the time and resources. For example, the initial uniform distribution of particles is rather inefficient. Rather than mindlessly generating them around the map, we could save time and processing power by only generating particles within the map and away from walls. This would allow us to significantly reduce the total number of particles. We could apply a similar practice to resampling as well, allowing it to converge even faster. It would also be a prudent to refactor the code without necessarily changing the logic to make it run faster. The faster the code runs, the more quickly and precicely the particle filter can respond to subtle changes in the robot's laser data that conflict with our projected pose. Particularly improvements could easily be found around redistributing and weighing the particles.

Another area of improvement could likely come from changing the scaling function. Right now it is linear, however a capped exponential decay function would likely improve convergence chances (based on visually seeing how to particles converge).

### Lessons for the Future

Our team had a lot of difficulty with the code working to different degrees on each team member's device. One of these inconsistencies was the the numpy version that each person had. Some of the mathematics were written for numpy 1.2, which uses some variable types that are incompatible with newer numpy versions. Figuring out which numpy to use, what was wrong in the first place, then rewritting affected code took up time and contributed to getting the team very out of sync. Had we dealt with the version and technology issues that held us back early in the project would have allowed us to function much better as a team, and hence, produce an even better product.
