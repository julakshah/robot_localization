# ROS BAG INFO

This file details the process used for recording bag files as well as how to
play them back. It contains key takeaways from every bag file run. Each bag file
was recorded in directly subsequent runs of the program with different random
seeds every time.

Take 1-3 were run off the macfirst_floor_take_1 bag file while takes 4-6 were 
run off the macfirst_floor_take_2 bag file. For the exploration of the files,
only recordings 1-3 will be considered. 4-6 are a reference for how the filter
reacts to a different set of sensor data.

## Recording the rosbags

The following command was used to record the ros2 bag files.

```
ros2 bag record -a 
```

The -a flag is used to record all topics. This is alright since the code was
recorded off another bag file that had already removed the unnecessary data 
(such as unused camera data).

## take1

This take is a good example of the code running as expected. After a few
iterations, the code converges upon the correct location and stays pretty locked
on the location. The expected error is present upon odom updates.

## take2

This take begins off track and really locks in towards the end. This is a good
example of how a random sample can get in the way of the most converged 
particles. Considering that most of particles are converged in the right 
location, perhaps implementing some form of weighting where most of the
particles are will make the filter converge better.

## take3

This run is a good example on how similar environment geometry can make the 
filter converge on the wrong area. It subsequently finds the correct area but
it is a fundamental flaw of the stochasticity of the filter and I can only think
to fix it with a lot of fine tuning and potentially extra features.
