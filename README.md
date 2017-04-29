# Particle Filter Localization

This code implements the MCL algorithm for the RACECAR. 

[![YouTube Demo](./media/thumb.jpg)](https://www.youtube.com/watch?v=-c_0hSjgLYw)

For high efficiency in Python, it uses Numpy arrays and RangeLibc for fast 2D ray casting.

# Installation

To run this, you need to ensure that both the map_server ROS package, and the python wrappers for RangeLibc are installed.

For the map server:
```
sudo apt-get update
rosdep install -r --from-paths src --ignore-src --rosdistro kinetic -y
```

For RangeLibc:

```
sudo pip install cython
git clone http://github.com/kctess5/range_libc
cd range_libc/pywrappers
# on VM
sudo python setup.py install
# on car - compiles GPU ray casting methods
sudo WITH_CUDA=ON python setup.py install
```

# Usage

The majority of parameters you might want to tweak are in the launch/localize.launch file. You may have to modify the "odometry_topic" parameter to match your environment.

```
roslaunch ta_lab5 localize.launch
```

Once the particle filter is running, you can visualize the map and other particle filter visualization message in RViz. Use the "2D Pose Estimate" tool from the RViz toolbar to initialize the particle locations.

See [launch/localize.launch](/ta_lab5/launch/localize.launch) for docs on available parameters and arguments.

The "range_method" parameter determines which RangeLibc ray casting method to use. The default is cddt because it is fast and has a low initialization time. The fastest option on the CPU is "glt" but it has a slow startup. The fastest version if you have can compile RangeLibc with CUDA enabled is "rmgpu". See this performance comparison chart:

![Range Method Performance Comparison](./media/comparison.png)

# Docs

This code is the staff solution to the lab guide found in the [/docs](/ta_lab5/docs) folder. A mathematical derivation of MCL is available in that guide.

There is also documentation on RangeLibc in the [/docs](/ta_lab5/docs) folder.

The code itself also contains comments describing purpose of each method.