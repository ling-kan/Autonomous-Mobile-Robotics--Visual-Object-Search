# University of Lincoln


![University of Lincoln](http://thelincolnite.co.uk/wp-content/uploads/2012/07/new_uni_crest.jpg "University of Lincoln")


----------

# CMP3103 Autonomous-Mobile-Robotics

Your task is to implement a behaviour that enables the robot in simulation to find a total of 4 objects
distributed in a simulated environment. You need to utilise the robot’s sensory input and its actuators
to guide the robot to each item. Success in locating an item is defined as: (a) being less than 1m from
the item, and (b) indication from the robot that it has found an object.

For the development and demonstration of your software component, you will be provided with a
simulation environment (called “Gazebo”). The required software is installed on all machines in the
Labs. The simulated environment includes four brightly coloured objects hidden in the environment at
increasing difficulty. Your robot starts from a predefined position. You will be provided with a
“training arena” in simulation (a simulation of an indoor environment in which 4 objects will be
“hidden”). This “training arena” will resemble the “test arena” in terms of structure and complexity
(same floor plan of the environment), but the positions of the objects will slightly vary to assess the
generality of your approach.

You may choose any sensors available on the robot to drive your search behaviour. However, your
system design should include the following elements:

1. Perception of the robot’s environment using the Kinect sensor, either in RGB or Depth space,
or using a combination of both RGB and Depth data in order find the object;
2. An implementation of an appropriate control law implementing a search behaviour on the
robot. You may choose to realise this as a simple reactive behaviour or a more complex one,
eg, utilising a previously acquired map of the environment;
3. Motor control of the (simulated) Turtlebot robot using the implemented control law.
The minimum required functionality consists of a simple reactive behaviour, allowing in principle to
find objects. For an average mark the behaviour should be able to successfully find some objects at
unknown locations. Further extensions are possible to improve your mark in this assessment, also to
enable you to find all objects. Possible extensions to the system may include (but are not limited to):
● An enhanced perception system – in-built colour appearance learning, use of additional visual
cues (e.g. edges), combination of RGB and Depth features, etc.;
● Exploiting maps and other structural features in the environment or more clever search
strategies.
● Utilising other existing ROS components that are available (like localisation, mapping, etc)
The software component must be implemented in Python and be supported by use of ROS to
communicate with the robot. 

----------


## Objectives


* [LO3] implement and empirically evaluate intelligent control strategies, by programming
autonomous mobile robots to perform complex tasks in dynamic environments

----------


## Built With





