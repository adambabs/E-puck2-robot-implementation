README
This file provides details on all of the java classes and indicates which
should be modified for the assignment.  It is organised into different sections
based on the organisation of the robot.  Files that you are expected to update
are as follows:
  * MyController.java
  * MotionModel.java
  * SensorModel.java
  * ParticleFilter.java

------------------------
WeBots Robot Controller
------------------------

MyController.java
=================
This is the main controller for the puck and is responsible for the main actions of
the robot.  There are essentially four parts:
  * A: Initialisation (up to the main loop) responsible for constructing the main instances
  * B: Navigation using a simple obstacle avoidance model
  * C: The particle filter which determines the odometry of the robot
  * D: Update the display

Note that the ePuck that is controlled by MyController has three "displays" that are mounted on the ePuck turret.  These will not physically appear on the rendered robot, but rather can be viewed as windows within the simulator (See Display Devices listed under the Overlays menu item).

The navigation of the robot is handled by the method 
  * public static void navigateRobot(SensorModel sensorModel, Motor leftMotor, Motor rightMotor)
Which adjusts the motors based on sensor readings around the robot itself.

    --------------------------------------
    | This code will need to be modified |
    --------------------------------------
    
More details are given in the assignment text.

------------------------------------
Modelling and Visualising the World
------------------------------------

defaultConfig.txt
=================
This is a representation of the map as a set of cells.  It is used by the robot to model
the world represented by WeBots.  Note that it assumes that the bottom most cell is indexed
from 1,1, as a wall is automatically added by the map modeller at cell 0,0 and surrounds the
full perimeter.  Also note that the cell 1,1 corresponds to the bottom left cell
(i.e. assuming a standard coordinate system, rather than an  inverted one used when plotting
to a display device).  Cells are assumed to be 100mm square.

* You can modify this to reflect any changes you make to the Webots world.
 

MapModel.java
=============
This code is responsible for modelling the map, and reads in the file "defaultConfig.txt" to
build the model.  It supports the notion of coloured cells (as used in the COMP329 robotics lab)
although these are not used in the COMP329 2020 programming assignment.  The one method that will
be of interest is:
  * double get_nearest_map_feature_dist(double x, double y) 
  This determines the distance to the nearest map feature of a point at position x,y.  As the
  features are axis-aligned rectangles, it determines the distance to one of the sides.  However,
  a distance of 0.0 may be returned if the point lies inside the obstacle.  Of course, this could
  be due to specular error.  A current fix has been implemented to only return distances > 1.0.

* This code can be inspected but should not be updated!


Cell.java
=========
The MapModel build up a data structure of Cell instances.  Each cell instance represents a
possible occupancy, as well as determining if there is a coloured cell. Each cell is characterised using the enum CellOccupancyType.

* This code can be inspected but should not be updated!


CellOccupancyType.java
======================
This is simply an enum defining the different cell occupancy types.

* This code can be inspected but should not be updated!


MapView.java
============
The ePuck has been modified to carry three displays on the turret (this can be inspected within
the webots platform).  One of these displays is the mapDisplay.  This visualises the map, the
location of the robot (as perceived by the robot) and particles resulting from the particle filter.
The main loop in MyController.java calls the main method in this class (paintView) to redraw the
Map and robot.  It also includes a method (paintParticles) to display the particles when the Particle
Filter is visualise.

Note that graphical displays assume an indexing schema where the y axis is inverted (i.e. (0,0)
is located in the top left of the display.  The code in this class takes this into account, so
that the position of the robot and the obstacles are drawn with the assumption of (0,0) in the
bottom left of the display.

* This code can be inspected but should not be updated!

------------------------
Odometry and Locomotion 
------------------------

MotionModel.java
================
This code is responsible for tracking the odometry of the robot around the map.  It models the robot's locomotion, and calculates the odometry of a differential drive based on wheel encoders (the number
of rotations are obtained from a position sensor for each wheel.  Some basic code has been written,
but the pose of the robot is not updated, and valid telemetry is not displayed on the Odometry Display 

You will need to update the method
 * public Pose updateOdometry(double leftPositionSensorValue,
                              double rightPositionSensorValue)

to correctly model the odometry based on the wheel sensors.

Remember that even if the odometry update works, the robot that appears on the map Display will
accumulate localisation errors.

    --------------------------------------
    | This code will need to be modified |
    --------------------------------------
    
More details are given in the assignment text.

Pose.java
=========
This class, used extensively in the COMP329 Worksheets, models the pose of an artifact, which
has a x and y coordinate and a heading (theta) which is stored as radians.  A pose could
represent the centre of the robot, a particle, a sensor etc.

--------
Sensors 
--------

SensorModel.java
================
The sensor model is responsible for managing the use of sensors on the robot.  It also visualises
their use through a display mounted on the ePuck turret.  There are 8 IR sensors on the ePuck,
though not evenly spaced out.  Their range is also severely limited.  These appear (labelled)
in the sensorDisplay, which is painted through a call to paintSensorDisplay() from the main
loop in MyController.

The location of the 8 sensors is given in the ePuck data sheet, assuming that the robot has a
heading of pi/2.  These coordinates have been updated to correctly position the sensors on the
modelled robot (i.e. each sensor has a pose using the robot's local coordinate system).

The method double likelihood_field_range_finder_model(Particle xt, MapModel map) is responsible
for generating a probability based on the sensors of the robot.

    --------------------------------------
    | This code will need to be modified |
    --------------------------------------
    
More details are given in the assignment text.


Sensor.java
===========
Each sensor is modelled as an instance of the class Sensor.  It has a pose which is given
relative to the robot coordinate system, as well as a reference to a DistanceSensor that
returns the distance value.  The distance value returned is inversely (and non-linearly)
proportional to the proximity of an object (i.e. the closer you are, the higher the number).
Therefore the method getSensorValue() uses a crude set of rules to map the returned values
into a distance value in mm.  If the max range is set to 19mm, then the results increase
in noise, so it has been set to 18mm for now.  Weights for the probability values, as well
as the standard deviation for the noise of the sensor itself have been included, but these
intrinsic values are arbitrary, and can be changed. 

* This code can be inspected but should not be updated.  However, you may want to change the
  intrinsic parameters at the top of the file.

-------------------------------------
Localisation and the Particle Filter 
-------------------------------------

ParticleFilter.java
===================
This class models a particle filter used to localise the robot itself.  It assumes two arrays to manage the particles; one (ParticleSet) holds the particles themselves, and a second one (cachedParticleSet) build the new set of particles for each update.  Note that the particle set is not *strictly* a set as a single particle may be sampled multiple times using the Monte-Carlo sampler.

The monte-carol sampler is based on the stochastic_universal_sampling algorithm as discussed in the notes and should be completed.

    --------------------------------------
    | This code will need to be modified |
    --------------------------------------
    
More details are given in the assignment text.

Currently it is incomplete (even when the Monte Carlo sampler is working) as the particles do not
always converge on the actual location of the robot. This could be for a variety of reasons.  You
are not expected to debug this or complete the work, but rather look at the code and argue reasons
why it may not converge on the robot location.  Observe the behaviour of the particles, and by
inspecting the code, provide justification for your reasoning.

Particle.java
=============
Each particle is modelled using the Particle class.  A particle is similar to a pose, but also
includes a weight.


