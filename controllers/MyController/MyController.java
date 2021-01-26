// You may need to add other webots classes such as
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Supervisor;
import com.cyberbotics.webots.controller.Display;
import com.cyberbotics.webots.controller.InertialUnit;
import java.util.ArrayList;
import java.util.*;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.Robot;

// Here is the main class of your controller.
// This class defines how to initialize and how to run your controller.
public class MyController {

  // This is the main function of your controller.
  // It creates an instance of your Robot instance and
  // it uses its function(s).
  // Note that only one instance of Robot should be created in
  // a controller program.
  // The arguments of the main function can be specified by the
  // "controllerArgs" field of the Robot node
  
  private final static double ROBOT_DIAMETER = 71;  // Robot size as defined by the data sheet  = 0.071m
  private final static double WHEEL_RADIUS = 20;    // 0.02m - note that parameters here should be in mm.
  private final static double AXLE_LENGTH = 52;     // 0.052m - note that parameters here should be in mm. 
  private final static double MAX_SPEED = 6.28;     // This is 2\pi radians per sec

// ==========================================================================
  // The following provides data about each of the sensors of the ePuck, as taken
  // from the Webots ePuck Data Sheet.  Note that the sensors are not evenly positioned
  // around the robot.  See: https://cyberbotics.com/doc/guide/epuck?tab-language=java
  // Note that coordinates are given here in mm
  //
  // UPDATE: The sensor positions assume that the robot has a heading of pi/2, and assumes
  // a negative y axis (actually this is z as y refers to the height in the data sheet).
  // The following positions reflect a rotation so that the sensors are positioned relative
  // to a robot with a heading of 0.
  private static String[] psNames = {
        "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"
    };    
  private static double[] psRelX = {
        30.0, 22.0, 0.0, -30.0, -30.0, 0.0, 22.0, 32.0
    };
  private static double[] psRelY = {
        -10.0, -25.0, -31.0, -15.0, 15.0, 31.0, 25.0, 10.0
    };
  private static double[] psRelTheta = {
        -0.301, -0.801, -1.571, 3.639, 2.639, 1.571, 0.799, 0.299
    };
   
// ==========================================================================

  public static double navigateRobot(SensorModel sensorModel, Motor leftMotor, Motor rightMotor, Supervisor robot, Pose myPose, double thetaTarget ){
  //public static double navigateRobot(SensorModel sensorModel, Motor leftMotor, Motor rightMotor, Supervisor robot, Pose myPose, double thetaTarget ){
    // The following code is based on the avoid obsticle code supplied
    // by the Webots platform for the ePuck and allows the robot to wander
    // randomly around the arena
    double[] psValues = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 8 ; i++) {
      psValues[i] = sensorModel.getSensorValue(i);
    }

    /*
    double proxDist = 8.0;    // Distance when robot avoids obstacle
    boolean right_obstacle =
          psValues[0] < proxDist ||
          psValues[1] < proxDist ||
          psValues[2] < proxDist;
    boolean left_obstacle =
          psValues[5] < proxDist ||
          psValues[6] < proxDist ||
          psValues[7] < proxDist;
    */     
      
    // =============================== SENSORS 0 AND 7 ====================================================================    
    
    double proxDist = 8.0;
    boolean right_obstacle =
          psValues[0] < proxDist; 
    boolean left_obstacle =
          psValues[7] < proxDist;       
       
    // ==============================================================
    // The boolean values above are then used to determine if the robot
    // should change direction based on a nearby obstacle
    
    // initialize motor speeds at 40% of MAX_SPEED.
       
    double mySpeed = 0.4;
    double leftSpeed  = mySpeed * MAX_SPEED;
    double rightSpeed = mySpeed * MAX_SPEED;

    double theta = myPose.getTheta(); // theta from odometry
    double turnAngle = theta - thetaTarget;
    
    
    // =================================================================
    // SOLUTION THAT TURNS EXACTLY 90 DEGREES, USES SUPERVISOR, OUTCOMMENT TO MAKE IT WORK 
    //      Please turn the supervisor to 'TRUE' in the SCENE TREE
    // =================================================================
    
    
    Node robotNode = robot.getSelf(); // Get a handle to the Robot Node
    double[] rot = robotNode.getOrientation(); // 3x3 Rotation matrix as vector of length 9 
    // calculate the Theta angle with 3rd column x and z value. See https://en.wikipedia.org/wiki/Atan2
    // for further explanation. We use z instead of y, as we want the rotation around the y axis. 
    // For theta = 0, the robot axis are aligned with the world coordinate axis 
    
    double theta1 = Math.atan2(rot[2], rot[8]); 

    double theta2 = myPose.getTheta(); // theta from odometry
    double turnAngle1 = theta1 - thetaTarget;
    turnAngle = Math.atan2(Math.sin(turnAngle1), Math.cos(turnAngle1)); // normalize angle -pi to pi
     
    if (right_obstacle || left_obstacle) {    
      if ((Math.abs(turnAngle1) < 0.001 && left_obstacle) || (Math.abs(turnAngle1) < 0.001 && right_obstacle)) {
        thetaTarget = theta1 + 1.57079632679;
        thetaTarget = Math.atan2(Math.sin(thetaTarget), Math.cos(thetaTarget)); // normalize angle -pi to pi
        leftSpeed  = mySpeed * MAX_SPEED /2;
        rightSpeed =  -mySpeed * MAX_SPEED /2;
      }    
      
    }     
    if (Math.abs(turnAngle) < 0.001 ){
       //turnAngle = 0;
       System.out.println("Forward");
       leftSpeed  =  mySpeed * MAX_SPEED ;
       rightSpeed =  mySpeed * MAX_SPEED ;       
     }  
    
    else  {
        System.out.println("Turning");
        leftSpeed = leftSpeed * turnAngle;
        rightSpeed = -rightSpeed * turnAngle;  
    }    // 
    
    
    
    // =================================================================
    // SOLUTION THAT TURNS AROUND 85 DEGREES, OUTCOMMENT TO MAKE IT WORK,  
    //               !!! USE ONLY 0 AND 7 SENSORS !!! 
    //  Please change the following: 1. Supervisor robot -> Robot robot in the function definition
    //                               2. Theta target in the main loop to the //double thetaTarget = myPose.getTheta();
    //                                  (instead of 0) 
    //                               3. Supervisor robot = new Supervisor()  -> Robot robot = new Robot()  
    //                               4. Supervisor mode can be turned off.
    // =================================================================
    
    /*
    if (Math.abs(turnAngle) < 0.01 && right_obstacle || left_obstacle) {
      if (left_obstacle) {
        thetaTarget = theta - 1.57079632679;
        leftSpeed  = -mySpeed * MAX_SPEED /2;
        rightSpeed =  mySpeed * MAX_SPEED /2;
      }    
      else if (right_obstacle) {        
        thetaTarget = theta - 1.57079632679;
        leftSpeed  = -mySpeed * MAX_SPEED/2;
        rightSpeed =  mySpeed * MAX_SPEED/2;
      }
    } 
    
    else if (Math.abs(turnAngle) <  0.001 ){
      // turnAngle = 0;
       leftSpeed  =  mySpeed * MAX_SPEED ;
       rightSpeed =  mySpeed * MAX_SPEED ;       
     }  
    
    else  {
        System.out.println("ELSE IS HAPPENING");
        leftSpeed = -leftSpeed * turnAngle;
        rightSpeed = rightSpeed * turnAngle;  
    }    */ 
    
    
    
    // =================================================================
    //  SOLUTION THAT WAS PROVIDED
    // ================================================================= 
    
    /*
    if (left_obstacle) {
      // turn right
      leftSpeed  = mySpeed * MAX_SPEED;
      rightSpeed = -mySpeed * MAX_SPEED;
    }
    else if (right_obstacle) {
      // turn left
      leftSpeed  = -mySpeed * MAX_SPEED;
      rightSpeed = mySpeed * MAX_SPEED;
    }    */
    
    // write actuators inputs
     
    leftMotor.setVelocity(leftSpeed);
    rightMotor.setVelocity(rightSpeed);
    return thetaTarget;
  }

// ==========================================================================
     
  public static void main(String[] args) {
  
    // ======================
    // Section A (see Readme)
    // ======================

    // create the Robot instance.
    //Robot robot = new Robot();
    Supervisor robot = new Supervisor();  // ##################################################################
    
    
    // get the time step of the current world.
    int timeStep = (int) Math.round(robot.getBasicTimeStep());
    // set up other robot-based parameters
    double MAX_SPEED = 6.28;        // This is 2\pi radians per sec
    int step_count = 0;             // We use this to track time in our simulation
    
    // ==============================================================
    // Load map model, motion model and particle filter
    // ==============================================================
    MapModel myMap = new MapModel("defaultConfig.txt", 12, 10);
    System.out.println(myMap.toString());
    
    Pose myPose = myMap.getCenterPose();  // Assume robot is in the center of the map
    
    //double thetaTarget = myPose.getTheta();  //##############################################################################
    double thetaTarget = 0;
    
    MotionModel motionModel = new MotionModel(WHEEL_RADIUS, AXLE_LENGTH);
    
    ParticleFilter myFilter = new ParticleFilter(myMap);
    
    // ===============================================================
   
    // ==============================================================
    // Initialise Devices
    // ==============================================================
    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor motor = robot.getMotor("motorname");
    //  DistanceSensor ds = robot.getDistanceSensor("dsname");
    //  ds.enable(timeStep);

    // ==============================================================
    // Initialise mapDisplay, odometryDisplay and sensorDisplay
    Display odometryDisplay = robot.getDisplay("odometryDisplay");
    motionModel.initialiseDisplay(odometryDisplay);
    
    Display mapDisplay = robot.getDisplay("mapDisplay");
    MapView mapView = new MapView(mapDisplay, myMap, myPose);
    
    Display sensorDisplay = robot.getDisplay("sensorDisplay");
        
    // ==============================================================
    // Initialise proximity sensors
    SensorModel sensorModel = new SensorModel(8);
    for (int i = 0; i < 8; i++) {
      sensorModel.initialiseSensor(i,
                                   robot.getDistanceSensor(psNames[i]),
                                   psNames[i],
                                   new Pose(psRelX[i], psRelY[i], psRelTheta[i]),
                                   timeStep);
    }
    
    sensorModel.initialiseDisplay(sensorDisplay);

    
    // ==============================================================
    // Initialise motors and wheel sensors for odometry
    
    // get a handler to the motors and set target position to infinity (speed control)
    Motor leftMotor = robot.getMotor("left wheel motor");
    Motor rightMotor = robot.getMotor("right wheel motor");
    leftMotor.setPosition(Double.POSITIVE_INFINITY);
    rightMotor.setPosition(Double.POSITIVE_INFINITY);
    leftMotor.setVelocity(0.0);
    rightMotor.setVelocity(0.0);
    
    // get a handler to the position sensors and enable them
    PositionSensor leftPositionSensor = robot.getPositionSensor("left wheel sensor");
    PositionSensor rightPositionSensor = robot.getPositionSensor("right wheel sensor");
    leftPositionSensor.enable(timeStep);    // sampling period
    rightPositionSensor.enable(timeStep);   // sampling period
    
    // Initialise the odometry of the motion model
    // motionModel.initialiseOdometry(myPose, leftPositionSensor, rightPositionSensor);
    motionModel.initialiseOdometry(myPose, leftPositionSensor.getValue(),
                                           rightPositionSensor.getValue());


    // ==============================================================
    // Main loop:
    // ==============================================================
    // - perform simulation steps until Webots is stopping the controller

    while (robot.step(timeStep) != -1) {
      step_count++;      // increment step count
      

      // ======================
      // Section B (see Readme)
      // ======================
      
      thetaTarget = navigateRobot(sensorModel, leftMotor, rightMotor, robot, myPose, thetaTarget);
      
      // ======================
      // Section C (see Readme)
      // ======================
      // compute odometry and speed values
      myPose = motionModel.updateOdometry(leftPositionSensor.getValue(), rightPositionSensor.getValue());
      mapView.setPose(myPose);
      
      // ==============================================================
      // Update the particle filter
        
      // 1) Draw new samples from distribution given weights
      myFilter.resampleParticles();
       
      myFilter.resetCachedParticleset();   
           
      double eta=0.0;    // Normaliser
          
      // 2) Iterate through each of the newly sampled particles
      for (Particle p:myFilter.getParticleSet()) {
        // 2.1) Apply the motion model to the particle and add to the cache
        Particle pp = motionModel.sample_motion_model(p); 
          
        
        // We can test to see if the particle moves into an obstacle
        // if this is the case, then we replace with new random particles
        if(myMap.get_nearest_map_feature_dist(pp.getX(), pp.getY()) <= ROBOT_DIAMETER/2.0) {
            
          // Replace with a random particle
          pp = myFilter.generateRandomParticle(ROBOT_DIAMETER/2.0,
                           myMap.getCellWidth() * myMap.getMapWidthInCells(),
                           myMap.getCellHeight()* myMap.getMapHeightInCells());
                           
         }
          	   
        // 3) Use the sensor model to update the particle weight
        double w = sensorModel.likelihood_field_range_finder_model(pp, myMap); 	   
        pp.setWeight(w);   // update the particle with the weight based on the sensor model
        myFilter.addToCachedParticleSet(pp);
    	   
        eta+=w;          // add the weight to the normaliser
      }
      // 5) Finally replace the particles in the filter with those in the cache
      myFilter.update_particleset_with_cached(eta);        

      // ======================
      // Section D (see Readme)
      // ======================

      // Update the different displays
      mapView.paintView();
      mapView.paintParticles(myFilter.getParticleSet());       
      sensorModel.paintSensorDisplay();
      
    };
    // Enter here exit cleanup code.
  }
}