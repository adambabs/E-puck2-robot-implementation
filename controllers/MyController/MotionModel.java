import java.util.Random;
import com.cyberbotics.webots.controller.Display;
 
public class MotionModel {

  // ===================================================
  // Instance Variables
  private final int NUM_NOISE_COEFFS = 6;
  private double[] a = new double[NUM_NOISE_COEFFS];
  private Random rand;
  
  private Pose odometryPose;     // Current odometry pose measured for the robot
  
  private double wheelRadius;    // Radius of the wheel - used to estimate distance travelled
  private double axelLength;     // The axel distance beteen the two wheels of a differential drive
  private double leftPosSample;  // The current sample of the left postion sensor value 
  private double rightPosSample; // The current sample of the right postion sensor value 

  private Display display;       // Reference to the display device for debugging
  
  // The followng variables store the relative odometry calculated in the method updateOdometry()
  private double d_rot1;         // Odometry component for initial rotation to current pose
  private double d_trans;        // Odometry component for translation to current pose
  private double d_rot2;         // Odometry component for final rotation to current pose
  
  private double stepcount;

  private final double[] NOISE_PROFILE = {0.005, 0.005, 0.001, 0.001, 0.005, 0.005}; // Odometry noise profile


  private final int BLACK = 0x000000;
  private final int WHITE = 0xFFFFFF;


  // ==============================================================
  // Constructor

  public MotionModel(double wheelRadius, double axelLength) {
    this.rand = new Random();
    this.odometryPose = new Pose();  // set a default pose - should be initialised later!
    this.wheelRadius = wheelRadius;
    this.axelLength = axelLength;
    this.display = null;
    this.stepcount = 0;
    double a[] = NOISE_PROFILE;       // Admittedly this could be done better!
     
    // Constructor takes noise profile
    for (int i=0;i<NUM_NOISE_COEFFS;i++)
      this.a[i]=a[i];   
      
    this.leftPosSample = 0.0;    // Initial (default) values
    this.rightPosSample = 0.0;   // Initial (default) values  
    this.d_rot1 = 0.0;           // Initial (default) values
    this.d_trans = 0.0;          // Initial (default) values
    this.d_rot2 = 0.0;           // Initial (default) values
  }

  // This class breaks the convention of having a view for each output device
  // by handing output directly from the class itself.  This may be fixed in
  // the future, but for now the device supports debugging.
  public void initialiseDisplay(Display odometryDisplay) {
    this.display = odometryDisplay;
  }

  // ==============================================================
  // Distribution Sampler

  public double sample_normal_distribution(double b) {
    // Note that we are assuming that b here is the 
    // standard deviation, so if a query is required 
    // given a variance, then the caller needs to pass
    // the root!
                
    double a=0;    // our accumulator   
    double t=b*2;  // probably unnecessary, but it is a constant.
                
    // Iterate 12 times summing random values between -b..b
    for (int i=0; i<12; i++)
      a+=(rand.nextDouble()*t-b);
                
    return (a/2.0);
  }     

  // ==============================================================
  // Odometry management
  public void initialiseOdometry(Pose p, double leftPositionSensorValue, 
                                         double rightPositionSensorValue) {
    // Set the current values of the position sensors, and associate with a pose.
    // Required before the first time updateOdometry is called if the correct position 
    // is to be determined 
   
    this.odometryPose = new Pose(p);
    this.leftPosSample = leftPositionSensorValue;
    this.rightPosSample = rightPositionSensorValue;
    
    // Check if the values are NAN - if so, then set to 0.0
    if (Double.isNaN(this.leftPosSample))
      this.leftPosSample=0.0;
    if (Double.isNaN(this.rightPosSample))
      this.rightPosSample=0.0;
  }

  public Pose updateOdometry(double leftPositionSensorValue, double rightPositionSensorValue) {
    // Given two position sensors, it determines the movement of the wheels,
    // which can be used to measure the relative changes in odometry
    
    // The algorithm here relies on determining the ICR (and distance to it) using the left
    // and right velocities, based on the notes on Locomotion and Kinematics
    // It also calculates the relative odometry between the current and new pose and stores
    // these as the instance variables d_rot1, d_trans and d_rot2, for later use by the
    // particle filter
    
    Pose oldPose = new Pose(this.odometryPose); // Take a copy of the old pose

    double l = leftPositionSensorValue;
    double r = rightPositionSensorValue;
    
    // Determine difference in distance travelled to previous sample
    
    double dl = (l-this.leftPosSample) * this.wheelRadius;
    double dr = (r-this.rightPosSample) * this.wheelRadius;

    // Declare other variables used in managing the odometry.axelLength
    // Not all may be relevant when updating the position, but can be displayed in the odometry display
    
    // Transient movement details
    double thetaChange = (dr - dl) / this.axelLength;
       
    double velocity = (dl + dr) / 2;                        // forward velocity 
    double omega = (dr - dl) / this.axelLength;             // angular speed
    

    double icr_r = 0.0;       // ICR distance (R)
    double icr_x = 0.0;       // ICR x coordinate (x)
    double icr_y = 0.0;       // ICR y coordinate (y)
    

      // =================================
      // ...
      // ...
      // Insert Code Here
      // ...
      // ...
      // Find the pose of the robot at each time interval 
      // (i.e. generate a new pose based on the old pose and change in odometry) 
      // by determining the forward and angular velocity of the robot based
      // on the changes over time of the wheel encoders.  
      
      // Use the values dl and dr for the velocities of the left and right wheels respectively.
      
      // To determine the rotation, you will need to find the ICR.
      //
      // Finally, calculate the change in pose and store in the following instance
      // variables:
      // * this.d_rot1
      // * this.d_trans 
      // * this.d_rot2
      //
      // These will be needed by the sample_motion_model() method in the particle
      // filter.
      //
      // Note that println statements have been included below to indicate the type
      // of movement that the robot does.  You should comment these out in your
      // final solution.  The values you create (see variables above) will be
      // visualised in the odometry display (see the bottom of this method).  You
      // can use this to support debugging. 
      // =================================

    // ==============================================================
    // Determine the new pose based on the wheel odometry position changes

      double lVel = dl;  // left wheel velocity 
      double rVel = dr;  // right wheel velocity 
      
      double x_pos= this.odometryPose.getX();
      double y_pos= this.odometryPose.getY();
          
      double theta_new = odometryPose.getTheta() + omega; // new  Orientation angle    
          
    if (dl == dr) {
      // If both distances are the same, then angular velocity is 0
      

      if (dl == 0.0) {
        System.out.println("No movement");
      } else {
      // INSERT CODE TO HANDLE FORWARD MOVEMENT
     
        y_pos =  y_pos + dl * Math.sin(theta_new);
        x_pos = x_pos + dl * Math.cos(theta_new);
       
        odometryPose.setPosition(x_pos, y_pos, odometryPose.getTheta());
        
        System.out.println("Move forward by "+dl);               
      }
    } else {

      // INSERT CODE TO HANDLE ROTATIONAL MOVEMENT

      icr_r = (axelLength/ 2) * ((lVel + rVel) / (rVel -lVel));  ;
      icr_x = oldPose.getX() - icr_r *  Math.sin(oldPose.getTheta());
      icr_y = oldPose.getY() + icr_r * Math.cos(oldPose.getTheta());
      
      
      x_pos = ((Math.cos(omega) * (oldPose.getX() - icr_x)) - (Math.sin(omega) * (oldPose.getY() - icr_y))) + icr_x;
      y_pos = ((Math.sin(omega) * (oldPose.getX() - icr_x)) + (Math.cos(omega) * (oldPose.getY() - icr_y))) + icr_y;
        
      odometryPose.setPosition(x_pos, y_pos, odometryPose.getTheta());

      System.out.println("Rotational movement");
    
    }

    // ==============================================================
    // Now we can calculate the relative odometry between the new pose in this.odometryPose
    // and the previous pose stored in oldPose

    // INSERT CODE TO CALCULATE THE CHANGE IN ODOMETRY

    double theta = odometryPose.getTheta() + omega; // new  Orientation angle
    this.odometryPose.setTheta(theta); // update the Pose with theta
    
    double changeOdometryX = odometryPose.getX() - oldPose.getX();
    double changeOdometryY = odometryPose.getY() - oldPose.getY();
    
    // this.d_rot1 = ???
    // this.d_trans = ???
    // this.d_rot2 = ???
     
    this.d_rot1 = Math.atan2(changeOdometryY, changeOdometryX) - oldPose.getTheta(); 
    this.d_rot2 = oldPose.getTheta() - odometryPose.getTheta() - d_rot1;    
    this.d_trans =  Math.sqrt(Math.pow(changeOdometryX,2) + Math.pow(changeOdometryY,2));
      
    // ==============================================================
    // Update the Odometry Display (primarily used for debugging)
    // Note that this ought to be handled as a separate view class!!!
    if (this.display!=null) {
      // Clear display
      this.display.setColor(WHITE);     // White
      this.display.fillRectangle(0,0,this.display.getWidth(),this.display.getHeight());
      
      this.display.setColor(BLACK);     // Black
      this.display.setFont("Arial", 18, true);  // font size = 18, with antialiasing
      this.display.drawText("Odometry Information", 1, 1);

      this.display.setFont("Arial", 12, true);  // font size = 12, with antialiasing
      this.display.drawText("Pose: "+this.odometryPose.toString(), 1, 30);
      this.display.drawText("Forward Velocity: "+String.format("%.03f", velocity), 1, 50);
      this.display.drawText("Angular Velocity: "+String.format("%.03f", omega), 1, 70);
      this.display.drawText("ICR: "+String.format("R=%.03f", icr_r), 1, 110);
      this.display.drawText("     "+String.format("x=%.03f", icr_x), 1, 130);
      this.display.drawText("     "+String.format("y=%.03f", icr_y), 1, 150);
      this.display.drawText("d_rot1:"+String.format("%.03f", d_rot1), 1, 190);
      this.display.drawText("d_trans:"+String.format("%.03f", d_trans), 1, 210);
      this.display.drawText("d_rot2:"+String.format("%.03f", d_rot2), 1, 230);

    }
        
    // Save the new sampled values
    this.leftPosSample = l;
    this.rightPosSample = r;

    // return a new version of the pose
    return this.getOdometryPose();
  }

  
  // Return the current odometryPose.  A copy of the internal odometry pose is returned.
  public Pose getOdometryPose() {
    return new Pose(this.odometryPose);
  }
  
  
  // ==============================================================
  // Generate a new sample particle based on the current instance variables
  // d_rot1, d_trans, d_rot2     
  public Particle sample_motion_model(Particle p) {
    double dp_rot1 = this.d_rot1 + this.sample_normal_distribution(
                                   (a[0]*Math.abs(this.d_rot1))+
                                   (a[1]*Math.abs(this.d_trans)));
    double dp_trans = this.d_trans + this.sample_normal_distribution(
                                   (a[2]*Math.abs(this.d_trans))+
                                   (a[3]*(Math.abs(this.d_rot1)+Math.abs(this.d_rot2))));
    double dp_rot2 = this.d_rot2 + this.sample_normal_distribution(
                                   (a[4]*Math.abs(this.d_rot2))+
                                   (a[5]*Math.abs(this.d_trans)));
                
    Particle pp = new Particle( 
        p.getX() + dp_trans * Math.cos(p.getTheta()+dp_rot1),  // x position
        p.getY() + dp_trans * Math.sin(p.getTheta()+dp_rot1),  // y position
        p.getTheta() + dp_rot1 + dp_rot2,                      // theta (heading)
        0.0);                                                  // weight
        

    return pp;  
  }     

}