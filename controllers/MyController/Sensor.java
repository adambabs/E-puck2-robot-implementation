// File: Sensor.java
// Date: 3rd Jan 2021
// Description: Sensor Class for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:
/**
 * This defines a sensor used as part of the sensor model.
 *
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */

import com.cyberbotics.webots.controller.DistanceSensor;

// Each sensor is modelled assuming a relative position on the robot, where the robot
// center (i.e. its pose) is positioned at the origin, with a heading of 0.0 radians
 
public class Sensor {
  private Pose localPose;    // relative position of the sensor with respect to the robot pose
  private String sensorName; // name of the sensor as used by the robot
  private DistanceSensor ps; // the distance sensor handler
  private double maxRange;   // maximum range as returned by the sensor
  
  private final double MAXRANGE = 18.0; // Maximum range of the sensor in mm (valid between 13..19

  private final double WEIGHT_RANDOM = 0.1;
  private final double WEIGHT_MAXRANGE = 0.3;
  private final double WEIGHT_HIT = 0.6;
  private final double SIGMA_HIT = 3.0;
    
  // Constructor - assumes that the sensor has been created by the caller
  public Sensor(DistanceSensor ps, String name, Pose p, int samplingPeriod) {
    this.ps = ps;
    this.sensorName = name;
    this.localPose = p;
    this.ps.enable(samplingPeriod);
  }
  
  // ==============================================================
  // Returns the sensor value for the sensor sensorID
  // The ePuck model generates a non-linear function which grows following an
  // exponential (like) function as the proximity decreases.  This has been
  // simulated using the following rules:
  //     r in [72..120]    d=28-(r/8)
  //     r in [120..170]   d=25-(r/10)
  //     r in [170..210]   d=16.5-(r/20)
  //     r in [210..420]   d=12-(r/35)
  //
  // Assume that min range is 0mm (i.e. r >410) but max range is 20mm (i.e. r<64)
  
  public double getSensorValue() {
    double r = this.ps.getValue();
    double d = MAXRANGE;
    
    // The following is crude, but permits the max range to be varied
    // between 13..19
    if (r < 8*(28-MAXRANGE))      
      d = MAXRANGE;
    else if (r < 120.0)
      d = 28.0 - (r/8.0);
    else if (r < 170.0)
      d = 25.0 - (r/10.0);
    else if (r < 210.0)
      d = 16.5 - (r/20.0);
    else d = 12.0 - (r/35.0);
    
    return Math.max(d,0.0);
  }
  
  // ==============================================================
  // Get the max range of this sensor
  public double getMaxRange() {
    return MAXRANGE;
  }
  
  public Pose getLocalPose() {
    return this.localPose;
  }

  // ==============================================================
  // Returns the sensor value for the sensor sensorID
  public String getSensorName() {
    return this.sensorName;  
  }

  // Sample probability on a zero centered gaussian,
  // where a is the query and b is the standard deviation
  double prob_normal_distribution(double a, double b) {
    double coeff=1.0/(Math.sqrt(2*Math.PI*b*b));
    double power=(-0.5*a*a)/(b*b);
    double result=coeff * Math.exp(power);
	
    /*	
    System.out.println(String.format("Sample Gaussian at a=%.03f, b=%.03f", a,b));
    System.out.println(String.format(
                    "Gaussian coefficient is %.03f and power is %.03f resulting in %.03f",
                    coeff, power, result));
     */
    return result;
  }

  
  public double getProbability(double dist) {
    
    /*
    System.out.println(String.format(
          "Probability on dist: %.03f gives %.06f + rand/max: %.06f",
          dist, this.prob_normal_distribution(dist, SIGMA_HIT), WEIGHT_RANDOM/MAXRANGE));
    */	
    return WEIGHT_HIT * this.prob_normal_distribution(dist, SIGMA_HIT) + WEIGHT_RANDOM/MAXRANGE;
  }
}