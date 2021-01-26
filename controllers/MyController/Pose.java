// File: Pose.java
// Date: 30th Dec 2020
// Description: Pose Class  support for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:
/**
 * The Pose class for a robot 
 * Based on Worksheet 3 for COMP329, Nov 2020
 * 
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */
 
 public class Pose {
  private double x;    // position on x axis - assume units are meters
  private double y;    // position on y axis - assume units are meters
  private double theta;  // This determines the angle (radians) anticlockwise from the x-axis line

  public Pose() {
    this(0.0,0.0,0.0);
  }

  public Pose(double xpos, double ypos, double theta) {
    this.x = xpos;
    this.y = ypos;
    this.setTheta(theta);
  }
  
  public Pose(Pose p) {
    this.x = p.getX();
    this.y = p.getY();
    this.setTheta(p.getTheta());
  }


  public void setTheta(double theta) {
    // Ensure that theta is in the range 0..2\pi
    Boolean neg=(theta<0);

    theta = Math.abs(theta);
    while (theta >= (2 * Math.PI))
      theta -= (2 * Math.PI);

    // Now replace the sign
    if (neg)
      this.theta = - theta;
    else
      this.theta = theta;
  }

  public void setPosition(double xpos, double ypos, double theta) {
    this.x = xpos;
    this.y = ypos;
    this.setTheta(theta);
  }
  
  public void setPosition(Pose p) {
    this.setPosition(p.getX(), p.getY(), p.getTheta());
  }


  public String toString() {
    return String.format("(%.03f", this.x) + ", " +
        String.format("%.03f", this.y) + ", " +
        String.format("%.03f", this.theta) +")";
        // + " heading (in degrees) is " + String.format("%.03f", Math.toDegrees(this.theta));
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getTheta() {
    return theta;
  }

}