// File: Particle.java
// Date: 1st Jan 2021
// Description: Particle class for the Particle Filter used in the COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:
/**
 * This is based on the Pose class used in COMP329 on
 *
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */

public class Particle {
  private double x;    // position on x axis 
  private double y;    // position on y axis
  private double theta;  // This determines the angle (radians) anticlockwise from the x-axis line
  private double weight;  // We'll attach the weight here of the particle, though it will be set through accessors

  public Particle() {
    // TODO Auto-generated constructor stub
    this(0.0,0.0,0.0,0.0);

  }
  
  public Particle(double xpos, double ypos, double theta, double weight) {
    this.x = xpos;
    this.y = ypos;
    this.setTheta(theta);
    this.weight = weight;
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
  
  public void setWeight(double weight) {
    this.weight=weight;
  }

  
  public String toString() {
    return String.format("(%.03f", this.x) + ", " + 
        String.format("%.03f", this.y) + ", " +
        String.format("%.03f", this.theta) +")" +
        " heading (in degrees) is " + String.format("%.03f", Math.toDegrees(this.theta));      
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

  public double getWeight() {
    return weight;
  }
}
