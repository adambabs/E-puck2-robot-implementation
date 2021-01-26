import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;
import java.util.ArrayList;
import java.util.*;

// ==============================================================
// COMP329 2020 Programming Assignment
// ==============================================================
// You will need to update the method
// * public double likelihood_field_range_finder_model(Particle xt, MapModel map)
// to return the correct probability.
// ==============================================================


public class SensorModel {
  private Sensor sensorArr[];    // Array of sensors
  private int numSensors;        // number of sensors (I.E. size of sensorArr)

  private Display display;       // Reference to the display device for debugging
 
  private final static int BLACK = 0x000000;
  private final static int WHITE = 0xFFFFFF;
  
  
  // Constructor - assumes that the sensor has been created and enabled by the caller
  public SensorModel(int numSensors) {
    this.numSensors = numSensors;
    this.sensorArr = new Sensor[this.numSensors];
    this.display = null;
    
  }
  
  // ==============================================================
  // This class breaks the convention of having a view for each output device
  // by handing output directly from the class itself.  This may be fixed in
  // the future, but for now the device supports debugging.
  public void initialiseDisplay(Display sensorDisplay) {
    this.display = sensorDisplay;
    }

  
  // This adds and initialises sensors to the model
  public void initialiseSensor(int sensorID, DistanceSensor ps, String name, Pose p, int samplingPeriod) {
    if (sensorID >= numSensors)
      System.out.println("Illegal sensor ID (id="+sensorID+") initialised in SensorModel");
    this.sensorArr[sensorID] = new Sensor(ps, name, p, samplingPeriod);
    System.out.println("Sensor Device "+name+
                       " with ID:"+sensorID+
                       " at pose: " + p.toString());
  }
  
  
  // Returns the sensor value for the sensor sensorID
  public double getSensorValue(int sensorID) {
    if (sensorID >= numSensors)
      System.out.println("Illegal sensor ID (id="+sensorID+") sampled in SensorModel");
    return this.sensorArr[sensorID].getSensorValue();
  }
  
  // ==============================================================
  // Based on the algorithm given in the notes on Sensors and Perception
	
  public double likelihood_field_range_finder_model(Particle xt, MapModel map) {
    double q = 1.0;
    double p;
    Sensor zk;   // one of the numSensors sensors
    double dist; // distance to nearest object from the location of the artifact sensed by zk
    double z;    // sensor value for sensor zk
    

    for (int i=0; i<this.numSensors; i++) {
      zk = this.sensorArr[i];
      z = zk.getSensorValue();
        if (z < zk.getMaxRange()) {
          
          double x = xt.getX() + 
            zk.getLocalPose().getX() * Math.cos(xt.getTheta()) -
            zk.getLocalPose().getY() * Math.sin(xt.getTheta()) +
            z * Math.cos(xt.getTheta()+zk.getLocalPose().getTheta());
          
          double y = xt.getY() +  
            zk.getLocalPose().getY() * Math.cos(xt.getTheta()) +
            zk.getLocalPose().getX() * Math.sin(xt.getTheta()) +
            z * Math.sin(xt.getTheta() + zk.getLocalPose().getTheta());
          
       dist = map.get_nearest_map_feature_dist(x,y);				
       p = zk.getProbability(dist);
       q = q * p;
      
       }
     }        

      // =================================
      // ...
      // ...
      // Insert Code Here
      // ...
      // ... 
      // Determine the probability of observing obstacles by implementing
      // the likelihood_field_range_finder_model as discussed in the
      // lecture notes.  The distance to the neareat obstacle can
      // be determined using:
      //
      // * map.get_nearest_map_feature_dist(x,y);
      //
      // and the probability of the resulting distance given the sensor can
      // be computed using:
      //
      // * zk.getProbability(dist);
      // 
      // Use these to update the return probability q
      // ================================= 
    return q;
  }

  // ==============================================================
  // Update the Sensor Display (primarily used for debugging)
  // Note that this ought to be handled as a separate view class!!! 
  public void paintSensorDisplay() {
    if (this.display!=null) {
      // Clear display
      this.display.setColor(WHITE);     // White
      this.display.fillRectangle(0,0,this.display.getWidth(),this.display.getHeight());
      
      this.display.setColor(BLACK);     // Black
      this.display.setFont("Arial", 16, true);  // font size = 18, with antialiasing
      this.display.drawText("Sensor Information", 1, 1);

      this.display.setFont("Arial", 9, true);  // font size = 12, with antialiasing

      this.display.drawText(this.sensorArr[0].getSensorName(), 110,30);
      this.display.drawText(String.format("%.02f", this.sensorArr[0].getSensorValue()), 105,46);

      this.display.drawText(this.sensorArr[1].getSensorName(), 140,55);
      this.display.drawText(String.format("%.02f", this.sensorArr[1].getSensorValue()), 135,71);

      this.display.drawText(this.sensorArr[2].getSensorName(), 165,110);
      this.display.drawText(String.format("%.02f", this.sensorArr[2].getSensorValue()), 160,126);

      this.display.drawText(this.sensorArr[3].getSensorName(), 125,170);
      this.display.drawText(String.format("%.02f", this.sensorArr[3].getSensorValue()), 120,186);
 
      this.display.drawText(this.sensorArr[4].getSensorName(), 55,170);
      this.display.drawText(String.format("%.02f", this.sensorArr[4].getSensorValue()), 50,186);

      this.display.drawText(this.sensorArr[5].getSensorName(), 15,110);
      this.display.drawText(String.format("%.02f", this.sensorArr[5].getSensorValue()), 10,126);
   
      this.display.drawText(this.sensorArr[6].getSensorName(), 40,55);
      this.display.drawText(String.format("%.02f", this.sensorArr[6].getSensorValue()), 35,71);

      this.display.drawText(this.sensorArr[7].getSensorName(), 70,30);
      this.display.drawText(String.format("%.02f", this.sensorArr[7].getSensorValue()), 65,46);
     
      // ------------------------
      // Draw Robot Body          
      this.display.setColor(0x3C3C3C);     // Dark Grey
      this.display.drawOval(100,115,40,40);
      this.display.drawLine(100,115,100,75);

    }
  }
}