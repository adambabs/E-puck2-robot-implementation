// File: MapView.java
// Date: 29th Dec 2020
// Description: View for the display device on the ePuck for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:


/**
 * This class displays the map model that is used to model the world as a set of
 * constant sized cells.  Ideally this should have been a subclass of the Display
 * class, but it is easier for this to take a reference to a Display object and
 * use this to render the graphic.
 * 
 * Originally based on the Robosim code for COMP329, Dec 2016.
 *
 * In addition to displaying the map, the class should also show the location
 * of particles with a particle field, and the estimated robot position.  Note
 * also that the size of the display is defined by the robot device definition,
 * and thus any graphic will have to be autromatically scaled.
 * 
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */
import java.util.ArrayList;
import com.cyberbotics.webots.controller.Display;

public class MapView {

  // ===================================================
  // Instance Variables
  private Display display;    // reference to the display device on the robot
  private MapModel map;       // reference to the map model
  private Pose robotPose;     // Pose of the robot
  
  private int deviceWidth;    // width of the display device
  private int deviceHeight;   // height of the display device
  
  private int cellWidth;      // width of a single cell rendered on the display device
  private int cellHeight;     // height of a single cell rendered on the display device
  
  private int mapWidth;       // width of the rendered map on the display device
  private int mapHeight;      // height of the rendered map on the display device

  private int numCellsInRow;	 // number of cells across
  private int numCellsInCol;	 // number of cells down
  
  private double scaleFactor; // Scale factor to scale rendered map to the maximal dimension on the display
	
  // Will we need to store a scale factor for each dimension? 
  private final static int FRAME_BORDER_WIDTH = 5;  // Border Width
  private final static int CELL_LINE_WIDTH = 1;     // Change if the cell lines aren't rendered properly
  private final static int RADIUS_ROBOTBODY = 36;	// The robot is modeled as a circle with this radius in mm.


  // Colours used by the display
  private final static int GREY = 0x787878;
  private final static int RED = 0xFF0000;
  private final static int GREEN = 0x00FF00;
  private final static int BLUE = 0x0000FF;
  private final static int BLACK = 0x000000;
  private final static int WHITE = 0xFFFFFF;
  

  public MapView(Display d, MapModel m, Pose p) {
    this.display = d;
    this.map = m;
    this.robotPose = new Pose(p);
    
    //this.robotPose = new Pose(m.getCenterPose());  // Create a pose, assuming the robot is in the center of the map

    this.deviceWidth = display.getWidth();
    this.deviceHeight = display.getHeight();	
    
    // Instance Variables inc local values
    this.numCellsInRow = this.map.getMapWidthInCells();		      // number of cells across
    this.numCellsInCol = this.map.getMapHeightInCells();		      // number of cells down
    
    // Determine the rendering scale factor
    double wsf = (double) (this.deviceWidth - 2 * FRAME_BORDER_WIDTH) /
                    (double) (this.map.getCellWidth() * this.numCellsInRow);
    double hsf = (double) (this.deviceHeight - 2 * FRAME_BORDER_WIDTH) / 
                    (double) (this.map.getCellHeight() * this.numCellsInCol);
    this.scaleFactor = Math.min(wsf, hsf);
    
    this.cellWidth = (int) (this.map.getCellWidth() * this.scaleFactor); // width of a single rendered cell
    this.cellHeight = (int) (this.map.getCellHeight() * this.scaleFactor);// height of a single rendered cell
    this.mapWidth = this.numCellsInRow * cellWidth;
    this.mapHeight = this.numCellsInCol * cellHeight;
    
  }
  
  public void setPose(Pose p) {
    // Sets the pose of the robot.  If an instance of a pose has yet to be created, then create one
    // Always copy the pose value rather than retain the instance, to ensure it is not side effected
    this.robotPose.setPosition(p);
  }
  
  public void paintView() {
  
    // ===================================================================================
    // draw a background

    this.display.setColor(0xF0F0F0);     // Off White
    //this.display.fillRectangle(0,0,this.deviceWidth, this.deviceHeight);
    this.display.fillRectangle(0, 0, mapWidth + FRAME_BORDER_WIDTH * 2, mapHeight + FRAME_BORDER_WIDTH * 2);

    // ===================================================================================
    // draw obstacles & coloured tiles
    //   - to do this, cycle through the model and update the occupancy

    // Take into acount that the robot position assumes that (0,0) is in the bottom 
    // left of the map, but the display is indexed with (0,0) in the top left.
    int cellOffsetX = FRAME_BORDER_WIDTH; // + colPos*model.getCellWidth();
    int cellOffsetY = this.deviceHeight-FRAME_BORDER_WIDTH; // + rowPos*model.getCellHeight();
    CellOccupancyType occType = CellOccupancyType.UNKNOWN;
                                        
    for (int rowPos=0; rowPos < numCellsInCol; rowPos++) { 
      cellOffsetX = FRAME_BORDER_WIDTH;
      for (int colPos=0; colPos < numCellsInRow; colPos++) {
        switch(occType = map.getOccupancy(colPos, rowPos)) {
        case OBSTACLE:          
          this.display.setColor(GREY);     // Grey
          this.display.fillRectangle(cellOffsetX, cellOffsetY-cellHeight, cellWidth, cellHeight);
          break;                        
        case RED:               
          this.display.setColor(RED);     // Red
          this.display.fillRectangle(cellOffsetX, cellOffsetY-cellHeight, cellWidth, cellHeight);
          break;                        
        case GREEN:             
          this.display.setColor(GREEN);     // Green
          this.display.fillRectangle(cellOffsetX, cellOffsetY-cellHeight, cellWidth, cellHeight);
          break;                        
        case BLUE:              
          this.display.setColor(BLUE);     // Blue
          this.display.fillRectangle(cellOffsetX, cellOffsetY-cellHeight, cellWidth, cellHeight);
          break;                        
        case ROBOT:             
        case EMPTY:             
        case UNKNOWN:           
        default:                
          ;  // Do nothing              
        }                       
        cellOffsetX += cellWidth;
      }                 
      cellOffsetY -= cellHeight;
    }    
    // ===================================================================================
    // draw lines for the model
    this.display.setColor(0xB4B4FF);     // ???????
    int lineOffset = FRAME_BORDER_WIDTH;
                
    // Draw Vertical Lines
    for (int x=0; x <= numCellsInRow; x++) {    // Draw extra border after last cell
      this.display.fillRectangle(lineOffset, FRAME_BORDER_WIDTH, CELL_LINE_WIDTH, mapHeight);
      lineOffset += cellWidth;
    }           
    // Draw Horizontal Lines
    lineOffset = FRAME_BORDER_WIDTH;      // Reset the value            
    for (int y=0; y <= numCellsInCol; y++) {    // Draw extra border after last cell
      this.display.fillRectangle(FRAME_BORDER_WIDTH, lineOffset, mapWidth, CELL_LINE_WIDTH);
      lineOffset += cellHeight;
    }     


    // ===================================================================================
    // Draw the position of the robot
    int robotX = FRAME_BORDER_WIDTH + (int) (this.robotPose.getX() * this.scaleFactor); // x coordinate

    // Take into acount that the robot position assumes that (0,0) is in the bottom 
    // left of the map, but the display is indexed with (0,0) in the top left.
    int robotY = this.deviceHeight - 
        (FRAME_BORDER_WIDTH + (int) (this.robotPose.getY() * this.scaleFactor));        // y coordinate

    int robotR = (int) (RADIUS_ROBOTBODY * this.scaleFactor);                           // body radius
    double robotH = this.robotPose.getTheta();                                          // heading angle in radians
    // ------------------------
    // Draw Robot Body          
    this.display.setColor(WHITE);     // White
    this.display.fillOval(robotX, robotY, robotR, robotR);
    this.display.setColor(0x3C3C3C);     // Dark Grey
    this.display.drawOval(robotX, robotY, robotR, robotR);
                      
    // ------------------------
    // Need to indicate heading          
    int headingX = robotX + (int) (Math.cos(robotH) * robotR);
    int headingY = robotY - (int) (Math.sin(robotH) * robotR);  // Note we invert the y axis as (0,0) is top left
    this.display.setColor(0x3C3C3C);     // Dark Grey
    this.display.drawLine(robotX, robotY, headingX, headingY);
                          
  }

  public void paintParticles(ArrayList<Particle> particleSet) {
    int particleX;
    int particleY;
    
    this.display.setColor(RED);     // RED Dots
    for (Particle p:particleSet) {
      particleX = FRAME_BORDER_WIDTH + (int) (p.getX() * this.scaleFactor);		// x coordinate

      // Take into acount that the robot position assumes that (0,0) is in the bottom 
      // left of the map, but the display is indexed with (0,0) in the top left.
      particleY = this.deviceHeight -
               (FRAME_BORDER_WIDTH + (int) (p.getY() * this.scaleFactor));		// y coordinate
      
      this.display.fillOval(particleX, particleY, 2, 2); // radius of 2 in both directions
    }			

  }

}