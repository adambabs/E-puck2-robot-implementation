// File: MapModel.java
// Date: 28th Dec 2020
// Description: Map Model for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:
/**
 * The map model that is used to model the world as a set of constant sized cells
 * Originally based on the Robosim code for COMP329, Dec 2016
 *
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.NoSuchElementException;
import java.util.Scanner;

public class MapModel {

  // ===================================================
  // Instance Variables
  private String configFileName;        // String containing a path and filename to the config string

  private int mapWidthInCells;
  private int mapHeightInCells;

  private static int CELLWIDTH = 100;      // Corresponds to the number of mm per cell
  private static int CELLHEIGHT = 100;    // Corresponds to the number of mm per cell

  private ArrayList<ArrayList<Cell>> grid;


  public MapModel() {
    // TODO Auto-generated constructor stub
  }


  // ===================================================
  // Getters / Setters

  /**
   * @return the mapWidthInCells
   */
  public int getMapWidthInCells() {
    return mapWidthInCells;
  }

  /**
   * @return the mapHeightInCells
   */
  public int getMapHeightInCells() {
    return mapHeightInCells;
  }

  /**
   * @return the cell height in mm
   */
  public int getCellHeight() {
    return CELLHEIGHT;
  }

  /**
   * @return the cell width in mm
   */
  public int getCellWidth() {
    return CELLWIDTH;
  }

  // ===================================================
  // Constructors

  /**
   * Constructor for the Map Model.  This version deviates from the original version written for
   * RoboSim as it does not require the perimeter cells (representing the walls) to be given
   * @param configFileName the full path and name of the ascii file containing the location of obstacles
   * @param mapWidth The width of the map (inc perimeter cells)
   * @param mapHeight The height of the map (inc perimeter cells)
   */
  public MapModel(String configFileName, int mapWidth, int mapHeight) {
    super();
    this.mapWidthInCells = mapWidth;
    this.mapHeightInCells = mapHeight;
    this.configFileName = configFileName;

    // Create the array of arrays
    this.grid = new ArrayList<ArrayList<Cell>>();

    // Construct the Map Cells and ensure all are empty
    for (int rowID=0; rowID < mapHeight; rowID++) {
      // Construct a new row in the grid
      ArrayList<Cell> row = new ArrayList<Cell>();
      for (int colID=0; colID < mapWidth; colID++) {
        // Construct and insert a new cell
        row.add(Cell.getEmptyCell());
      }
      // Now add the row to the grid
      grid.add(row);
    }

    // Add any obstacles in the map
    this.readConfig();
  }


  /**
   * Set the occupancy of an empty cell.
   * Note that the method will fail if the cell has been set to something not empty
   *
   * @param colPos  column position of the cell
   * @param rowPos  row position of the cell
   * @param occType  the new occupancy type
   * @return boolean  to state if he call was successful (i.e. the cell was not originally empty)
   */
  private Boolean setOccupancy(int colPos, int rowPos,CellOccupancyType occType) {
    Boolean result = false;
    ArrayList<Cell> row;
    Cell cell;

    // Check the bounds of the col and row pos
    if ((rowPos >= 0) && (rowPos < this.grid.size())) {
      row = this.grid.get(rowPos);
      if ((colPos >= 0) && (colPos < row.size())) {
        cell = row.get(colPos);

        // Only change the occupancy of a cell if it is empty
        // Note, separate methods will be used when modelling the robot
        if (cell.isEmpty()) {
          cell.setCellType(occType);
          result = true;
        }
      }
    }
    return result;
  }


  /**
   * Return the occupancy type of the cell.
   * @param colPos   column position of the cell
   * @param rowPos  row position of the cell
   * @return occupancy type of the cell, or UNKNOWN if the positions were out of bounds
   */
  public CellOccupancyType getOccupancy(int colPos, int rowPos) {
    ArrayList<Cell> row;
    Cell cell;

    // Check the bounds of the col and row pos
    if ((rowPos >= 0) && (rowPos < this.grid.size())) {
      row = this.grid.get(rowPos);
      if ((colPos >= 0) && (colPos < row.size())) {
        cell = row.get(colPos);

        return cell.getCellType();
      }
    }
    return CellOccupancyType.UNKNOWN;  // Something went wrong, or the bounds were invalid
  }

  // ===========================================================================
  // Given two coordinates, iterate through the set of map features, to
  // find the distance to the nearest one.  Note that we don't return
  // details of the feature in this implementation
  double get_nearest_map_feature_dist(double x, double y) {
    double min_distSQ = Double.MAX_VALUE;
    double distSq;
    double ox, oy;    // lower x and y coords of the obstacle
    double cx, cy;    // clamped x and y coordinates
    double dx, dy;    // difference (delta) between the x and y coordinates

    for (int row=0; row < this.getMapHeightInCells(); row++) {
      for(int col=0; col < this.getMapWidthInCells(); col++) {
        if (this.getOccupancy(col, row).equals(CellOccupancyType.OBSTACLE)) {
          // Calculate distance to this obstacle (an axis aligned rectangle)
          // to do this we need to "clamp" the point into the rectange and determine
          // the distance from the clamped point
          ox = (col * this.getCellWidth());
          oy = (row * this.getCellHeight());
          cx = Math.max(Math.min(x, ox+this.getCellWidth()), ox);
          cy = Math.max(Math.min(y, oy+this.getCellHeight()), oy);
          
          distSq = ((x-cx)*(x-cx)) + ((y-cy)*(y-cy));
          
          // If the distance is 0.0, then this could be because the coordinate
          // is inside the obstacle, which can lead to biassing weights towards
          // invalid particles.  For now we only consider distances greater than
          // an arbitary value (in this case 1.0)
          if ((distSq>1.0) && (distSq < min_distSQ))
            min_distSQ = distSq;
        }
      }
    }
    return Math.sqrt(min_distSQ);	
  }
  // ==================================================================================





  // ===========================================================================
  // Parser Code
  // Note that this parser is really flakey
  // ===========================================================================

  /**
   * This parses a simple line of text to position artifacts in the map
   * @param line a line of text from the file
   */
  private void parseConfigLine(String line) {
    Scanner sc = new Scanner(line);
    sc.useDelimiter("\\s*,\\s*");
    try {
      // Format is colPos, rowPos, type
      int colPos = sc.nextInt();
      int rowPos = sc.nextInt();
      String occTypeStr = sc.next();
      CellOccupancyType occ = CellOccupancyType.EMPTY;

      // Would be nice to use a switch here !!! But for compatabilities sake...
      // Looking for one of: OBSTACLE, EMPTY, ROBOT, RED, BLUE, GREEN

      if (occTypeStr.equals("OBSTACLE")) occ = CellOccupancyType.OBSTACLE;
      else if (occTypeStr.equals("BLUE")) occ = CellOccupancyType.BLUE;
      else if (occTypeStr.equals("GREEN")) occ = CellOccupancyType.GREEN;
      else if (occTypeStr.equals("RED")) occ = CellOccupancyType.RED;
      else if (occTypeStr.equals("ROBOT")) occ = CellOccupancyType.ROBOT;

      if (!occ.equals(CellOccupancyType.EMPTY)) {
        if (!this.setOccupancy(colPos, rowPos, occ)) {
          System.err.println("Error in parseConfigLine setting ("+colPos+","+rowPos+")");
        }
      }
    } catch (NoSuchElementException e) {
      //e.printStackTrace();
    }
    sc.close();

  }


  /**
   * This reads the config file containing the positions of the artifacts in the map
   * @return true - this should be the success condition of parsing the config file
   */

  private Boolean readConfig() {
    File file = new File(this.configFileName);
    String result = new String();

    try {
      Scanner sc = new Scanner(file);
      //      sc.useDelimiter(System.getProperty("line.separator"));
      while (sc.hasNextLine()) {
        String line = sc.nextLine();
        this.parseConfigLine(line);
      }
      sc.close();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    for (int j=0; j<this.getMapHeightInCells(); j++) {
      this.setOccupancy(0, j, CellOccupancyType.OBSTACLE);
      this.setOccupancy(this.getMapWidthInCells()-1, j, CellOccupancyType.OBSTACLE);
    }

    for (int i=0; i < this.getMapWidthInCells(); i++) {
      this.setOccupancy(i, 0, CellOccupancyType.OBSTACLE);
      this.setOccupancy(i, this.getMapHeightInCells()-1, CellOccupancyType.OBSTACLE);
    }

    System.out.println(result);

    return true;
  }

  // ===================================================
  // Other Public Methods

  /**
   * Determine a position in the center of the map and return the position as a pose,
   * assuming that the robot has the orientation of PI/2 radians (i.e. 90 degrees)
   * If the center is not positioned in a cell, then positions will be rounded down
   * to assuming an odd number of cells
   *
   * @return the center of the map as a pose
   */
  public Pose getCenterPose() {
    Pose p = new Pose();
    // Note we add 1 to the cell count.  If odd, then this will result in an even number
    // where the center cell is the mid point (e.g. 11 cells gives position 6) whereas if
    // even, then the mid point is rounded down due to integer arithmetic (e.g. 10 cells
    // gives position 5).
    
    // Also need to offset the robot into the center of the cell (hence the -0.5 scalar offset)
    
    int w = (1+this.getMapWidthInCells())/2; // Int arithmetic, hence numbers are rounded down
    int h = (1+this.getMapHeightInCells())/2;// Int arithmetic, hence numbers are rounded down
    double x = ((double) w - 0.5) * (double) this.getCellWidth();  // Needs to be "real" arithmetic
    double y = ((double) h - 0.5) * (double) this.getCellHeight(); // Needs to be "real" arithmetic
    p.setPosition(x,y,Math.PI/2.0);

    return p;
  }

  /**
   * Generate a string representation of the Map
   * Note that we need to invert so that cell 0,0 is at the bottom left
   */
  public String toString() {
    String result = new String();
    String rowStr = new String();
    result = "Map (Arena): " + this.getMapWidthInCells() + " x " + this.getMapHeightInCells() + "\n";
    for (ArrayList<Cell> row: this.grid) {
      rowStr="";
      for (Cell c:row) {
        rowStr += c.toString();
      }
      rowStr += "\n";
      result = rowStr+result;
    }
    return result;
  }

}
