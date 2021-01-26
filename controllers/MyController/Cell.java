// File: Cell.java
// Date: 26th Dec 2020
// Description: Cell Class for the Map Model for COMP329 Programming Assignment (2020)
// Author: Terry Payne
// Modifications:

/**
 * This class models a grid cell within the Arena Model
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 * 
 * Based on the original RoboSim code written for COMP329, Dec 2016
 *
 */

public class Cell {
  private CellOccupancyType cellType;

  // ==============================================
  // Getters & Setters
  /**   
   * @return the cellType
   */    
  public CellOccupancyType getCellType() {
        return cellType;
  }     

  /**   
   * @param cellType the cellType to set
   */    
  public void setCellType(CellOccupancyType cellType) {
        this.cellType = cellType;
  }     
        
  // ===================================================
  // Class Constructor
  /**   
   * Returns a new empty grid cell. 
   */    
  public static Cell getEmptyCell()
  {     
        Cell empty = new Cell(); 
        empty.setEmpty();
        return empty;
  }     

  // ===================================================
  // Methods
        
  /**   
   */    
  public void setEmpty() {
        this.cellType = CellOccupancyType.EMPTY;
  }     


  /**   
   * Returns true if the cell is empty.
   */    
  public Boolean isEmpty() {
        if (this.cellType == CellOccupancyType.EMPTY)
                return true;
        return false;
  }     
        
  public String toString() {
        String result = new String();
        switch(this.cellType) {
        case EMPTY:
                result = " ";
                break;  
        case OBSTACLE:
                result = "#";
                break;  
        case ROBOT:
                result = "*";
                break;  
        case RED:
                result = "r";
                break;  
        case GREEN:
                result = "g";
                break;  
        case BLUE:
                result = "b";
                break;  
        default:
                result = "?";
        }       
        result += " ";   // Space out the value

        return result; 
  }     
}

