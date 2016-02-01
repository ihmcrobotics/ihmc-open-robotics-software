package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;

/**
 * Occupancy grid map in 2D.  Indicates the probability of there build an obstacle.
 *
 * @author Peter Abeles
 */
public class OccupancyGrid_U16
{
   public short map[];
   public int width;
   public int height;
   public double cellSize;
   public Point2D_F64 offset = new Point2D_F64();

   public OccupancyGrid_U16(int width, int height, double cellSize)
   {
      this.width = width;
      this.height = height;
      this.cellSize = cellSize;

      map = new short[width*height];
   }

   public void setOffsetMap(double x, double y) {
      offset.set(x, y);
   }

   public void worldToCell( double x , double y , Point2D_I32 grid ) {
      grid.x = (int)Math.floor((x + offset.x)/ cellSize);
      grid.y = (int)Math.floor((y + offset.y)/ cellSize);
   }

   public void setAll( short value ) {
      for( int i = 0; i < map.length; i++ )
         map[i] = value;
   }

   public int get( int x , int y ) {
      checkBounds(x, y);

      return map[ y*width + x ] & 0xFFFF;
   }

   public int unsafe_get( int x , int y ) {
      return map[ y*width + x ] & 0xFFFF;
   }

   public void set( int x , int y , int value ) {
      checkBounds(x, y);

      map[ y*width + x ] = (short)value;
   }

   public void unsafe_set( int x , int y , int value ) {
      map[ y*width + x ] = (short)value;
   }

   public void setTo( OccupancyGrid_U16 grid ) {
      if( grid.width != width || grid.height != height )
         throw new IllegalArgumentException("Both grids must have the same width and height");

      System.arraycopy(grid.map,0,map,0,width*height);
      this.cellSize = grid.cellSize;
   }

   private void checkBounds(int x, int y)
   {
      if( x < 0 || x >= width )
         throw new IllegalArgumentException("x coordinate is out of bounds: "+x);
      if( y < 0 || y >= height )
         throw new IllegalArgumentException("y coordinate is out of bounds: "+y);
   }

   public int getWidth()
   {
      return width;
   }

   public int getHeight()
   {
      return height;
   }

   public double getCellSize()
   {
      return cellSize;
   }

   public void setCellSize(double cellsize)
   {
      this.cellSize = cellsize;
   }

   public void resize( int width , int height ) {
      this.width = width;
      this.height = height;

      if( width*height > map.length ) {
         map = new short[ width*height ];
      }
   }

   public boolean isInBounds( int x , int y ) {
      return x >=0 && x < width && y >= 0 && y < height;
   }
}
