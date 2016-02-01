package us.ihmc.darpaRoboticsChallenge.driving.imageProcessing;

/**
 * @author Peter Abeles
 */
public class CombineMapsTogether
{
   OccupancyGrid old = new OccupancyGrid(1,1,1);
   double centerX,centerY;
   double unknownValue;

   public CombineMapsTogether(double centerX, double centerY, double unknownValue)
   {
      this.centerX = centerX;
      this.centerY = centerY;
      this.unknownValue = unknownValue;
   }

   double dx;double dy;double dyaw;

   public void setOldMap( OccupancyGrid map ) {
      old.resize(map.width,map.height);
      old.setCellSize(map.getCellSize());

      System.arraycopy(map.map,0,old.map,0,map.width*map.height);
   }

   public void setOldMap( double dx , double dy , double dyaw ) {
      this.dx = dx;
      this.dy = dy;
      this.dyaw = dyaw;
   }

   /**
    *
    * (dx,dy,dyaw) specify pose in the old map's coordinate system.
    *
    * @param map
    * @param dx
    * @param dy
    * @param dyaw
    */
   public void setOldMap( OccupancyGrid map , double dx , double dy , double dyaw ) {
      old.resize(map.width,map.height);
      old.setCellSize(map.getCellSize());

      System.arraycopy(map.map,0,old.map,0,map.width*map.height);

      this.dx = dx;
      this.dy = dy;
      this.dyaw = dyaw;

      // compute inverse transform
//      double c = Math.cos(-dyaw);
//      double s = Math.sin(-dyaw);
//
//      // R*X + T = Y ==>  x = R'*Y - R'*T
//      this.dx = -(dx*c - dy*s);
//      this.dy = -(dx*s + dy*c);
//      this.dyaw = -dyaw;
   }

   public void combineMap( OccupancyGrid map ) {

      double c = Math.cos(dyaw);
      double s = Math.sin(dyaw);

      for( int y = 0; y < map.height; y++ ) {
         for( int x = 0; x < map.width; x++ ) {
            double v = map.unsafe_get(x,y);

            if( v != unknownValue )
               continue;

            // map coordinate
            double currX = x*map.cellSize - centerX;
            double currY = y*map.cellSize - centerY;

            // find location in previous map
            double oldX =  currX*c - currY*s + dx;
            double oldY =  currX*s + currY*c + dy;

            // find it in the grid
            int gridX = (int)((oldX + centerX)/old.cellSize);
            int gridY = (int)((oldY + centerY)/old.cellSize);

            if( old.isInBounds(gridX,gridY) ) {
               v = old.unsafe_get(gridX,gridY);
//               if( v == 0 )
//                  v = 0.01;
               map.unsafe_set(x,y,v);
            }
         }
      }
   }


}
