package us.ihmc.robotics.heightMap;

public interface HeightMapParametersReadOnly
{
   /**
    * Resolution of the height map grid
    */
   double getGridResolutionXY();

   /**
    * Length of the side of the square height map grid
    */
   double getGridSizeXY();

   /**
    * Max z relative to robot mid foot z. Points above this threshold are ignored.
    */
   double getMaxZ();

   /**
    * When calibrated on flat ground, this is the average standard deviation observed for a grid cell.
    */
   double getNominalStandardDeviation();

   int getMaxPointsPerCell();

   /**
    * If a grid cell is at height h, points below (h - s * m) are ignored, and points above (h + s * m) will cause the cell to throw out old data and reset.
    * where s is getNominalStandardDeviation() and m is this value.
    */
   double getMahalanobisScale();

}
