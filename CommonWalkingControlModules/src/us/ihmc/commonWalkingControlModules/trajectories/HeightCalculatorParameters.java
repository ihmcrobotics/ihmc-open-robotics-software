package us.ihmc.commonWalkingControlModules.trajectories;

/**
 * Created by agrabertilton on 3/10/15.
 */
public class HeightCalculatorParameters
{
   private double horizontalBuffer;
   private double verticalBuffer;
   private double pathWidth;
   private double zHeightForOutlierQualification;
   private int numberOfOutliersToIgnore;

   public HeightCalculatorParameters(double verticalBuffer, double pathWidth){
      this(0.0, verticalBuffer, pathWidth);
   }

   public HeightCalculatorParameters(double horizontalBuffer, double verticalBuffer, double pathWidth){
      this(horizontalBuffer, verticalBuffer, pathWidth, 0.0, 0);
   }

   public HeightCalculatorParameters(double horizontalBuffer, double verticalBuffer, double pathWidth, double zHeightForOutlierQualification, int numberOfOutliersToIgnore){
      this.horizontalBuffer = horizontalBuffer;
      this.verticalBuffer = verticalBuffer;
      this.pathWidth = pathWidth;
      this.zHeightForOutlierQualification = zHeightForOutlierQualification;
      this.numberOfOutliersToIgnore = numberOfOutliersToIgnore;
   }

   public double getHorizontalBuffer()
   {
      return horizontalBuffer;
   }

   public double getVerticalBuffer()
   {
      return verticalBuffer;
   }

   public double getPathWidth()
   {
      return pathWidth;
   }

   public double getzHeightForOutlierQualification()
   {
      return zHeightForOutlierQualification;
   }

   public int getNumberOfOutliersToIgnore()
   {
      return numberOfOutliersToIgnore;
   }
}
