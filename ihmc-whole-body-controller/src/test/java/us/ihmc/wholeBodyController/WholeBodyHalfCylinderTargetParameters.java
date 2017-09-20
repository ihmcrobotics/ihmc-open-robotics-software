package us.ihmc.wholeBodyController;

public class WholeBodyHalfCylinderTargetParameters
{
   private double maxReachRadius = 0.8;
   private double maxHeight = 1.5;
   private double maxTheta = Math.PI * 3 / 4;
   private int radiusIncrements = 4;
   private int heightIncrements = 4;
   private int thetaIncrements = 4;
   
   public double getMaxReachRadius()
   {
      return maxReachRadius;
   }
   public void setMaxReachRadius(double maxReachRadius)
   {
      this.maxReachRadius = maxReachRadius;
   }
   public double getMaxHeight()
   {
      return maxHeight;
   }
   public void setMaxHeight(double maxHeight)
   {
      this.maxHeight = maxHeight;
   }
   public double getMaxTheta()
   {
      return maxTheta;
   }
   public void setMaxTheta(double maxTheta)
   {
      this.maxTheta = maxTheta;
   }
   public int getRadiusIncrements()
   {
      return radiusIncrements;
   }
   public void setRadiusIncrements(int radiusIncrements)
   {
      this.radiusIncrements = radiusIncrements;
   }
   public int getHeightIncrements()
   {
      return heightIncrements;
   }
   public void setHeightIncrements(int heightIncrements)
   {
      this.heightIncrements = heightIncrements;
   }
   public int getThetaIncrements()
   {
      return thetaIncrements;
   }
   public void setThetaIncrements(int thetaIncrements)
   {
      this.thetaIncrements = thetaIncrements;
   }
   
   
}
