package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

public class LidarCarLocalizerWheelAngleEstimate
{
   public double angle;
   public LidarCarLocalizerWheelAngleEstimate()
   {
      angle = 0 ;
   }
   public void setAngle(double angle)
   {
      this.angle = angle;
   }
   public double getAngle()
   {
      return this.angle;
   }
}
