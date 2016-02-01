package us.ihmc.commonWalkingControlModules.trajectories;


public class CenterOfMassHeightVelocityAndAccelerationData
{
   private double comZPosition;
   private double comZVelocity;
   private double comZAcceleration;
   
   public void set(CenterOfMassHeightVelocityAndAccelerationData comData)
   {
      this.comZPosition = comData.comZPosition;
      this.comZVelocity = comData.comZVelocity;
      this.comZAcceleration = comData.comZAcceleration;
   }
   
   public double getCoMZPosition()
   {
      return comZPosition;
   }
   
   public double getCoMZVelocity()
   {
      return comZVelocity;
   }
   
   public double getCoMZAcceleration()
   {
      return comZAcceleration;
   }
}
