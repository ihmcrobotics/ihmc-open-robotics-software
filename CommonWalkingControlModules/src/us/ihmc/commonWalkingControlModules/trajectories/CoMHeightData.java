package us.ihmc.commonWalkingControlModules.trajectories;

public class CoMHeightData
{
   private double comHeight, comHeightVelocity, comHeightAcceleration;

   public double getComHeight()
   {
      return comHeight;
   }

   public void setComHeight(double comHeight)
   {
      this.comHeight = comHeight;
   }

   public double getComHeightVelocity()
   {
      return comHeightVelocity;
   }

   public void setComHeightVelocity(double comHeightVelocity)
   {
      this.comHeightVelocity = comHeightVelocity;
   }

   public double getComHeightAcceleration()
   {
      return comHeightAcceleration;
   }

   public void setComHeightAcceleration(double comHeightAcceleration)
   {
      this.comHeightAcceleration = comHeightAcceleration;
   }


   public void set(CoMHeightData heightZData)
   {
      this.comHeight = heightZData.comHeight;
      this.comHeightVelocity = heightZData.comHeightVelocity;
      this.comHeightAcceleration = heightZData.comHeightAcceleration;   
   }
   
   
}
