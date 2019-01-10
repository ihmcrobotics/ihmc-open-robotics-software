package us.ihmc.quadrupedPlanning.footstepPlanning;

public interface ConstantAccelerationBodyPathParameters
{
   default double getMaxForwardAcceleration()
   {
      return 5.0;
   }

   default double getMaxLateralAcceleration()
   {
      return 2.5;
   }

   default double getMaxYawAcceleration()
   {
      return 2.0 * Math.PI;
   }

   default double getMaxYawRate()
   {
      return Math.PI / 4.0;
   }

   default double getFastVelocity()
   {
      return 1.0;
   }

   default double getMediumVelocity()
   {
      return 0.5;
   }

   default double getSlowVelocity()
   {
      return 0.1;
   }

}
