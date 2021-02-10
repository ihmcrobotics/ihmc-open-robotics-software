package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Specifies a desired virtual repellent point velocity for segment {@link #getSegmentNumber()} to achieve at time {@link #getTimeOfObjective()}.
 */
public class VRPVelocityCommand extends MPCValueCommand
{
   public int getDerivativeOrder()
   {
      return 1;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.VRP;
   }
}
