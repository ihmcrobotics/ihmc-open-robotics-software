package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Specifies a desired divergent component of motion velocity for segment {@link #getSegmentNumber()} to achieve at time {@link #getTimeOfObjective()}.
 */
public class DCMVelocityCommand extends MPCValueCommand
{
   public int getDerivativeOrder()
   {
      return 1;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.DCM;
   }
}
