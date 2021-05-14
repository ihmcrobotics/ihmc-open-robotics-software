package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Specifies a desired divergent component of motion position for segment {@link #getSegmentNumber()} to achieve at time {@link #getTimeOfObjective()}.
 */
public class DCMPositionCommand extends MPCValueCommand
{
   public int getDerivativeOrder()
   {
      return 0;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.DCM;
   }
}
