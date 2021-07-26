package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Specifies a desired center of mass position for segment {@link #getSegmentNumber()} to achieve at time {@link #getTimeOfObjective()}.
 */
public class CoMPositionCommand extends MPCValueCommand
{
   public int getDerivativeOrder()
   {
      return 0;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.COM;
   }
}
