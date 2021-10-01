package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Specifies a desired center of mass acceleration for segment {@link #getSegmentNumber()} to achieve at time {@link #getTimeOfObjective()}.
 */
public class CoMAccelerationCommand extends MPCValueCommand
{
   public int getDerivativeOrder()
   {
      return 2;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.COM;
   }
}
