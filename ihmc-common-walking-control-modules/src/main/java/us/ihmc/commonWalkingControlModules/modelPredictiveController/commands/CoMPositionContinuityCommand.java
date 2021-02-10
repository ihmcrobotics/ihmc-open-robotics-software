package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Specifies that the center of mass position for the segment {@link #getFirstSegmentNumber()} at time {@link #getFirstSegmentDuration()} must be equal to the
 * next segment at time 0.
 */
public class CoMPositionContinuityCommand extends MPCContinuityCommand
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
