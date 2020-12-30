package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

public class BodyOrientationContinuityCommand extends MPCContinuityCommand
{
   public int getDerivativeOrder()
   {
      return 0;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.ORIENTATION;
   }
}
