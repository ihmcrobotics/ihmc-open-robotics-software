package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

public class BodyOrientationCommand extends MPCValueCommand
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
