package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

public class BodyAngularVelocityContinuityCommand extends MPCContinuityCommand
{
   public int getDerivativeOrder()
   {
      return 1;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.ORIENTATION;
   }
}
