package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

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
