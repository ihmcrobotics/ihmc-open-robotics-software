package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

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
