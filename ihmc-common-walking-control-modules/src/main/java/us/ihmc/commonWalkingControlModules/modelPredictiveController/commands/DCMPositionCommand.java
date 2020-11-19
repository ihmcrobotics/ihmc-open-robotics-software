package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

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
