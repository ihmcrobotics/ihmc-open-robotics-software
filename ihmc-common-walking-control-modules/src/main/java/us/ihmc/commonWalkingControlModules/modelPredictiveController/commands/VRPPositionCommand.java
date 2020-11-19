package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

public class VRPPositionCommand extends MPCValueCommand
{
   public int getDerivativeOrder()
   {
      return 0;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.VRP;
   }
}
