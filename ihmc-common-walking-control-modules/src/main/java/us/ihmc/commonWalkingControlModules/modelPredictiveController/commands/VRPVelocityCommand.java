package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

public class VRPVelocityCommand extends MPCValueCommand
{
   public int getDerivativeOrder()
   {
      return 1;
   }

   public MPCValueType getValueType()
   {
      return MPCValueType.VRP;
   }
}
