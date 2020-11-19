package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

public class CoMVelocityContinuityCommand extends CoMContinuityCommand
{
   public int getDerivativeOrder()
   {
      return 1;
   }
}
