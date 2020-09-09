package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

public class CoPTrajectoryParameters implements CoPTrajectoryParametersReadOnly
{
   public double stepLengthToDoToeOff = 0.05;

   public double getStepLengthToPlanToeOff()
   {
      return stepLengthToDoToeOff;
   }
}
