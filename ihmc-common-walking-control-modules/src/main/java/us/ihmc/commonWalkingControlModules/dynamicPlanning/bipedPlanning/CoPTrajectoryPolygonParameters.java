package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

public class CoPTrajectoryPolygonParameters implements CoPTrajectoryPolygonParametersReadOnly
{
   public double stepLengthToDoToeOff = 0.05;

   public double getStepLengthToPlanToeOff()
   {
      return stepLengthToDoToeOff;
   }
}
