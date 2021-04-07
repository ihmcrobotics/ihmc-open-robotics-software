package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

public class OrientationTrajectoryConstructor
{
   private final OrientationDynamicsCalculator inputCalculator;

   public OrientationTrajectoryConstructor(SE3MPCIndexHandler indexHandler, double mass, double gravity)
   {
      inputCalculator = new OrientationDynamicsCalculator(indexHandler, mass, gravity);
   }
}
