package us.ihmc.stateEstimation.humanoid;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;

public interface DRCStateEstimatorInterface extends RobotController
{
   public abstract StateEstimator getStateEstimator();

   public abstract void initializeEstimatorToActual(Tuple3d initialCoMPosition, Quat4d initialEstimationLinkOrientation);
}
