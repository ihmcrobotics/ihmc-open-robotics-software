package us.ihmc.stateEstimation.humanoid;

import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimator;

public interface DRCStateEstimatorInterface extends RobotController
{
   public abstract StateEstimator getStateEstimator();

   public void initializeEstimatorToActual(Tuple3DReadOnly initialCoMPosition, QuaternionReadOnly initialEstimationLinkOrientation);
}
