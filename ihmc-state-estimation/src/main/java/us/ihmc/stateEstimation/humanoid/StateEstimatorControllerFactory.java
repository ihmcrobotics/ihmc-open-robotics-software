package us.ihmc.stateEstimation.humanoid;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;

public interface StateEstimatorControllerFactory
{
   /**
    * @param gravitationalAcceleration the same gravity the main estimator uses, passed by the estimator thread
    *                                  factory so a secondary estimator cannot disagree with it. Sign not
    *                                  considered; ignore it if the estimator is purely kinematic.
    */
   StateEstimatorController createStateEstimator(FullHumanoidRobotModel fullRobotModel, SensorReader sensorReader, double gravitationalAcceleration);
}
