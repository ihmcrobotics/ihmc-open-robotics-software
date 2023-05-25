package us.ihmc.stateEstimation.humanoid;

import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;

public interface StateEstimatorControllerFactory
{
   StateEstimatorController createStateEstimator(FullHumanoidRobotModel fullRobotModel, SensorReader sensorReader);
}
