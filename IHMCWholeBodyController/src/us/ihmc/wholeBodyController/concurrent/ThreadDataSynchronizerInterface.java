package us.ihmc.wholeBodyController.concurrent;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.utilities.humanoidRobot.model.CenterOfPressureDataHolder;
import us.ihmc.utilities.humanoidRobot.model.ForceSensorDataHolder;

public interface ThreadDataSynchronizerInterface
{

   public abstract boolean receiveEstimatorStateForController();

   public abstract void publishEstimatorState(long timestamp, long estimatorTick, long estimatorClockStartTime);

   public abstract SDFFullRobotModel getEstimatorFullRobotModel();

   public abstract ForceSensorDataHolder getEstimatorForceSensorDataHolder();

   public abstract SDFFullRobotModel getControllerFullRobotModel();

   public abstract ForceSensorDataHolder getControllerForceSensorDataHolder();

   public abstract RawJointSensorDataHolderMap getEstimatorRawJointSensorDataHolderMap();

   public abstract RawJointSensorDataHolderMap getControllerRawJointSensorDataHolderMap();

   public abstract CenterOfPressureDataHolder getEstimatorCenterOfPressureDataHolder();

   public abstract CenterOfPressureDataHolder getControllerCenterOfPressureDataHolder();

   public abstract long getTimestamp();

   public abstract long getEstimatorClockStartTime();

   public abstract long getEstimatorTick();

   public abstract void publishControllerData();

   public abstract boolean receiveControllerDataForEstimator();

}