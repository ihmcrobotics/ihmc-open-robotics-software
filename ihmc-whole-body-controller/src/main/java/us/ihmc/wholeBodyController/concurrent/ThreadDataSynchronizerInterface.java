package us.ihmc.wholeBodyController.concurrent;

import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public interface ThreadDataSynchronizerInterface
{

   public abstract boolean receiveEstimatorStateForController();

   public abstract void publishEstimatorState(long timestamp, long estimatorTick, long estimatorClockStartTime);

   public abstract FullHumanoidRobotModel getEstimatorFullRobotModel();

   public abstract ForceSensorDataHolder getEstimatorForceSensorDataHolder();

   public abstract CenterOfMassDataHolder getEstimatorCenterOfMassDataHolder();

   public abstract FullHumanoidRobotModel getControllerFullRobotModel();

   public abstract ForceSensorDataHolderReadOnly getControllerForceSensorDataHolder();
   
   public abstract CenterOfMassDataHolderReadOnly getControllerCenterOfMassDataHolder();

   public abstract RawJointSensorDataHolderMap getEstimatorRawJointSensorDataHolderMap();

   public abstract RawJointSensorDataHolderMap getControllerRawJointSensorDataHolderMap();

   public abstract CenterOfPressureDataHolder getEstimatorCenterOfPressureDataHolder();

   public abstract CenterOfPressureDataHolder getControllerCenterOfPressureDataHolder();

   public abstract RobotMotionStatusHolder getEstimatorRobotMotionStatusHolder();

   public abstract RobotMotionStatusHolder getControllerRobotMotionStatusHolder();

   public abstract long getTimestamp();

   public abstract long getEstimatorClockStartTime();

   public abstract long getEstimatorTick();

   public abstract void publishControllerData();

   public abstract boolean receiveControllerDataForEstimator();

   public abstract JointDesiredOutputList getEstimatorDesiredJointDataHolder();
   
   public abstract JointDesiredOutputList getControllerDesiredJointDataHolder();

}