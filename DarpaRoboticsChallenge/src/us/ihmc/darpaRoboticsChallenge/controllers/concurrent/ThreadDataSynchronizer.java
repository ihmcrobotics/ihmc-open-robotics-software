package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import java.util.Arrays;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.sensorProcessing.sensors.ForceSensorDataHolder;

public class ThreadDataSynchronizer
{
   private final SDFFullRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder estimatorForceSensorDataHolder;

   private final SDFFullRobotModel controllerFullRobotModel;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;

   private final ConcurrentCopier<IntermediateEstimatorStateHolder> estimatorStateCopier;
   
   private long timestamp;
   private long estimatorClockStartTime;

   public ThreadDataSynchronizer(DRCRobotModel robotModel)
   {
      estimatorFullRobotModel = robotModel.createFullRobotModel();
      estimatorForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(estimatorFullRobotModel.getForceSensorDefinitions()),
            estimatorFullRobotModel.getRootJoint());

      controllerFullRobotModel = robotModel.createFullRobotModel();
      controllerForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(controllerFullRobotModel.getForceSensorDefinitions()),
            controllerFullRobotModel.getRootJoint());

      IntermediateEstimatorStateHolder.Builder stateCopierBuilder = new IntermediateEstimatorStateHolder.Builder(robotModel,
            estimatorFullRobotModel.getElevator(), controllerFullRobotModel.getElevator(), estimatorForceSensorDataHolder, controllerForceSensorDataHolder);
      estimatorStateCopier = new ConcurrentCopier<IntermediateEstimatorStateHolder>(stateCopierBuilder);

   }

   public void receiveControllerState()
   {
      IntermediateEstimatorStateHolder estimatorStateHolder = estimatorStateCopier.getCopyForReading();
      if (estimatorStateHolder != null)
      {
         estimatorStateHolder.getIntoControllerModel();
         timestamp = estimatorStateHolder.getTimestamp();
         estimatorClockStartTime = estimatorStateHolder.getEstimatorClockStartTime();
      }
      else
      {
         timestamp = Long.MIN_VALUE;
         estimatorClockStartTime = Long.MIN_VALUE;
      }
   }

   public void publishEstimatorState(long timestamp, long estimatorTick, long estimatorClockStartTime)
   {
      IntermediateEstimatorStateHolder estimatorStateHolder = estimatorStateCopier.getCopyForWriting();
      estimatorStateHolder.setFromEstimatorModel(timestamp, estimatorTick, estimatorClockStartTime);
      estimatorStateCopier.commit();
   }

   public SDFFullRobotModel getEstimatorFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   public ForceSensorDataHolder getEstimatorForceSensorDataHolder()
   {
      return estimatorForceSensorDataHolder;
   }

   public SDFFullRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   public ForceSensorDataHolder getControllerForceSensorDataHolder()
   {
      return controllerForceSensorDataHolder;
   }

   public long getTimestamp()
   {
      return timestamp;
   }

   public long getEstimatorClockStartTime()
   {
      return estimatorClockStartTime;
   }

}
