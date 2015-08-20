package us.ihmc.wholeBodyController.concurrent;

import java.util.Arrays;

import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.sensorProcessing.model.DesiredJointDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class ThreadDataSynchronizer implements ThreadDataSynchronizerInterface
{
   private final SDFFullHumanoidRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final ContactSensorHolder estimatorContactSensorHolder;
   private final RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;
   private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;

   private final SDFFullHumanoidRobotModel controllerFullRobotModel;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;
   private final ContactSensorHolder controllerContactSensorHolder;
   private final RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;
   
   private final DesiredJointDataHolder estimatorDesiredJointDataHolder;
   private final DesiredJointDataHolder controllerDesiredJointDataHolder;


   private final ConcurrentCopier<IntermediateEstimatorStateHolder> estimatorStateCopier;

   private final ConcurrentCopier<ControllerDataForEstimatorHolder> controllerStateCopier;

   private long timestamp;
   private long estimatorClockStartTime;
   private long estimatorTick;

   public ThreadDataSynchronizer(WholeBodyControllerParameters wholeBodyControlParameters)
   {
      estimatorFullRobotModel = wholeBodyControlParameters.createFullRobotModel();
      estimatorForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(estimatorFullRobotModel.getForceSensorDefinitions()));
      estimatorRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(estimatorFullRobotModel);
      estimatorContactSensorHolder = new ContactSensorHolder(Arrays.asList(estimatorFullRobotModel.getContactSensorDefinitions()));
      estimatorCenterOfPressureDataHolder = new CenterOfPressureDataHolder(estimatorFullRobotModel.getSoleFrames());
      estimatorRobotMotionStatusHolder = new RobotMotionStatusHolder();
      estimatorDesiredJointDataHolder = new DesiredJointDataHolder(estimatorFullRobotModel.getOneDoFJoints());

      controllerFullRobotModel = wholeBodyControlParameters.createFullRobotModel();
      controllerForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(controllerFullRobotModel.getForceSensorDefinitions()));
      controllerContactSensorHolder = new ContactSensorHolder(Arrays.asList(controllerFullRobotModel.getContactSensorDefinitions()));
      controllerRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(controllerFullRobotModel);
      controllerCenterOfPressureDataHolder =  new CenterOfPressureDataHolder(controllerFullRobotModel.getSoleFrames());
      controllerRobotMotionStatusHolder = new RobotMotionStatusHolder();
      controllerDesiredJointDataHolder = new DesiredJointDataHolder(controllerFullRobotModel.getOneDoFJoints());

      IntermediateEstimatorStateHolder.Builder stateCopierBuilder = new IntermediateEstimatorStateHolder.Builder(wholeBodyControlParameters,
            estimatorFullRobotModel.getElevator(), controllerFullRobotModel.getElevator(), estimatorForceSensorDataHolder, controllerForceSensorDataHolder,
            estimatorContactSensorHolder, controllerContactSensorHolder, estimatorRawJointSensorDataHolderMap, controllerRawJointSensorDataHolderMap);
      estimatorStateCopier = new ConcurrentCopier<IntermediateEstimatorStateHolder>(stateCopierBuilder);

      ControllerDataForEstimatorHolder.Builder controllerStateCopierBuilder = new ControllerDataForEstimatorHolder.Builder(estimatorCenterOfPressureDataHolder,
            controllerCenterOfPressureDataHolder, estimatorRobotMotionStatusHolder, controllerRobotMotionStatusHolder, estimatorDesiredJointDataHolder, controllerDesiredJointDataHolder);
      controllerStateCopier = new ConcurrentCopier<>(controllerStateCopierBuilder);
   }

   @Override
   public boolean receiveEstimatorStateForController()
   {
      IntermediateEstimatorStateHolder estimatorStateHolder = estimatorStateCopier.getCopyForReading();
      if (estimatorStateHolder != null)
      {
         estimatorStateHolder.getIntoControllerModel();
         timestamp = estimatorStateHolder.getTimestamp();
         estimatorClockStartTime = estimatorStateHolder.getEstimatorClockStartTime();
         estimatorTick = estimatorStateHolder.getEstimatorTick();
         return true;
      }
      else
      {
         return false;
      }
   }

   @Override
   public void publishEstimatorState(long timestamp, long estimatorTick, long estimatorClockStartTime)
   {      
      IntermediateEstimatorStateHolder estimatorStateHolder = estimatorStateCopier.getCopyForWriting();
      estimatorStateHolder.setFromEstimatorModel(timestamp, estimatorTick, estimatorClockStartTime);
      estimatorStateCopier.commit();
   }

   @Override
   public SDFFullHumanoidRobotModel getEstimatorFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   @Override
   public ForceSensorDataHolder getEstimatorForceSensorDataHolder()
   {
      return estimatorForceSensorDataHolder;
   }

   @Override
   public SDFFullHumanoidRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   @Override
   public ForceSensorDataHolder getControllerForceSensorDataHolder()
   {
      return controllerForceSensorDataHolder;
   }
   
   @Override
   public ContactSensorHolder getControllerContactSensorHolder()
   {
      return controllerContactSensorHolder;
   }
   
   @Override
   public ContactSensorHolder getEstimatorContactSensorHolder()
   {
      return estimatorContactSensorHolder;
   }

   @Override
   public RawJointSensorDataHolderMap getEstimatorRawJointSensorDataHolderMap()
   {
      return estimatorRawJointSensorDataHolderMap;
   }

   @Override
   public RawJointSensorDataHolderMap getControllerRawJointSensorDataHolderMap()
   {
      return controllerRawJointSensorDataHolderMap;
   }

   @Override
   public CenterOfPressureDataHolder getEstimatorCenterOfPressureDataHolder()
   {
      return estimatorCenterOfPressureDataHolder;
   }

   @Override
   public CenterOfPressureDataHolder getControllerCenterOfPressureDataHolder()
   {
      return controllerCenterOfPressureDataHolder;
   }

   @Override
   public RobotMotionStatusHolder getEstimatorRobotMotionStatusHolder()
   {
      return estimatorRobotMotionStatusHolder;
   }

   @Override
   public RobotMotionStatusHolder getControllerRobotMotionStatusHolder()
   {
      return controllerRobotMotionStatusHolder;
   }

   @Override
   public DesiredJointDataHolder getEstimatorDesiredJointDataHolder()
   {
      return estimatorDesiredJointDataHolder;
   }
   
   @Override
   public long getTimestamp()
   {
      return timestamp;
   }

   @Override
   public long getEstimatorClockStartTime()
   {
      return estimatorClockStartTime;
   }

   @Override
   public long getEstimatorTick()
   {
      return estimatorTick;
   }

   @Override
   public void publishControllerData()
   {
      ControllerDataForEstimatorHolder holder = controllerStateCopier.getCopyForWriting();
      if (holder != null)
      {
         holder.writeControllerDataFromController();
         controllerStateCopier.commit();
      }
   }

   @Override
   public boolean receiveControllerDataForEstimator()
   {
      ControllerDataForEstimatorHolder holder = controllerStateCopier.getCopyForReading();
      if (holder != null)
      {
         holder.readControllerDataIntoEstimator();
         return true;
      }
      else
      {
         return false;
      }
   }

}
