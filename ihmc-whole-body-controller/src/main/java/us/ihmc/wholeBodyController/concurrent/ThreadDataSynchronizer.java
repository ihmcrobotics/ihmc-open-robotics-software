package us.ihmc.wholeBodyController.concurrent;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.CenterOfMassDataHolderReadOnly;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;

public class ThreadDataSynchronizer implements ThreadDataSynchronizerInterface
{
   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder estimatorForceSensorDataHolder;
   private final CenterOfMassDataHolder estimatorCenterOfMassDataHolder;
   private final ContactSensorHolder estimatorContactSensorHolder;
   private final RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder estimatorCenterOfPressureDataHolder;
   private final RobotMotionStatusHolder estimatorRobotMotionStatusHolder;

   private final FullHumanoidRobotModel controllerFullRobotModel;
   private final ForceSensorDataHolder controllerForceSensorDataHolder;
   private final CenterOfMassDataHolder controllerCenterOfMassDataHolder;
   private final ContactSensorHolder controllerContactSensorHolder;
   private final RawJointSensorDataHolderMap controllerRawJointSensorDataHolderMap;
   private final CenterOfPressureDataHolder controllerCenterOfPressureDataHolder;
   private final RobotMotionStatusHolder controllerRobotMotionStatusHolder;

   private final JointDesiredOutputList estimatorDesiredJointDataHolder;
   private final JointDesiredOutputList controllerDesiredJointDataHolder;

   private final ConcurrentCopier<IntermediateEstimatorStateHolder> estimatorStateCopier;
   private final ConcurrentCopier<ControllerDataForEstimatorHolder> controllerStateCopier;

   private long timestamp;
   private long estimatorClockStartTime;
   private long estimatorTick;

   public ThreadDataSynchronizer(FullHumanoidRobotModelFactory robotModelFactory)
   {
      estimatorFullRobotModel = robotModelFactory.createFullRobotModel();
      estimatorForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(estimatorFullRobotModel.getForceSensorDefinitions()));
      estimatorCenterOfMassDataHolder = new CenterOfMassDataHolder();
      estimatorRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(estimatorFullRobotModel);
      estimatorContactSensorHolder = new ContactSensorHolder(Arrays.asList(estimatorFullRobotModel.getContactSensorDefinitions()));
      estimatorRobotMotionStatusHolder = new RobotMotionStatusHolder();
      estimatorDesiredJointDataHolder = new JointDesiredOutputList(estimatorFullRobotModel.getControllableOneDoFJoints());

      List<ReferenceFrame> estimatorSoleFrames = new ArrayList<>();
      List<RigidBodyBasics> estimatorFeet = new ArrayList<>();
      for(RobotSide robotSide : RobotSide.values)
      {
         estimatorSoleFrames.add(estimatorFullRobotModel.getSoleFrame(robotSide));
         estimatorFeet.add(estimatorFullRobotModel.getFoot(robotSide));
      }
      estimatorCenterOfPressureDataHolder = new CenterOfPressureDataHolder(estimatorFeet);

      controllerFullRobotModel = robotModelFactory.createFullRobotModel();
      controllerForceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(controllerFullRobotModel.getForceSensorDefinitions()));
      controllerCenterOfMassDataHolder = new CenterOfMassDataHolder();
      controllerContactSensorHolder = new ContactSensorHolder(Arrays.asList(controllerFullRobotModel.getContactSensorDefinitions()));
      controllerRawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(controllerFullRobotModel);
      controllerRobotMotionStatusHolder = new RobotMotionStatusHolder();
      controllerDesiredJointDataHolder = new JointDesiredOutputList(controllerFullRobotModel.getControllableOneDoFJoints());

      List<ReferenceFrame> controllerSoleFrames = new ArrayList<>();
      List<RigidBodyBasics> controllerFeet = new ArrayList<>();
      for(RobotSide robotSide : RobotSide.values)
      {
         controllerSoleFrames.add(controllerFullRobotModel.getSoleFrame(robotSide));
         controllerFeet.add(controllerFullRobotModel.getFoot(robotSide));
      }
      controllerCenterOfPressureDataHolder = new CenterOfPressureDataHolder(controllerFeet);

      IntermediateEstimatorStateHolder.Builder stateCopierBuilder = new IntermediateEstimatorStateHolder.Builder(robotModelFactory,
            estimatorFullRobotModel.getElevator(), controllerFullRobotModel.getElevator(), estimatorForceSensorDataHolder, controllerForceSensorDataHolder,
            estimatorCenterOfMassDataHolder, controllerCenterOfMassDataHolder,
            estimatorContactSensorHolder, controllerContactSensorHolder, estimatorRawJointSensorDataHolderMap, controllerRawJointSensorDataHolderMap);
      estimatorStateCopier = new ConcurrentCopier<IntermediateEstimatorStateHolder>(stateCopierBuilder);

      ControllerDataForEstimatorHolder.Builder controllerStateCopierBuilder = new ControllerDataForEstimatorHolder.Builder(estimatorCenterOfPressureDataHolder,
                                                                                                                           controllerCenterOfPressureDataHolder,
                                                                                                                           estimatorRobotMotionStatusHolder,
                                                                                                                           controllerRobotMotionStatusHolder,
                                                                                                                           estimatorDesiredJointDataHolder,
                                                                                                                           controllerDesiredJointDataHolder,
                                                                                                                           estimatorSoleFrames,
                                                                                                                           controllerSoleFrames);
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
   public FullHumanoidRobotModel getEstimatorFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   @Override
   public ForceSensorDataHolder getEstimatorForceSensorDataHolder()
   {
      return estimatorForceSensorDataHolder;
   }

   @Override
   public CenterOfMassDataHolder getEstimatorCenterOfMassDataHolder()
   {
      return estimatorCenterOfMassDataHolder;
   }

   @Override
   public FullHumanoidRobotModel getControllerFullRobotModel()
   {
      return controllerFullRobotModel;
   }

   @Override
   public ForceSensorDataHolder getControllerForceSensorDataHolder()
   {
      return controllerForceSensorDataHolder;
   }

   @Override
   public CenterOfMassDataHolderReadOnly getControllerCenterOfMassDataHolder()
   {
      return controllerCenterOfMassDataHolder;
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
   public JointDesiredOutputList getEstimatorDesiredJointDataHolder()
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

   @Override
   public JointDesiredOutputList getControllerDesiredJointDataHolder()
   {
      return controllerDesiredJointDataHolder;
   }


}
