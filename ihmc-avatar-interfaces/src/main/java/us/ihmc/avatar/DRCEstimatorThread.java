package us.ihmc.avatar;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket;
import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.ForceSensorToJointTorqueProjector;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.ekf.HumanoidRobotEKFWithSimpleJoints;
import us.ihmc.stateEstimation.ekf.LeggedRobotEKF;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorStateUpdater;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.KinematicsBasedStateEstimatorFactory;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class DRCEstimatorThread implements MultiThreadedRobotControlElement
{
   private static final boolean USE_FORCE_SENSOR_TO_JOINT_TORQUE_PROJECTOR = false;

   /** Set this to true to create and run, but not use the EKF estimator */
   private static final boolean CREATE_EKF_ESTIMATOR = false;
   /** Set this to true to use the EKF estimator */
   private static final boolean USE_EKF_ESTIMATOR = false;

   private final YoVariableRegistry estimatorRegistry = new YoVariableRegistry("DRCEstimatorThread");
   private final RobotVisualizer robotVisualizer;
   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder forceSensorDataHolderForEstimator;
   private final ModularRobotController estimatorController;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final StateEstimatorController drcStateEstimator;
   private final StateEstimatorController ekfStateEstimator;
   private final YoBoolean reinitializeEKF;

   private final ThreadDataSynchronizerInterface threadDataSynchronizer;
   private final SensorReader sensorReader;

   private final YoLong estimatorTime = new YoLong("estimatorTime", estimatorRegistry);
   private final YoLong estimatorTick = new YoLong("estimatorTick", estimatorRegistry);
   private final YoBoolean firstTick = new YoBoolean("firstTick", estimatorRegistry);
   private final YoBoolean outputWriterInitialized = new YoBoolean("outputWriterInitialized", estimatorRegistry);
   private final YoBoolean controllerDataValid = new YoBoolean("controllerDataValid", estimatorRegistry);

   private final YoLong startClockTime = new YoLong("startTime", estimatorRegistry);
   private final ExecutionTimer estimatorTimer = new ExecutionTimer("estimatorTimer", 10.0, estimatorRegistry);

   private long lastReadSystemTime = 0L;
   private final YoDouble actualEstimatorDT = new YoDouble("actualEstimatorDTInMillis", estimatorRegistry);

   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   private final RobotMotionStatusHolder robotMotionStatusFromController;
   private final DRCPoseCommunicator poseCommunicator;

   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame rootFrame;

   private final JointDesiredOutputWriter outputWriter;

   private final IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> controllerCrashPublisher;

   private final ForceSensorStateUpdater forceSensorStateUpdater;

   public DRCEstimatorThread(String robotName, HumanoidRobotSensorInformation sensorInformation, RobotContactPointParameters<RobotSide> contactPointParameters,
                             DRCRobotModel robotModel, StateEstimatorParameters stateEstimatorParameters, SensorReaderFactory sensorReaderFactory,
                             ThreadDataSynchronizerInterface threadDataSynchronizer, RealtimeRos2Node realtimeRos2Node,
                             PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber, JointDesiredOutputWriter outputWriter,
                             RobotVisualizer robotVisualizer, double gravity)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.robotVisualizer = robotVisualizer;
      estimatorFullRobotModel = threadDataSynchronizer.getEstimatorFullRobotModel();
      FloatingJointBasics rootJoint = estimatorFullRobotModel.getRootJoint();
      rootFrame = rootJoint.getFrameAfterJoint();

      forceSensorDataHolderForEstimator = threadDataSynchronizer.getEstimatorForceSensorDataHolder();

      IMUDefinition[] imuDefinitions = estimatorFullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = estimatorFullRobotModel.getForceSensorDefinitions();
      JointDesiredOutputListBasics estimatorDesiredJointDataHolder = threadDataSynchronizer.getEstimatorDesiredJointDataHolder();

      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, estimatorDesiredJointDataHolder, estimatorRegistry);

      sensorReader = sensorReaderFactory.getSensorReader();

      estimatorController = new ModularRobotController("EstimatorController");

      robotMotionStatusFromController = threadDataSynchronizer.getEstimatorRobotMotionStatusHolder();

      sensorOutputMapReadOnly = sensorReader.getSensorOutputMapReadOnly();
      sensorRawOutputMapReadOnly = sensorReader.getSensorRawOutputMapReadOnly();

      if (realtimeRos2Node != null)
         controllerCrashPublisher = ROS2Tools.createPublisher(realtimeRos2Node,
                                                              ControllerCrashNotificationPacket.class,
                                                              ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName));
      else
         controllerCrashPublisher = null;

      if (sensorReaderFactory.useStateEstimator())
      {
         if (forceSensorDataHolderForEstimator != null)
         {
            forceSensorStateUpdater = new ForceSensorStateUpdater(estimatorFullRobotModel.getRootJoint(),
                                                                  sensorOutputMapReadOnly,
                                                                  forceSensorDataHolderForEstimator,
                                                                  stateEstimatorParameters,
                                                                  gravity,
                                                                  robotMotionStatusFromController,
                                                                  yoGraphicsListRegistry,
                                                                  estimatorRegistry);
         }
         else
         {
            forceSensorStateUpdater = null;
         }

         if (realtimeRos2Node != null)
         {
            RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber = new RequestWristForceSensorCalibrationSubscriber();
            MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
            ROS2Tools.createCallbackSubscription(realtimeRos2Node,
                                                 RequestWristForceSensorCalibrationPacket.class,
                                                 subscriberTopicNameGenerator,
                                                 subscriber -> requestWristForceSensorCalibrationSubscriber.receivedPacket(subscriber.takeNextData()));
            forceSensorStateUpdater.setRequestWristForceSensorCalibrationSubscriber(requestWristForceSensorCalibrationSubscriber);
         }
      }
      else
      {
         forceSensorStateUpdater = null;
      }

      if (sensorReaderFactory.useStateEstimator() && !USE_EKF_ESTIMATOR)
      {
         // Create DRC Estimator:
         KinematicsBasedStateEstimatorFactory estimatorFactory = new KinematicsBasedStateEstimatorFactory();
         ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
         ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
         ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();
         ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
         contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
         contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                          contactPointParameters.getControllerToeContactLines());
         for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
            contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i),
                                                               additionalContactNames.get(i),
                                                               additionalContactTransforms.get(i));
         CenterOfPressureDataHolder centerOfPressureDataHolderFromController = threadDataSynchronizer.getEstimatorCenterOfPressureDataHolder();
         CenterOfMassDataHolder centerOfMassDataHolderForEstimator = threadDataSynchronizer.getEstimatorCenterOfMassDataHolder();
         estimatorFactory.setEstimatorFullRobotModel(estimatorFullRobotModel).setSensorInformation(sensorInformation)
                         .setSensorOutputMapReadOnly(sensorOutputMapReadOnly).setGravity(gravity).setStateEstimatorParameters(stateEstimatorParameters)
                         .setContactableBodiesFactory(contactableBodiesFactory).setEstimatorForceSensorDataHolderToUpdate(forceSensorDataHolderForEstimator)
                         .setEstimatorCenterOfMassDataHolderToUpdate(centerOfMassDataHolderForEstimator)
                         .setCenterOfPressureDataHolderFromController(centerOfPressureDataHolderFromController)
                         .setRobotMotionStatusFromController(robotMotionStatusFromController);
         drcStateEstimator = estimatorFactory.createStateEstimator(estimatorRegistry, yoGraphicsListRegistry);
         estimatorController.addRobotController(drcStateEstimator);
      }
      else
      {
         drcStateEstimator = null;
      }

      RobotJointLimitWatcher robotJointLimitWatcher = new RobotJointLimitWatcher(estimatorFullRobotModel.getOneDoFJoints(), sensorRawOutputMapReadOnly);
      estimatorController.addRobotController(robotJointLimitWatcher);

      if (USE_FORCE_SENSOR_TO_JOINT_TORQUE_PROJECTOR)
      {
         for (ForceSensorDefinition forceSensorDefinition : forceSensorDataHolderForEstimator.getForceSensorDefinitions())
         {
            String sensorName = forceSensorDefinition.getSensorName();
            RigidBodyBasics sensorLink = forceSensorDefinition.getRigidBody();
            ForceSensorData forceSensorData = forceSensorDataHolderForEstimator.get(forceSensorDefinition);
            ForceSensorToJointTorqueProjector footSensorToJointTorqueProjector = new ForceSensorToJointTorqueProjector(sensorName, forceSensorData, sensorLink);
            estimatorController.addRobotController(footSensorToJointTorqueProjector);
         }
      }

      if (realtimeRos2Node != null)
      {
         ForceSensorDataHolderReadOnly forceSensorDataHolderToSend;
         if (sensorReaderFactory.useStateEstimator() && forceSensorStateUpdater.getForceSensorOutputWithGravityCancelled() != null)
            forceSensorDataHolderToSend = forceSensorStateUpdater.getForceSensorOutputWithGravityCancelled();
         else
            forceSensorDataHolderToSend = forceSensorDataHolderForEstimator;

         JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(estimatorFullRobotModel,
                                                                                                           forceSensorDataHolderToSend);

         poseCommunicator = new DRCPoseCommunicator(estimatorFullRobotModel,
                                                    jointConfigurationGathererAndProducer,
                                                    sensorReader,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName),
                                                    realtimeRos2Node,
                                                    sensorOutputMapReadOnly,
                                                    sensorRawOutputMapReadOnly,
                                                    robotMotionStatusFromController,
                                                    sensorInformation);
         estimatorController.setRawOutputWriter(poseCommunicator);
      }
      else
      {
         poseCommunicator = null;
      }

      firstTick.set(true);
      outputWriterInitialized.set(false);
      controllerDataValid.set(false);

      estimatorRegistry.addChild(estimatorController.getYoVariableRegistry());

      this.outputWriter = outputWriter;
      if (this.outputWriter != null)
      {
         this.outputWriter.setForceSensorDataHolder(forceSensorDataHolderForEstimator);
         this.outputWriter.setJointDesiredOutputList(estimatorDesiredJointDataHolder);
         if (this.outputWriter.getYoVariableRegistry() != null)
         {
            estimatorRegistry.addChild(this.outputWriter.getYoVariableRegistry());
         }

      }

      ParameterLoaderHelper.loadParameters(this, robotModel, estimatorRegistry);

      // Create EKF Estimator:
      if (sensorReaderFactory.useStateEstimator() && USE_EKF_ESTIMATOR)
      {
         reinitializeEKF = null;

         double estimatorDT = stateEstimatorParameters.getEstimatorDT();
         SideDependentList<String> footForceSensorNames = sensorInformation.getFeetForceSensorNames();
         String primaryImuName = sensorInformation.getPrimaryBodyImu();
         Collection<String> imuSensorNames = Arrays.asList(sensorInformation.getIMUSensorsToUseInStateEstimator());
         ekfStateEstimator = new HumanoidRobotEKFWithSimpleJoints(estimatorFullRobotModel,
                                                                  primaryImuName,
                                                                  imuSensorNames,
                                                                  footForceSensorNames,
                                                                  sensorRawOutputMapReadOnly,
                                                                  estimatorDT,
                                                                  gravity,
                                                                  sensorOutputMapReadOnly,
                                                                  yoGraphicsListRegistry,
                                                                  estimatorFullRobotModel);

         InputStream ekfParameterStream = LeggedRobotEKF.class.getResourceAsStream("/ekf.xml");
         if (ekfParameterStream == null)
         {
            throw new RuntimeException("Did not find parameter file for EKF.");
         }
         ParameterLoaderHelper.loadParameters(this, ekfParameterStream, ekfStateEstimator.getYoVariableRegistry());

         estimatorController.addRobotController(ekfStateEstimator);
      }
      else if (sensorReaderFactory.useStateEstimator() && CREATE_EKF_ESTIMATOR)
      {
         reinitializeEKF = new YoBoolean("ReinitializeEKF", estimatorRegistry);

         FullHumanoidRobotModel ekfFullRobotModel = robotModel.createFullRobotModel();
         double estimatorDT = stateEstimatorParameters.getEstimatorDT();
         SideDependentList<String> footForceSensorNames = sensorInformation.getFeetForceSensorNames();
         String primaryImuName = sensorInformation.getPrimaryBodyImu();
         Collection<String> imuSensorNames = Arrays.asList(sensorInformation.getIMUSensorsToUseInStateEstimator());
         ekfStateEstimator = new HumanoidRobotEKFWithSimpleJoints(ekfFullRobotModel,
                                                                  primaryImuName,
                                                                  imuSensorNames,
                                                                  footForceSensorNames,
                                                                  sensorRawOutputMapReadOnly,
                                                                  estimatorDT,
                                                                  gravity,
                                                                  sensorOutputMapReadOnly,
                                                                  yoGraphicsListRegistry,
                                                                  estimatorFullRobotModel);

         InputStream ekfParameterStream = LeggedRobotEKF.class.getResourceAsStream("/ekf.xml");
         if (ekfParameterStream == null)
         {
            throw new RuntimeException("Did not find parameter file for EKF.");
         }
         ParameterLoaderHelper.loadParameters(this, ekfParameterStream, ekfStateEstimator.getYoVariableRegistry());

         estimatorController.addRobotController(ekfStateEstimator);
      }
      else
      {
         reinitializeEKF = null;
         ekfStateEstimator = null;
      }

      if (robotVisualizer != null)
      {
         robotVisualizer.setMainRegistry(estimatorRegistry, estimatorFullRobotModel.getElevator(), yoGraphicsListRegistry);
      }
   }

   public void setupHighLevelControllerCallback(String robotName, RealtimeRos2Node realtimeRos2Node,
                                                Map<HighLevelControllerName, StateEstimatorMode> stateModeMap)
   {
      MessageTopicNameGenerator publisherTopicNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, publisherTopicNameGenerator, subscriber ->
      {
         HighLevelStateChangeStatusMessage message = subscriber.takeNextData();
         LogTools.debug("Estimator recieved message: controller going to {}", HighLevelControllerName.fromByte(message.getEndHighLevelControllerName()));
         StateEstimatorMode requestedMode = stateModeMap.get(HighLevelControllerName.fromByte(message.getEndHighLevelControllerName()));
         LogTools.debug("Estimator going to {}", requestedMode);

         if (drcStateEstimator != null)
            drcStateEstimator.requestStateEstimatorMode(requestedMode);
         if (ekfStateEstimator != null)
            ekfStateEstimator.requestStateEstimatorMode(requestedMode);
      });
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return estimatorRegistry;
   }

   @Override
   public String getName()
   {
      return estimatorRegistry.getName();
   }

   @Override
   public void read(long currentClockTime)
   {
      try
      {
         long nanoTime = System.nanoTime();
         actualEstimatorDT.set(Conversions.nanosecondsToMilliseconds((double) (nanoTime - lastReadSystemTime)));
         lastReadSystemTime = nanoTime;

         startClockTime.set(currentClockTime);

         controllerDataValid.set(threadDataSynchronizer.receiveControllerDataForEstimator());

         if (outputWriter != null)
         {
            if (controllerDataValid.getBooleanValue())
            {
               if (!outputWriterInitialized.getBooleanValue())
               {
                  outputWriter.initialize();
                  outputWriterInitialized.set(true);
               }

               outputWriter.writeBefore(currentClockTime);
            }
         }

         sensorReader.read();

         estimatorTime.set(sensorOutputMapReadOnly.getTimestamp());
      }
      catch (Throwable e)
      {
         if (controllerCrashPublisher != null)
         {
            controllerCrashPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.ESTIMATOR_READ, e.getMessage()));
         }
         throw new RuntimeException(e);
      }
   }

   @Override
   public void run()
   {
      try
      {
         if (firstTick.getBooleanValue())
         {
            estimatorController.initialize();
            if (forceSensorStateUpdater != null)
            {
               forceSensorStateUpdater.initialize();
            }
            firstTick.set(false);
         }

         estimatorTimer.startMeasurement();
         if (reinitializeEKF != null && reinitializeEKF.getValue())
         {
            reinitializeEKF.set(false);
            estimatorFullRobotModel.getRootJoint().getJointConfiguration(rootToWorldTransform);
            ekfStateEstimator.initializeEstimator(rootToWorldTransform);
         }
         estimatorController.doControl();
         if (forceSensorStateUpdater != null)
         {
            forceSensorStateUpdater.updateForceSensorState();
         }
         estimatorTimer.stopMeasurement();
      }
      catch (Throwable e)
      {
         if (controllerCrashPublisher != null)
         {
            controllerCrashPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.ESTIMATOR_RUN, e.getMessage()));
         }
         throw new RuntimeException(e);
      }
   }

   @Override
   public void write(long timestamp)
   {
      try
      {
         if (outputWriter != null)
         {
            if (controllerDataValid.getBooleanValue())
            {
               outputWriter.writeAfter();
            }
         }

         long startTimestamp = estimatorTime.getLongValue();
         threadDataSynchronizer.publishEstimatorState(startTimestamp, estimatorTick.getLongValue(), startClockTime.getLongValue());
         if (robotVisualizer != null)
         {
            robotVisualizer.update(startTimestamp);
         }
         estimatorTick.increment();

         rootFrame.getTransformToDesiredFrame(rootToWorldTransform, ReferenceFrame.getWorldFrame());
         yoGraphicsListRegistry.setControllerTransformToWorld(rootToWorldTransform);
      }
      catch (Throwable e)
      {
         if (controllerCrashPublisher != null)
         {
            controllerCrashPublisher.publish(MessageTools.createControllerCrashNotificationPacket(ControllerCrashLocation.ESTIMATOR_WRITE, e.getMessage()));
         }
         throw new RuntimeException(e);
      }
   }

   @Override
   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public ForceSensorCalibrationModule getForceSensorCalibrationModule()
   {
      return forceSensorStateUpdater;
   }

   @Override
   public long nextWakeupTime()
   {
      throw new RuntimeException("Estimator thread should not wake up based on clock");
   }

   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      if (ekfStateEstimator != null)
         ekfStateEstimator.initializeEstimator(rootJointTransform, jointPositions);
      if (drcStateEstimator != null)
         drcStateEstimator.initializeEstimator(rootJointTransform, jointPositions);
   }

   public List<? extends IMUSensorReadOnly> getSimulatedIMUOutput()
   {
      return sensorOutputMapReadOnly.getIMUProcessedOutputs();
   }

   public void dispose()
   {
   }

   /**
    * used primarily for unit tests, but could be useful.
    */
   public void addRobotController(RobotController controller)
   {
      estimatorController.addRobotController(controller);
   }
}
