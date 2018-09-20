package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket;
import controller_msgs.msg.dds.StateEstimatorModePacket;
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
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.humanoidRobotics.communication.subscribers.StateEstimatorModeSubscriber;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotDataLogger.RobotVisualizer;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.sensors.CenterOfMassDataHolder;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.IMUSensorReadOnly;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.DRCKinematicsBasedStateEstimator;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.KinematicsBasedStateEstimatorFactory;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class DRCEstimatorThread implements MultiThreadedRobotControlElement
{
   private static final boolean USE_FORCE_SENSOR_TO_JOINT_TORQUE_PROJECTOR = false;

   private final YoVariableRegistry estimatorRegistry = new YoVariableRegistry("DRCEstimatorThread");
   private final RobotVisualizer robotVisualizer;
   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final ForceSensorDataHolder forceSensorDataHolderForEstimator;
   private final CenterOfMassDataHolder centerOfMassDataHolderForEstimator;
   private final ContactSensorHolder contactSensorHolder;
   private final ModularRobotController estimatorController;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private final DRCKinematicsBasedStateEstimator drcStateEstimator;

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
   private final CenterOfPressureDataHolder centerOfPressureDataHolderFromController;
   private final RobotMotionStatusHolder robotMotionStatusFromController;
   private final DRCPoseCommunicator poseCommunicator;

   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();
   private final ReferenceFrame rootFrame;

   private final JointDesiredOutputWriter outputWriter;

   private final IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> controllerCrashPublisher;

   public DRCEstimatorThread(String robotName, DRCRobotSensorInformation sensorInformation, RobotContactPointParameters<RobotSide> contactPointParameters,
                             WholeBodyControllerParameters<RobotSide> wholeBodyControllerParameters, StateEstimatorParameters stateEstimatorParameters,
                             SensorReaderFactory sensorReaderFactory, ThreadDataSynchronizerInterface threadDataSynchronizer, RealtimeRos2Node realtimeRos2Node,
                             JointDesiredOutputWriter outputWriter, RobotVisualizer robotVisualizer, double gravity)
   {
      this.threadDataSynchronizer = threadDataSynchronizer;
      this.robotVisualizer = robotVisualizer;
      estimatorFullRobotModel = threadDataSynchronizer.getEstimatorFullRobotModel();
      FloatingInverseDynamicsJoint rootJoint = estimatorFullRobotModel.getRootJoint();
      rootFrame = rootJoint.getFrameAfterJoint();

      forceSensorDataHolderForEstimator = threadDataSynchronizer.getEstimatorForceSensorDataHolder();
      centerOfMassDataHolderForEstimator = threadDataSynchronizer.getEstimatorCenterOfMassDataHolder();
      ContactSensorHolder estimatorContactSensorHolder = threadDataSynchronizer.getEstimatorContactSensorHolder();
      contactSensorHolder = estimatorContactSensorHolder;

      IMUDefinition[] imuDefinitions = estimatorFullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = estimatorFullRobotModel.getForceSensorDefinitions();
      RawJointSensorDataHolderMap estimatorRawJointSensorDataHolderMap = threadDataSynchronizer.getEstimatorRawJointSensorDataHolderMap();
      JointDesiredOutputList estimatorDesiredJointDataHolder = threadDataSynchronizer.getEstimatorDesiredJointDataHolder();

      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, estimatorContactSensorHolder, estimatorRawJointSensorDataHolderMap,
                                estimatorDesiredJointDataHolder, estimatorRegistry);

      sensorReader = sensorReaderFactory.getSensorReader();

      estimatorController = new ModularRobotController("EstimatorController");

      centerOfPressureDataHolderFromController = threadDataSynchronizer.getEstimatorCenterOfPressureDataHolder();
      robotMotionStatusFromController = threadDataSynchronizer.getEstimatorRobotMotionStatusHolder();

      sensorOutputMapReadOnly = sensorReader.getSensorOutputMapReadOnly();
      sensorRawOutputMapReadOnly = sensorReader.getSensorRawOutputMapReadOnly();

      if (realtimeRos2Node != null)
         controllerCrashPublisher = ROS2Tools.createPublisher(realtimeRos2Node, ControllerCrashNotificationPacket.class,
                                                              ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName));
      else
         controllerCrashPublisher = null;

      if (sensorReaderFactory.useStateEstimator())
      {
         KinematicsBasedStateEstimatorFactory estimatorFactory = new KinematicsBasedStateEstimatorFactory();
         ArrayList<String> additionalContactRigidBodyNames = contactPointParameters.getAdditionalContactRigidBodyNames();
         ArrayList<String> additionalContactNames = contactPointParameters.getAdditionalContactNames();
         ArrayList<RigidBodyTransform> additionalContactTransforms = contactPointParameters.getAdditionalContactTransforms();

         ContactableBodiesFactory<RobotSide> contactableBodiesFactory = new ContactableBodiesFactory<>();
         contactableBodiesFactory.setFootContactPoints(contactPointParameters.getFootContactPoints());
         contactableBodiesFactory.setToeContactParameters(contactPointParameters.getControllerToeContactPoints(),
                                                          contactPointParameters.getControllerToeContactLines());
         for (int i = 0; i < contactPointParameters.getAdditionalContactNames().size(); i++)
            contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionalContactNames.get(i),
                                                               additionalContactTransforms.get(i));

         estimatorFactory.setEstimatorFullRobotModel(estimatorFullRobotModel).setSensorInformation(sensorInformation)
                         .setSensorOutputMapReadOnly(sensorOutputMapReadOnly).setGravity(gravity).setStateEstimatorParameters(stateEstimatorParameters)
                         .setContactableBodiesFactory(contactableBodiesFactory).setEstimatorForceSensorDataHolderToUpdate(forceSensorDataHolderForEstimator)
                         .setEstimatorCenterOfMassDataHolderToUpdate(centerOfMassDataHolderForEstimator).setContactSensorHolder(contactSensorHolder)
                         .setCenterOfPressureDataHolderFromController(centerOfPressureDataHolderFromController)
                         .setRobotMotionStatusFromController(robotMotionStatusFromController);

         drcStateEstimator = estimatorFactory.createStateEstimator(estimatorRegistry, yoGraphicsListRegistry);

         if (realtimeRos2Node != null)
         {
            StateEstimatorModeSubscriber stateEstimatorModeSubscriber = new StateEstimatorModeSubscriber();
            RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber = new RequestWristForceSensorCalibrationSubscriber();

            MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, StateEstimatorModePacket.class, subscriberTopicNameGenerator,
                                                 subscriber -> stateEstimatorModeSubscriber.receivedPacket(subscriber.takeNextData()));
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, RequestWristForceSensorCalibrationPacket.class, subscriberTopicNameGenerator,
                                                 subscriber -> requestWristForceSensorCalibrationSubscriber.receivedPacket(subscriber.takeNextData()));

            drcStateEstimator.setOperatingModeSubscriber(stateEstimatorModeSubscriber);
            drcStateEstimator.setRequestWristForceSensorCalibrationSubscriber(requestWristForceSensorCalibrationSubscriber);
         }

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
            RigidBody sensorLink = forceSensorDefinition.getRigidBody();
            ForceSensorData forceSensorData = forceSensorDataHolderForEstimator.get(forceSensorDefinition);
            ForceSensorToJointTorqueProjector footSensorToJointTorqueProjector = new ForceSensorToJointTorqueProjector(sensorName, forceSensorData, sensorLink);
            estimatorController.addRobotController(footSensorToJointTorqueProjector);
         }
      }

      if (realtimeRos2Node != null)
      {
         ForceSensorDataHolderReadOnly forceSensorDataHolderToSend;
         if (drcStateEstimator != null && drcStateEstimator.getForceSensorOutputWithGravityCancelled() != null)
            forceSensorDataHolderToSend = drcStateEstimator.getForceSensorOutputWithGravityCancelled();
         else
            forceSensorDataHolderToSend = forceSensorDataHolderForEstimator;

         JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(estimatorFullRobotModel,
                                                                                                           forceSensorDataHolderToSend);

         poseCommunicator = new DRCPoseCommunicator(estimatorFullRobotModel, jointConfigurationGathererAndProducer, sensorReader,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName), realtimeRos2Node,
                                                    sensorOutputMapReadOnly, sensorRawOutputMapReadOnly, robotMotionStatusFromController, sensorInformation);
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

      ParameterLoaderHelper.loadParameters(this, wholeBodyControllerParameters, estimatorRegistry);

      if (robotVisualizer != null)
      {
         robotVisualizer.setMainRegistry(estimatorRegistry, estimatorFullRobotModel.getElevator(), yoGraphicsListRegistry);
      }
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
            firstTick.set(false);
         }

         estimatorTimer.startMeasurement();
         estimatorController.doControl();
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
      return drcStateEstimator.getForceSensorCalibrationModule();
   }

   @Override
   public long nextWakeupTime()
   {
      throw new RuntimeException("Estimator thread should not wake up based on clock");
   }

   public void initializeEstimator(RigidBodyTransform rootJointTransform)
   {
      if (drcStateEstimator != null)
         drcStateEstimator.initializeEstimator(rootJointTransform);
   }

   public void setExternalPelvisCorrectorSubscriber(PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber)
   {
      if (drcStateEstimator != null)
         drcStateEstimator.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
   }

   public void requestStateEstimatorMode(StateEstimatorMode stateEstimatorMode)
   {
      if (drcStateEstimator != null)
         drcStateEstimator.requestStateEstimatorMode(stateEstimatorMode);
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
