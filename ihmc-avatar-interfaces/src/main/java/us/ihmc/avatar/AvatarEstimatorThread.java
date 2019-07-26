package us.ihmc.avatar;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Map;

import controller_msgs.msg.dds.ControllerCrashNotificationPacket;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.RequestWristForceSensorCalibrationPacket;
import gnu.trove.map.TObjectDoubleMap;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextDataFactory;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextTools;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.ControllerCrashLocation;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.sensing.StateEstimatorMode;
import us.ihmc.humanoidRobotics.communication.subscribers.PelvisPoseCorrectionCommunicatorInterface;
import us.ihmc.humanoidRobotics.communication.subscribers.RequestWristForceSensorCalibrationSubscriber;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.ModularRobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.ros2.RealtimeRos2Node;
import us.ihmc.sensorProcessing.communication.producers.DRCPoseCommunicator;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputWriter;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;
import us.ihmc.sensorProcessing.sensorData.JointConfigurationGatherer;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.sensorProcessing.sensorProcessors.SensorOutputMapReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.SensorRawOutputMapReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.stateEstimation.ekf.HumanoidRobotEKFWithSimpleJoints;
import us.ihmc.stateEstimation.ekf.LeggedRobotEKF;
import us.ihmc.stateEstimation.humanoid.StateEstimatorController;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorCalibrationModule;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ForceSensorStateUpdater;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.KinematicsBasedStateEstimatorFactory;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.parameters.ParameterLoaderHelper;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AvatarEstimatorThread
{
   /** Set this to true to create and run, but not use the EKF estimator */
   private static final boolean CREATE_EKF_ESTIMATOR = false;
   /** Set this to true to use the EKF estimator */
   private static final boolean USE_EKF_ESTIMATOR = false;

   private final YoVariableRegistry estimatorRegistry = new YoVariableRegistry("DRCEstimatorThread");
   private final FullHumanoidRobotModel estimatorFullRobotModel;
   private final ModularRobotController estimatorController;
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final StateEstimatorController drcStateEstimator;
   private final StateEstimatorController ekfStateEstimator;
   private final YoBoolean reinitializeEKF;

   private final SensorReader sensorReader;

   private final YoBoolean firstTick = new YoBoolean("firstTick", estimatorRegistry);

   private final SensorOutputMapReadOnly sensorOutputMapReadOnly;
   private final SensorRawOutputMapReadOnly sensorRawOutputMapReadOnly;
   private final DRCPoseCommunicator poseCommunicator;

   private final RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();

   private final IHMCRealtimeROS2Publisher<ControllerCrashNotificationPacket> controllerCrashPublisher;

   private final ForceSensorStateUpdater forceSensorStateUpdater;

   private final HumanoidRobotContextData humanoidRobotContextData;

   @SuppressWarnings("unused")
   public AvatarEstimatorThread(String robotName, HumanoidRobotSensorInformation sensorInformation,
                                RobotContactPointParameters<RobotSide> contactPointParameters, DRCRobotModel robotModel,
                                StateEstimatorParameters stateEstimatorParameters, SensorReaderFactory sensorReaderFactory,
                                HumanoidRobotContextDataFactory contextDataFactory, RealtimeRos2Node realtimeRos2Node,
                                PelvisPoseCorrectionCommunicatorInterface externalPelvisPoseSubscriber, JointDesiredOutputWriter outputWriter, double gravity)
   {
      estimatorFullRobotModel = robotModel.createFullRobotModel();

      HumanoidRobotContextJointData processedJointData = new HumanoidRobotContextJointData(estimatorFullRobotModel.getOneDoFJoints().length);
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(estimatorFullRobotModel.getForceSensorDefinitions()));
      CenterOfPressureDataHolder centerOfPressureDataHolderFromController = new CenterOfPressureDataHolder(estimatorFullRobotModel);
      LowLevelOneDoFJointDesiredDataHolder desiredJointDataHolder = new LowLevelOneDoFJointDesiredDataHolder(estimatorFullRobotModel.getControllableOneDoFJoints());
      RobotMotionStatusHolder robotMotionStatusFromController = new RobotMotionStatusHolder();
      contextDataFactory.setForceSensorDataHolder(forceSensorDataHolder);
      contextDataFactory.setCenterOfPressureDataHolder(centerOfPressureDataHolderFromController);
      contextDataFactory.setRobotMotionStatusHolder(robotMotionStatusFromController);
      contextDataFactory.setJointDesiredOutputList(desiredJointDataHolder);
      contextDataFactory.setProcessedJointData(processedJointData);
      contextDataFactory.setSensorDataContext(new SensorDataContext(estimatorFullRobotModel));
      humanoidRobotContextData = contextDataFactory.createHumanoidRobotContextData();

      IMUDefinition[] imuDefinitions = estimatorFullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = estimatorFullRobotModel.getForceSensorDefinitions();

      // This is only used by the perfect sensor reader if we are not using the estimator. It will update the data structure.
      sensorReaderFactory.setForceSensorDataHolder(forceSensorDataHolder);

      FloatingJointBasics rootJoint = estimatorFullRobotModel.getRootJoint();
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, desiredJointDataHolder, estimatorRegistry);
      sensorReader = sensorReaderFactory.getSensorReader();

      estimatorController = new ModularRobotController("EstimatorController");


      sensorOutputMapReadOnly = sensorReader.getSensorOutputMapReadOnly();
      sensorRawOutputMapReadOnly = sensorReader.getSensorRawOutputMapReadOnly();

      if (realtimeRos2Node != null)
         controllerCrashPublisher = ROS2Tools.createPublisher(realtimeRos2Node, ControllerCrashNotificationPacket.class,
                                                              ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName));
      else
         controllerCrashPublisher = null;

      if (sensorReaderFactory.useStateEstimator())
      {
         // Updates the force sensor data when running with the estimator.
         forceSensorStateUpdater = new ForceSensorStateUpdater(estimatorFullRobotModel.getRootJoint(), sensorOutputMapReadOnly,
                                                               forceSensorDataHolder, stateEstimatorParameters, gravity,
                                                               robotMotionStatusFromController, yoGraphicsListRegistry, estimatorRegistry);

         if (realtimeRos2Node != null)
         {
            RequestWristForceSensorCalibrationSubscriber requestWristForceSensorCalibrationSubscriber = new RequestWristForceSensorCalibrationSubscriber();
            MessageTopicNameGenerator subscriberTopicNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotName);
            ROS2Tools.createCallbackSubscription(realtimeRos2Node, RequestWristForceSensorCalibrationPacket.class, subscriberTopicNameGenerator,
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
            contactableBodiesFactory.addAdditionalContactPoint(additionalContactRigidBodyNames.get(i), additionalContactNames.get(i),
                                                               additionalContactTransforms.get(i));
         estimatorFactory.setEstimatorFullRobotModel(estimatorFullRobotModel).setSensorInformation(sensorInformation)
                         .setSensorOutputMapReadOnly(sensorOutputMapReadOnly).setGravity(gravity).setStateEstimatorParameters(stateEstimatorParameters)
                         .setContactableBodiesFactory(contactableBodiesFactory).setEstimatorForceSensorDataHolder(forceSensorDataHolder)
                         .setCenterOfPressureDataHolderFromController(centerOfPressureDataHolderFromController)
                         .setRobotMotionStatusFromController(robotMotionStatusFromController);
         estimatorFactory.setExternalPelvisCorrectorSubscriber(externalPelvisPoseSubscriber);
         drcStateEstimator = estimatorFactory.createStateEstimator(estimatorRegistry, yoGraphicsListRegistry);
         estimatorController.addRobotController(drcStateEstimator);
      }
      else
      {
         drcStateEstimator = null;
      }

      RobotJointLimitWatcher robotJointLimitWatcher = new RobotJointLimitWatcher(estimatorFullRobotModel.getOneDoFJoints(), sensorRawOutputMapReadOnly);
      estimatorController.addRobotController(robotJointLimitWatcher);

      if (realtimeRos2Node != null)
      {
         ForceSensorDataHolderReadOnly forceSensorDataHolderToSend;
         if (sensorReaderFactory.useStateEstimator() && forceSensorStateUpdater.getForceSensorOutputWithGravityCancelled() != null)
            forceSensorDataHolderToSend = forceSensorStateUpdater.getForceSensorOutputWithGravityCancelled();
         else
            forceSensorDataHolderToSend = forceSensorDataHolder;

         JointConfigurationGatherer jointConfigurationGathererAndProducer = new JointConfigurationGatherer(estimatorFullRobotModel,
                                                                                                           forceSensorDataHolderToSend);

         poseCommunicator = new DRCPoseCommunicator(estimatorFullRobotModel, jointConfigurationGathererAndProducer,
                                                    ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName), realtimeRos2Node,
                                                    sensorOutputMapReadOnly, sensorRawOutputMapReadOnly, robotMotionStatusFromController, sensorInformation);
         estimatorController.setRawOutputWriter(poseCommunicator);
      }
      else
      {
         poseCommunicator = null;
      }

      firstTick.set(true);

      estimatorRegistry.addChild(estimatorController.getYoVariableRegistry());

      if (outputWriter != null)
      {
         outputWriter.setJointDesiredOutputList(desiredJointDataHolder);
         if (outputWriter.getYoVariableRegistry() != null)
         {
            estimatorRegistry.addChild(outputWriter.getYoVariableRegistry());
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
         ekfStateEstimator = new HumanoidRobotEKFWithSimpleJoints(estimatorFullRobotModel, primaryImuName, imuSensorNames, footForceSensorNames,
                                                                  sensorRawOutputMapReadOnly, estimatorDT, gravity, sensorOutputMapReadOnly,
                                                                  yoGraphicsListRegistry, estimatorFullRobotModel);

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
         ekfStateEstimator = new HumanoidRobotEKFWithSimpleJoints(ekfFullRobotModel, primaryImuName, imuSensorNames, footForceSensorNames,
                                                                  sensorRawOutputMapReadOnly, estimatorDT, gravity, sensorOutputMapReadOnly,
                                                                  yoGraphicsListRegistry, estimatorFullRobotModel);

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
   }

   public void setupHighLevelControllerCallback(String robotName, RealtimeRos2Node realtimeRos2Node,
                                                Map<HighLevelControllerName, StateEstimatorMode> stateModeMap)
   {
      MessageTopicNameGenerator publisherTopicNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotName);
      ROS2Tools.createCallbackSubscription(realtimeRos2Node, HighLevelStateChangeStatusMessage.class, publisherTopicNameGenerator, subscriber -> {
         HighLevelStateChangeStatusMessage message = subscriber.takeNextData();
         StateEstimatorMode requestedMode = stateModeMap.get(HighLevelControllerName.fromByte(message.getEndHighLevelControllerName()));

         if (drcStateEstimator != null)
            drcStateEstimator.requestStateEstimatorMode(requestedMode);
         if (ekfStateEstimator != null)
            ekfStateEstimator.requestStateEstimatorMode(requestedMode);
      });
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return estimatorRegistry;
   }

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

         if (reinitializeEKF != null && reinitializeEKF.getValue())
         {
            reinitializeEKF.set(false);
            estimatorFullRobotModel.getRootJoint().getJointConfiguration(rootToWorldTransform);
            ekfStateEstimator.initializeEstimator(rootToWorldTransform);
         }

         sensorReader.compute(humanoidRobotContextData.getTimestamp(), humanoidRobotContextData.getSensorDataContext());
         estimatorController.doControl();
         if (forceSensorStateUpdater != null)
         {
            forceSensorStateUpdater.updateForceSensorState();
         }

         HumanoidRobotContextTools.updateContext(estimatorFullRobotModel, humanoidRobotContextData.getProcessedJointData());
         humanoidRobotContextData.setEstimatorRan(!firstTick.getValue());
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

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public ForceSensorCalibrationModule getForceSensorCalibrationModule()
   {
      return forceSensorStateUpdater;
   }

   public void initializeEstimator(RigidBodyTransform rootJointTransform, TObjectDoubleMap<String> jointPositions)
   {
      sensorReader.getSensorOutputMapReadOnly().reset();

      if (ekfStateEstimator != null)
         ekfStateEstimator.initializeEstimator(rootJointTransform, jointPositions);
      if (drcStateEstimator != null)
         drcStateEstimator.initializeEstimator(rootJointTransform, jointPositions);

      firstTick.set(true);
      humanoidRobotContextData.setControllerRan(false);
      humanoidRobotContextData.setEstimatorRan(false);
   }

   /**
    * used primarily for unit tests, but could be useful.
    */
   public void addRobotController(RobotController controller)
   {
      estimatorController.addRobotController(controller);
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return estimatorFullRobotModel;
   }

   public HumanoidRobotContextData getHumanoidRobotContextData()
   {
      return humanoidRobotContextData;
   }

   @Deprecated // TODO: Split up the sensor reader and move the part needed outside out of this class!
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }
}
