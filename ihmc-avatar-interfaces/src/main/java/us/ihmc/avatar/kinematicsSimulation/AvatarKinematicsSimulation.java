package us.ihmc.avatar.kinematicsSimulation;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.YoVariableServer;
import us.ihmc.robotDataLogger.logger.DataServerSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.functional.FunctionalTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Arrays;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class AvatarKinematicsSimulation
{
   private static final double DT = UnitConversions.hertzToSeconds(70);
   public static final double PLAYBACK_SPEED = 10.0;
   private final DRCRobotModel robotModel;
   private final AvatarKinematicsSimulationController avatarKinematicsSimulationController;
   private final ExceptionHandlingThreadScheduler scheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                   DefaultExceptionHandler.PRINT_MESSAGE,
                                                                                                   5);
   private final ExceptionHandlingThreadScheduler yoVariableScheduler = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                             DefaultExceptionHandler.PRINT_MESSAGE,
                                                                                                             5);
   private final Ros2Node ros2Node;
   private final IHMCROS2Publisher<RobotConfigurationData> robotConfigurationDataPublisher;
   private final IHMCROS2Publisher<CapturabilityBasedStatus> capturabilityBasedStatusPublisher;
   private final IHMCROS2Publisher<WalkingStatusMessage> walkingStatusPublisher;
   private final IHMCROS2Publisher<FootstepStatusMessage> footstepStatusPublisher;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
   private double yoVariableServerTime = 0.0;
   private YoVariableServer yoVariableServer;
   private ScheduledFuture<?> yoVariableServerScheduled;

   public static void createForManualTest(DRCRobotModel robotModel, boolean createYoVariableServer)
   {
      create(robotModel, createYoVariableServer, PubSubImplementation.FAST_RTPS);
   }

   public static void createForAutomatedTest(DRCRobotModel robotModel, boolean createYoVariableServer)
   {
      create(robotModel, createYoVariableServer, PubSubImplementation.INTRAPROCESS);
   }

   private static void create(DRCRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      new AvatarKinematicsSimulation(robotModel, createYoVariableServer, pubSubImplementation);
   }

   public AvatarKinematicsSimulation(DRCRobotModel robotModel, boolean createYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      this.robotModel = robotModel;

      // instantiate some existing controller ROS2 API?
      ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, ROS2Tools.HUMANOID_CONTROLLER.getNodeName("kinematic"));

      robotConfigurationDataPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                                RobotConfigurationData.class,
                                                                robotModel.getSimpleRobotName(),
                                                                ROS2Tools.HUMANOID_CONTROLLER);
      capturabilityBasedStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                                  CapturabilityBasedStatus.class,
                                                                  robotModel.getSimpleRobotName(),
                                                                  ROS2Tools.HUMANOID_CONTROLLER);
      walkingStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                       WalkingStatusMessage.class,
                                                       robotModel.getSimpleRobotName(),
                                                       ROS2Tools.HUMANOID_CONTROLLER);
      footstepStatusPublisher = new IHMCROS2Publisher<>(ros2Node,
                                                        FootstepStatusMessage.class,
                                                        robotModel.getSimpleRobotName(),
                                                        ROS2Tools.HUMANOID_CONTROLLER);

      new ROS2Callback<>(ros2Node,
                         FootstepDataListMessage.class,
                         robotModel.getSimpleRobotName(),
                         ROS2Tools.HUMANOID_CONTROLLER,
                         this::acceptFootstepDataListMessage);

      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(0.0, 0.0);

      avatarKinematicsSimulationController = new AvatarKinematicsSimulationController(robotModel,
                                                                                      robotInitialSetup.getInitialPelvisPose(),
                                                                                      robotInitialSetup.getInitialJointAngles(),
                                                                                      DT,
                                                                                      yoGraphicsListRegistry,
                                                                                      registry,
                                                                                      walkingStatusPublisher,
                                                                                      footstepStatusPublisher);

      FunctionalTools.runIfTrue(createYoVariableServer, this::createYoVariableServer);

      avatarKinematicsSimulationController.initialize();

      scheduler.schedule(this::controllerTick, Conversions.secondsToNanoseconds(DT / PLAYBACK_SPEED), TimeUnit.NANOSECONDS);
   }

   private void controllerTick()
   {
      avatarKinematicsSimulationController.doControl();

      robotConfigurationDataPublisher.publish(extractRobotConfigurationData(avatarKinematicsSimulationController.getFullRobotModel()));

      capturabilityBasedStatusPublisher.publish(avatarKinematicsSimulationController.getCapturabilityBasedStatus());
   }

   private void acceptFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      avatarKinematicsSimulationController.getInputManager().submitMessage(footstepDataListMessage);
   }

   private void createYoVariableServer()
   {
      yoVariableServer = new YoVariableServer(getClass(), robotModel.getLogModelProvider(), new DataServerSettings(false), 0.01);
      yoVariableServer.setMainRegistry(registry, robotModel.createFullRobotModel().getElevator(), yoGraphicsListRegistry);
      ThreadTools.startAThread(() -> yoVariableServer.start(), getClass().getSimpleName() + "YoVariableServer");

      yoVariableServerScheduled = yoVariableScheduler.schedule(this::yoVariableUpdateThread, 1, TimeUnit.MILLISECONDS);
   }

   private void yoVariableUpdateThread()
   {
      if (!Thread.interrupted())
      {
         yoVariableServerTime += Conversions.millisecondsToSeconds(1);
         yoVariableServer.update(Conversions.secondsToNanoseconds(yoVariableServerTime));
      }
   }

   public static RobotConfigurationData extractRobotConfigurationData(FullHumanoidRobotModel fullRobotModel)
   {
      fullRobotModel.updateFrames();
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModel);
      RobotConfigurationData robotConfigurationData = RobotConfigurationDataFactory.create(joints,
                                                                                           fullRobotModel.getForceSensorDefinitions(),
                                                                                           fullRobotModel.getIMUDefinitions());
      RobotConfigurationDataFactory.packJointState(robotConfigurationData, Arrays.stream(joints).collect(Collectors.toList()));
      robotConfigurationData.getRootTranslation().set(fullRobotModel.getRootJoint().getJointPose().getPosition());
      robotConfigurationData.getRootOrientation().set(fullRobotModel.getRootJoint().getJointPose().getOrientation());
      return robotConfigurationData;
   }
}
