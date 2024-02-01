package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.StepConstraintMessage;
import perception_msgs.msg.dds.PlanarRegionMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class StepConstraintToolboxModule extends ToolboxModule
{
   private static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 10;

   protected final StepConstraintToolboxController controller;
   private IHMCROS2Publisher<StepConstraintMessage> constraintRegionPublisher;

   public StepConstraintToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation, double gravityZ)
   {
      super(robotModel.getSimpleRobotName(),
            robotModel.createFullRobotModel(),
            robotModel.getLogModelProvider(),
            startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS,
            pubSubImplementation);

      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);
      controller = new StepConstraintToolboxController(statusOutputManager,
                                                       constraintRegionPublisher,
                                                       robotModel.getWalkingControllerParameters(),
                                                       fullRobotModel,
                                                       gravityZ,
                                                       registry,
                                                       yoGraphicsListRegistry);

      startYoVariableServer();
      if (yoVariableServer != null)
      {
         JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer);
         jvmStatisticsGenerator.start();
      }
   }

   @Override
   public void registerExtraPuSubs(ROS2NodeInterface ros2Node)
   {
      ROS2Topic controllerPubGenerator = ControllerAPIDefinition.getOutputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            controller.updateRobotConfigurationData(s.takeNextData());
         }
      });

      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            s.takeNextData(robotConfigurationData, null);
            controller.updateRobotConfigurationData(robotConfigurationData);
         }
      });

      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            s.takeNextData(capturabilityBasedStatus, null);
            controller.updateCapturabilityBasedStatus(capturabilityBasedStatus);
         }
      });

      FootstepStatusMessage statusMessage = new FootstepStatusMessage();

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepStatusMessage.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            s.takeNextData(statusMessage, null);
            controller.updateFootstepStatus(statusMessage);
         }
      });

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    PlanarRegionsListMessage.class,
                                                    REACommunicationProperties.outputTopic,
                                                    s -> updatePlanarRegion(s.takeNextData()));

      constraintRegionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, StepConstraintMessage.class, ControllerAPIDefinition.getInputTopic(robotName));
   }

   public void setSwitchPlanarRegionConstraintsAutomatically(boolean switchAutomatically)
   {
      controller.setSwitchPlanarRegionConstraintsAutomatically(switchAutomatically);
   }

   public void updatePlanarRegion(PlanarRegionsListMessage planarRegionsListMessage)
   {
      if (controller != null)
      {
         controller.updatePlanarRegions(planarRegionsListMessage);
      }
   }

   @Override
   public ToolboxController getToolboxController()
   {
      return controller;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return new ArrayList<>();
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      status.add(PlanarRegionsListMessage.class);
      status.add(PlanarRegionMessage.class);
      return status;
   }

   @Override
   public ROS2Topic getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic getOutputTopic(String robotName)
   {
      return ROS2Tools.STEP_CONSTRAINT_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic getInputTopic(String robotName)
   {
      return ROS2Tools.STEP_CONSTRAINT_TOOLBOX.withRobot(robotName).withInput();
   }
}
