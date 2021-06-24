package us.ihmc.avatar.networkProcessor.pushRecoveryToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule.StepConstraintToolboxController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;

import java.util.ArrayList;
import java.util.List;

public class PushRecoveryToolboxModule extends ToolboxModule
{
   private static final int DEFAULT_UPDATE_PERIOD_MILLISECONDS = 10;

   protected final PushRecoveryToolboxController controller;
   private IHMCRealtimeROS2Publisher<PushRecoveryResultMessage> constraintRegionPublisher;

   public PushRecoveryToolboxModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation, double gravityZ)
   {
      super(robotModel.getSimpleRobotName(),
            robotModel.createFullRobotModel(),
            robotModel.getLogModelProvider(),
            startYoVariableServer,
            DEFAULT_UPDATE_PERIOD_MILLISECONDS,
            pubSubImplementation);

      setTimeWithoutInputsBeforeGoingToSleep(Double.POSITIVE_INFINITY);

      ConvexPolygon2D defaultFootPolygon = new ConvexPolygon2D();
      defaultFootPolygon.addVertices(Vertex2DSupplier.asVertex2DSupplier(robotModel.getContactPointParameters().getFootContactPoints().get(RobotSide.RIGHT)));
      defaultFootPolygon.update();

      controller = new PushRecoveryToolboxController(statusOutputManager,
                                                     constraintRegionPublisher,
                                                     fullRobotModel,
                                                     defaultFootPolygon,
                                                     robotModel.getPushRecoveryControllerParameters(),
                                                     gravityZ,
                                                     registry,
                                                     yoGraphicsListRegistry);

      new DefaultParameterReader().readParametersInRegistry(getRegistry());
      startYoVariableServer();
      if (yoVariableServer != null)
      {
         JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(yoVariableServer);
         jvmStatisticsGenerator.start();
      }
   }

   public void attachListener(Runnable listener)
   {
      controller.attachListener(listener);
   }

   @Override
   public void registerExtraPuSubs(RealtimeROS2Node realtimeROS2Node)
   {
      ROS2Topic<?> controllerPubGenerator = ControllerAPIDefinition.getOutputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            controller.updateRobotConfigurationData(s.takeNextData());
         }
      });

      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            s.takeNextData(robotConfigurationData, null);
            controller.updateRobotConfigurationData(robotConfigurationData);
         }
      });

      CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            s.takeNextData(capturabilityBasedStatus, null);
            controller.updateCapturabilityBasedStatus(capturabilityBasedStatus);
         }
      });


      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node,
                                                    PlanarRegionsListMessage.class,
                                                    REACommunicationProperties.outputTopic,
                                                    s -> updatePlanarRegion(s.takeNextData()));

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, HighLevelStateMessage.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            controller.updateHighLevelState(s.takeNextData());
         }
      });

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, HighLevelStateChangeStatusMessage.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            controller.updateHighLevelStateChange(s.takeNextData());
         }
      });

      ROS2Tools.createCallbackSubscriptionTypeNamed(realtimeROS2Node, FootstepStatusMessage.class, controllerPubGenerator, s ->
      {
         if (controller != null)
         {
            controller.updateFootstepStatus(s.takeNextData());
         }
      });


      constraintRegionPublisher = ROS2Tools.createPublisherTypeNamed(realtimeROS2Node,
                                                                     PushRecoveryResultMessage.class,
                                                                     ControllerAPIDefinition.getInputTopic(robotName));
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
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ROS2Tools.STEP_CONSTRAINT_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ROS2Tools.STEP_CONSTRAINT_TOOLBOX.withRobot(robotName).withInput();
   }
}
