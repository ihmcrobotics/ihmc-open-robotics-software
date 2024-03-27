package us.ihmc.avatar.networkProcessor.directionalControlToolboxModule;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlInputCommand;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

public class DirectionalControlModule extends ToolboxModule
{
   private ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;
   private ROS2PublisherBasics<FootstepDataListMessage> footstepPublisher;
   private ROS2PublisherBasics<FootstepDataListMessage> footstepVisualizationPublisher;
   private final DirectionalControlController steppingController;

   /*
    * Determines how often the controller's update function will be called. Note that incoming messages
    * will be received and processed immediately when they arrive (this is the purpose of the
    * InputListener defined in the constructor). However, this level of processing is limited to simple
    * updates. The heavier work of reacting to the input happens in the controller's updateInternal()
    * function. Unlike the InputListener, the call to upateInternal is performed on a separate thread,
    * and so will not block other processing.
    */
   private final static int UPDATE_PERIOD_IN_MS = 10;

   public DirectionalControlModule(DRCRobotModel robotModel, boolean startYoVariableServer, RealtimeROS2Node ros2Node)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, UPDATE_PERIOD_IN_MS,
            ros2Node);

      steppingController = new DirectionalControlController(fullRobotModel, robotModel.getWalkingControllerParameters(), statusOutputManager, registry);
      setup(robotModel);
   }

   public DirectionalControlModule(DRCRobotModel robotModel, boolean startYoVariableServer, PubSubImplementation pubSubImplementation)
   {
      super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer, UPDATE_PERIOD_IN_MS,
            pubSubImplementation);

      steppingController = new DirectionalControlController(fullRobotModel, robotModel.getWalkingControllerParameters(), statusOutputManager, registry);
      setup(robotModel);
   }

   private void setup(DRCRobotModel robotModel)
   {
      steppingController.setPauseWalkingPublisher(pauseWalkingPublisher::publish);
      steppingController.setFootstepPublisher(footstepPublisher::publish);
      steppingController.setFootstepVisualizationPublisher(footstepVisualizationPublisher::publish);

      /*
       * Register a listener to process incoming commands to the toolbox. This listener is tailored to the
       * command we want to process in this module (so not, for example, for generic toolbox messages).
       * Actions taken by the handler should be as short as possible, since the handler blocks processing
       * of incoming commands by the CommandInputManager.
       */
      commandInputManager.registerHasReceivedInputListener(commandClass ->
      {
         DirectionalControlConfigurationCommand configCommand = commandInputManager.pollNewestCommand(DirectionalControlConfigurationCommand.class);
         if (configCommand != null)
         {
            steppingController.updateConfiguration(configCommand);
         }

         DirectionalControlInputCommand inputCommand = commandInputManager.pollNewestCommand(DirectionalControlInputCommand.class);
         if (inputCommand != null)
         {
            steppingController.updateInputs(inputCommand);
         }
      });

      // Must start the Yo Variable Server since it is used to track message timeouts
      startYoVariableServer();
   }

   /**
    * Register extra pubs/subs that are needed by the controller but are not controller messages. For
    * DirectionalControl, we do not use this because of the desire to keep the controller usable in a
    * standalone mode.
    */
   @Override
   public void registerExtraPuSubs(ROS2NodeInterface ros2Node)
   {
      ROS2Topic<?> controllerPubGenerator = ControllerAPIDefinition.getOutputTopic(robotName);

      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerPubGenerator, s ->
      {
         if (steppingController != null)
            steppingController.updateRobotConfigurationData(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, FootstepStatusMessage.class, controllerPubGenerator, s ->
      {
         if (steppingController != null)
            steppingController.updateFootstepStatusMessage(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic, s ->
      {
         if (steppingController != null)
            steppingController.updatePlanarRegionsListMessage(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, WalkingControllerFailureStatusMessage.class, controllerPubGenerator, s ->
      {
         if (steppingController != null)
            steppingController.updateWalkingControllerFailureStatusMessage(s.takeNextData());
      });
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, CapturabilityBasedStatus.class, controllerPubGenerator, s ->
      {
         if (steppingController != null)
            steppingController.updateCapturabilityBasedStatus(s.takeNextData());
      });

      ROS2Topic<?> controllerSubGenerator = ControllerAPIDefinition.getInputTopic(robotName);

      pauseWalkingPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, controllerSubGenerator);
      footstepPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, FootstepDataListMessage.class, controllerSubGenerator);
      footstepVisualizationPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node,
                                                                          FootstepDataListMessage.class,
                                                                          DirectionalControlModule.getOutputTopic(robotName));

   }

   @Override
   public ToolboxController getToolboxController()
   {
      return steppingController;
   }

   @Override
   public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands()
   {
      return supportedCommands();
   }

   /**
    * Here we declare the toolbox-specific commands. Note that these are commands, not messages. The
    * CommandInputManager receives messages on the toolbox topics, and converts them to commands for
    * processing.
    * 
    * @return
    */
   public static List<Class<? extends Command<?, ?>>> supportedCommands()
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
      commands.add(DirectionalControlConfigurationCommand.class);
      commands.add(DirectionalControlInputCommand.class);
      return commands;
   }

   @Override
   public List<Class<? extends Settable<?>>> createListOfSupportedStatus()
   {
      return supportedStatus();
   }

   /**
    * Here we declare the toolbox-specific status messages.
    * 
    * @return
    */
   public static List<Class<? extends Settable<?>>> supportedStatus()
   {
      List<Class<? extends Settable<?>>> status = new ArrayList<>();
      return status;
   }

   @Override
   public void sleep()
   {
      LogTools.info("Directional control toolbox told to sleep");
      super.sleep();
   }

   @Override
   public void wakeUp()
   {
      LogTools.info("Directional control toolbox told to wake up");
      super.wakeUp();
   }

   @Override
   public ROS2Topic<?> getOutputTopic()
   {
      return getOutputTopic(robotName);
   }

   public static ROS2Topic<?> getOutputTopic(String robotName)
   {
      return ROS2Tools.DIRECTIONAL_CONTROL_TOOLBOX.withRobot(robotName).withOutput();
   }

   @Override
   public ROS2Topic<?> getInputTopic()
   {
      return getInputTopic(robotName);
   }

   public static ROS2Topic<?> getInputTopic(String robotName)
   {
      return ROS2Tools.DIRECTIONAL_CONTROL_TOOLBOX.withRobot(robotName).withInput();
   }
}
