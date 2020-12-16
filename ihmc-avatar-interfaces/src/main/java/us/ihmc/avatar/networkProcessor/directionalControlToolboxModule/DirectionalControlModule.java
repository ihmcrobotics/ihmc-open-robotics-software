package us.ihmc.avatar.networkProcessor.directionalControlToolboxModule;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.CommandInputManager.HasReceivedInputListener;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.RealtimeRos2Node;

public class DirectionalControlModule extends ToolboxModule {

	private final DirectionalControlController steppingController;
	
	/* Determines how often the controller's update function will be called. Note that incoming messages will be 
	 * received and processed immediately when they arrive (this is the purpose of the InputListener defined 
	 * in the constructor). However, this level of processing is limited to simple updates. The heavier work of 
	 * reacting to the input happens in the controller's updateInternal() function. Unlike the InputListener, the 
	 * call to upateInternal is performed on a separate thread, and so will not block other processing.
	 */
	private final static int UPDATE_PERIOD_IN_MS = 1;

	public DirectionalControlModule(DRCRobotModel robotModel, boolean startYoVariableServer) {
		super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(),
				startYoVariableServer, UPDATE_PERIOD_IN_MS);

		steppingController = new DirectionalControlController(fullRobotModel, robotModel, statusOutputManager, registry);

		/*
		 * Register a listener to process incoming commands to the toolbox. This listener is tailored
		 * to the command we want to process in this module (so not, for example, for generic toolbox messages).
		 * 
		 * Actions taken by the handler should be as short as possible, since the handler blocks processing
		 * of incoming commands by the CommandInputManager.
		 */
		commandInputManager.registerHasReceivedInputListener(new HasReceivedInputListener() {

			@Override
			public void hasReceivedInput(Class<? extends Command<?, ?>> commandClass) {
				
				DirectionalControlConfigurationCommand configCommand = commandInputManager
						.pollNewestCommand(DirectionalControlConfigurationCommand.class);
				if (configCommand != null) {
					steppingController.updateConfiguration(configCommand);
				}

				DirectionalControlInputCommand inputCommand = commandInputManager
						.pollNewestCommand(DirectionalControlInputCommand.class);
				if (inputCommand != null) {
					steppingController.updateInputs(inputCommand);
				}
			}

		});

		// Must start the Yo Variable Server since it is used to track message timeouts
		startYoVariableServer();
	}

	/**
	 * Register extra pubs/subs that are needed by the controller but are not controller messages.
	 * For DirectionalControl, we do not use this because of the desire to keep the controller
	 * usable in a standalone mode.
	 */
	@Override
	public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node) {
	}

	@Override
	public ToolboxController getToolboxController() {
		return steppingController;
	}

	@Override
	public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands() {
		return supportedCommands();
	}

	/**
	 * Here we declare the toolbox-specific commands. Note that these are commands, not messages.
	 * The CommandInputManager receives messages on the toolbox topics, and converts them to commands
	 * for processing. 
	 * @return
	 */
	public static List<Class<? extends Command<?, ?>>> supportedCommands() {
		List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();
		commands.add(DirectionalControlConfigurationCommand.class);
		commands.add(DirectionalControlInputCommand.class);
		return commands;
	}

	@Override
	public List<Class<? extends Settable<?>>> createListOfSupportedStatus() {
		return supportedStatus();
	}

	/**
	 * Here we declare the toolbox-specific status messages.
	 * @return
	 */
	public static List<Class<? extends Settable<?>>> supportedStatus() {
		List<Class<? extends Settable<?>>> status = new ArrayList<>();
		return status;
	}

	@Override
	public MessageTopicNameGenerator getPublisherTopicNameGenerator() {
		return getPublisherTopicNameGenerator(robotName);
	}

	@Override
	public MessageTopicNameGenerator getSubscriberTopicNameGenerator() {
		return getSubscriberTopicNameGenerator(robotName);
	}

	@Override
	public void sleep() {
		LogTools.info("Directional control toolbox told to sleep");
		super.sleep();
	}

	@Override
	public void wakeUp() {
		LogTools.info("Directional control toolbox told to wake up");
		super.wakeUp();
	}

	public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName) {
		return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.DIRECTIONAL_CONTROL_TOOLBOX, ROS2TopicQualifier.INPUT);
	}

	public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName) {
		return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.DIRECTIONAL_CONTROL_TOOLBOX, ROS2TopicQualifier.OUTPUT);
	}
}
