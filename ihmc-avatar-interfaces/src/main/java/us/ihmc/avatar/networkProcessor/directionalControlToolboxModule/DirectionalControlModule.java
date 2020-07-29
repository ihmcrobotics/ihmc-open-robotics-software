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
	private final static int update_period_in_ms = 1000;

	public DirectionalControlModule(DRCRobotModel robotModel, boolean startYoVariableServer) {
		super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(),
				startYoVariableServer, update_period_in_ms);

		steppingController = new DirectionalControlController(fullRobotModel, robotModel, statusOutputManager, registry,
				getSubscriberTopicNameGenerator(), getPublisherTopicNameGenerator());

		commandInputManager.registerHasReceivedInputListener(new HasReceivedInputListener() {

			@Override
			public void hasReceivedInput(Class<? extends Command<?, ?>> commandClass) {
				DirectionalControlConfigurationCommand configCommand = commandInputManager
						.pollNewestCommand(DirectionalControlConfigurationCommand.class);
				if (configCommand != null) {
					LogTools.info("Got new config command");
					steppingController.updateConfiguration(configCommand.getMessage());
				}

				DirectionalControlInputCommand inputCommand = commandInputManager
						.pollNewestCommand(DirectionalControlInputCommand.class);
				if (inputCommand != null) {
					LogTools.info("Got new input command");
					steppingController.updateInputs(inputCommand.getMessage());
				}
			}

		});

		// Must start the Yo Variable Server since it is used to track message timeouts
		startYoVariableServer();
	}

	@Override
	public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node) {
		MessageTopicNameGenerator controllerPubGenerator = ControllerAPIDefinition
				.getPublisherTopicNameGenerator(robotName);

		ROS2Tools.createCallbackSubscription(realtimeRos2Node, RobotConfigurationData.class, controllerPubGenerator,
				s -> {
					if (steppingController != null)
						steppingController.updateRobotConfigurationData(s.takeNextData());
				});
	}

	@Override
	public ToolboxController getToolboxController() {
		return steppingController;
	}

	@Override
	public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands() {
		return supportedCommands();
	}

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
		LogTools.info("Directional toolbox told to sleep");
		super.sleep();
	}

	@Override
	public void wakeUp() {
		LogTools.info("Directional toolbox told to wake up");
		super.wakeUp();
	}

	public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName) {
		return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.DIRECTIONAL_CONTROL_TOOLBOX, ROS2TopicQualifier.INPUT);
	}

	public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName) {
		return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.DIRECTIONAL_CONTROL_TOOLBOX, ROS2TopicQualifier.OUTPUT);
	}
}
