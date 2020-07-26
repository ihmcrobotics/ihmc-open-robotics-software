package us.ihmc.avatar.networkProcessor.directionalControlToolboxModule;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.JoystickRemoteControlMessage;
import controller_msgs.msg.dds.JoystickRemoteInputMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.avatar.networkProcessor.modules.ToolboxModule;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxConfigurationCommand;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.RealtimeRos2Node;

public class DirectionalControlModule extends ToolboxModule {

	private final DirectionalControlController steppingController;

	public DirectionalControlModule(DRCRobotModel robotModel, boolean startYoVariableServer) {
		super(robotModel.getSimpleRobotName(), robotModel.createFullRobotModel(), robotModel.getLogModelProvider(), startYoVariableServer);

		steppingController = new DirectionalControlController(fullRobotModel, robotModel, statusOutputManager, registry);
	}

	@Override
	public void registerExtraPuSubs(RealtimeRos2Node realtimeRos2Node) {
	}

	@Override
	public ToolboxController getToolboxController() {
		return steppingController;
	}

	@Override
	public List<Class<? extends Command<?, ?>>> createListOfSupportedCommands() {
		List<Class<? extends Command<?, ?>>> list = new ArrayList<Class<? extends Command<?, ?>>>();
		return list;
	}

	static List<Class<? extends Command<?, ?>>> supportedCommands() {
		List<Class<? extends Command<?, ?>>> list = new ArrayList<Class<? extends Command<?, ?>>>();
		return list;
	}

	@Override
	public List<Class<? extends Settable<?>>> createListOfSupportedStatus() {
		List<Class<? extends Settable<?>>> list = new ArrayList<Class<? extends Settable<?>>>();
		return list;
	}

	@Override
	public MessageTopicNameGenerator getPublisherTopicNameGenerator() {
		return getPublisherTopicNameGenerator(robotName);
	}

	@Override
	public MessageTopicNameGenerator getSubscriberTopicNameGenerator() {
		return getSubscriberTopicNameGenerator(robotName);
	}

	public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName) {
		return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.INPUT);
	}

	public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName) {
		return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.KINEMATICS_PLANNING_TOOLBOX, ROS2TopicQualifier.OUTPUT);
	}
}
