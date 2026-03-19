package us.ihmc.avatar.joystickBasedLocomotion;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import controller_msgs.msg.dds.VelocityBasedWalkingInputMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.CSGROS2CommunicationHelper;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.tools.Timer;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.avatar.AvatarControllerThread;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.ContinuousStepGeneratorParametersCommand;
import us.ihmc.avatar.AvatarStepGeneratorThread;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorNetworkSubscriber;

/**
 * Abstract plugin for using any handheld RC controller to send velocity-based walking commands using:
 * <p>
 * 1) {@link VelocityBasedWalkingInputMessage} which communicates with controllers in the {@link AvatarControllerThread} via
 * the {@link ControllerAPIDefinition} and {@link ControllerNetworkSubscriber}
 *
 * 2) {@link ContinuousStepGeneratorInputMessage} which communicates with the {@link ContinuousStepGenerator} in the
 * {@link AvatarStepGeneratorThread} via the {@link StepGeneratorAPIDefinition} and {@link StepGeneratorNetworkSubscriber}.
 * Communication with the {@link AvatarStepGeneratorThread} is facilitated by the {@link CSGROS2CommunicationHelper}
 *
 * This plugin can take desired forward, lateral, and turning velocity setpoints from any arbitrary RC handheld, pack
 * them in the appropriate message types, and send those messages to the correct places to enact a walking movement.
 * Additionally, this abstract plugin can send desired parameters to the {@link ContinuousStepGenerator} using
 * {@link ContinuousStepGeneratorParametersCommand}. Thanks to the {@link CSGROS2CommunicationHelper}, large discrete
 * jumps in CSG parameters are prevented, since the {@link ContinuousStepGeneratorParametersCommand} is regularly reset
 * to current parameter values which are provided via a subscription to {@link ContinuousStepGeneratorStatusMessage}
 *
 * @author Stefan Fasano
 */
public abstract class AbstractJoystickLocomotionPlugin
{
   protected static final double PUBLISHER_HEARTBEAT_DURATION = 1.0;

   // CSG thread ROS communication helper
   protected final CSGROS2CommunicationHelper csgROS2CommunicationHelper;

   // Publisher for DirectionalControlInputMessage (going into the controller thread)
   protected final ROS2Publisher<VelocityBasedWalkingInputMessage> directionalControlInputMessagePublisher;

   // Stuff to limit how often we publish
   protected final Throttler publisherThrottler = new Throttler();
   protected final Timer heartBeat = new Timer();

   // CSG thread command and status messages
   protected final ContinuousStepGeneratorInputMessage csgInputCommand;
   protected final ContinuousStepGeneratorParametersMessage csgParametersCommand;
   protected final ContinuousStepGeneratorStatusMessage csgStatusMessage;

   // Controller thread walking message
   protected final VelocityBasedWalkingInputMessage directionalControlInputMessage;

   public AbstractJoystickLocomotionPlugin(DRCRobotModel robotModel, ROS2Node ros2Node)
   {
      csgROS2CommunicationHelper = new CSGROS2CommunicationHelper(robotModel.getSimpleRobotName(), ros2Node, robotModel.getWalkingControllerParameters());
      directionalControlInputMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(VelocityBasedWalkingInputMessage.class, robotModel.getSimpleRobotName()));

      csgInputCommand = csgROS2CommunicationHelper.getCSGInputCommand();
      csgParametersCommand = csgROS2CommunicationHelper.getCSGParametersCommand();
      csgStatusMessage = csgROS2CommunicationHelper.getCSGStatusMessage();

      directionalControlInputMessage = new VelocityBasedWalkingInputMessage();

      publisherThrottler.setPeriod(robotModel.getStepGeneratorDT() * 3.0); // Publish at 1/3 the rate of CSG thread
      heartBeat.reset();
   }

   public abstract void update();

   protected void publish()
   {
      csgROS2CommunicationHelper.publish(csgInputCommand);
      directionalControlInputMessagePublisher.publish(directionalControlInputMessage);
   }

   public void sendStopWalkingCommands()
   {
      // Setting walking to false and desired velocities to 0
      setDesiredWalkingCommands(false, 0.0, 0.0, 0.0, true);

      // Publish messages
      publish();
   }

   protected void setDesiredWalkingCommands(boolean requestWalking, double forwardVelocity, double lateralVelocity, double turnVelocity, boolean areVelocitiesNormalized)
   {
      // Populate CSG input command
      csgInputCommand.setWalk(requestWalking);
      csgInputCommand.setAreVelocitiesNormalized(areVelocitiesNormalized);
      csgInputCommand.setForwardVelocity(forwardVelocity);
      csgInputCommand.setLateralVelocity(lateralVelocity);
      csgInputCommand.setTurnVelocity(turnVelocity);

      // Populate directional control input command
      directionalControlInputMessage.setWalk(requestWalking);
      directionalControlInputMessage.setAreVelocitiesNormalized(areVelocitiesNormalized);
      directionalControlInputMessage.setForwardVelocity(forwardVelocity);
      directionalControlInputMessage.setLateralVelocity(lateralVelocity);
      directionalControlInputMessage.setTurnVelocity(turnVelocity);

      if (requestWalking)
         heartBeat.reset();
   }

   public void shutdown()
   {
      sendStopWalkingCommands();

      csgROS2CommunicationHelper.destroy();
   }
}
