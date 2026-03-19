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

public abstract class AbstractJoystickLocomotionPlugin
{
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

      publisherThrottler.setPeriod(robotModel.getStepGeneratorDT() * 2); // Publish at half the rate of CSG thread
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
      setDesiredWalkingCommands(false, 0.0, 0.0, 0.0);

      // Publish messages
      publish();
   }

   protected void setDesiredWalkingCommands(boolean requestWalking, double forwardVelocity, double lateralVelocity, double turnVelocity)
   {
      // Populate CSG input command
      csgInputCommand.setWalk(requestWalking);
      csgInputCommand.setAreVelocitiesNormalized(false);
      csgInputCommand.setForwardVelocity(forwardVelocity);
      csgInputCommand.setLateralVelocity(lateralVelocity);
      csgInputCommand.setTurnVelocity(turnVelocity);

      // Populate directional control input command
      directionalControlInputMessage.setWalk(requestWalking);
      directionalControlInputMessage.setAreVelocitiesNormalized(false);
      directionalControlInputMessage.setForwardVelocity(forwardVelocity);
      directionalControlInputMessage.setLateralVelocity(lateralVelocity);
      directionalControlInputMessage.setTurnVelocity(turnVelocity);

      if (requestWalking)
         heartBeat.reset();
   }

   protected void setDesiredWalkingParameters(double swingDuration, double transferDuration, double swingHeight)
   {
      csgParametersCommand.setSwingDuration(swingDuration);
      csgParametersCommand.setTransferDuration(transferDuration);
      csgParametersCommand.setSwingHeight(swingHeight);
   }

   public void shutdown()
   {
      sendStopWalkingCommands();

      csgROS2CommunicationHelper.destroy();
   }
}
