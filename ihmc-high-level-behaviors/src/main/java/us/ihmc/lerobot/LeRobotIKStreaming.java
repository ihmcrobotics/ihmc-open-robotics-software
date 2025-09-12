package us.ihmc.lerobot;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

/**
 * Handles streaming the output of visuomotor policy inference to the IK streaming toolbox.
 */
public class LeRobotIKStreaming
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> inputPublisher;
   private final ROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private boolean controlRobot = false;

   public LeRobotIKStreaming(ROS2Node ros2Node, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;

      String robotName = robotModel.getSimpleRobotName();
      inputPublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingInputTopic(robotName));
      toolboxStatePublisher = ros2Node.createPublisher(ToolboxAPIs.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName)
                                                                                               .withInput()
                                                                                               .withTypeName(ToolboxStateMessage.class));
   }

   public void wakeUp()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      toolboxStatePublisher.publish(toolboxStateMessage);
   }

   public void update(long actionTimestampNanos, SideDependentList<Pose3D> handPoses, SideDependentList<Pose3D> forearmPoses)
   {
      KinematicsStreamingToolboxInputMessage ikInputMessage = new KinematicsStreamingToolboxInputMessage();
      ikInputMessage.setStreamToController(controlRobot);
      ikInputMessage.setTimestamp(actionTimestampNanos);
      for (RobotSide side : RobotSide.values)
      {
         KinematicsToolboxRigidBodyMessage rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
         rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getHand(side).hashCode());
         rigidBodyMessage.getDesiredPositionInWorld().set(handPoses.get(side).getTranslation());
         rigidBodyMessage.getDesiredOrientationInWorld().set(handPoses.get(side).getRotation());
         rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.02);
         rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.02);
         rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.02);
         ikInputMessage.getInputs().add().set(rigidBodyMessage);

         rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
         rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getForearm(side).hashCode());
         rigidBodyMessage.getDesiredPositionInWorld().set(forearmPoses.get(side).getTranslation());
         rigidBodyMessage.getLinearSelectionMatrix().setXSelected(false); // Disable position tracking for forearm
         rigidBodyMessage.getLinearSelectionMatrix().setYSelected(false);
         rigidBodyMessage.getLinearSelectionMatrix().setZSelected(false);
         rigidBodyMessage.getDesiredOrientationInWorld().set(forearmPoses.get(side).getRotation());
         rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.01);
         rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.01);
         rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.001);
         ikInputMessage.getInputs().add().set(rigidBodyMessage);
      }
      inputPublisher.publish(ikInputMessage);
   }

   public void setControlRobot(boolean controlRobot)
   {
      this.controlRobot = controlRobot;
   }
}
