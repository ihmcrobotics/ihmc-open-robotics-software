package us.ihmc.lerobot;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInitialConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.controllerAPI.ControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

/**
 * Handles streaming the output of visuomotor policy inference to the IK streaming toolbox.
 */
public class LeRobotIKStreaming
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final Vector3D positionWeight = new Vector3D(-1.0, -1.0, -1.0); // default values are used
   private final Vector3D orientationWeight = new Vector3D(-1.0, -1.0, -1.0); // default values are used
   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> inputPublisher;
   private final ROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final ROS2Publisher<KinematicsStreamingToolboxInitialConfigurationMessage> initialConfigurationPublisher;
   private boolean controlRobot = false;
   private RigidBodyTransform initialChestPose;
   private RigidBodyTransform initialPelvisPose;

   public LeRobotIKStreaming(ROS2Node ros2Node, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;

      String robotName = robotModel.getSimpleRobotName();
      inputPublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingInputTopic(robotName));
      toolboxStatePublisher = ros2Node.createPublisher(ToolboxAPIs.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName)
                                                                                               .withInput()
                                                                                               .withTypeName(ToolboxStateMessage.class));
      initialConfigurationPublisher = ros2Node.createPublisher(ControllerAPI.getTopic(ToolboxAPIs.KINEMATICS_STREAMING_TOOLBOX.withRobot(robotName).withInput(),
                                                                                      KinematicsStreamingToolboxInitialConfigurationMessage.class));
   }

   public void saveInitialConfiguration()
   {
      synchronized (syncedRobot)
      {
         initialChestPose = syncedRobot.getReferenceFrames().getChestFrame().getTransformToWorldFrame();
         initialPelvisPose = syncedRobot.getReferenceFrames().getPelvisFrame().getTransformToWorldFrame();
      }
   }

   public void update(long actionTimestampNanos, SideDependentList<Pose3D> handPoses, boolean controlArmsOnly)
   {
      KinematicsStreamingToolboxInputMessage ikInputMessage = new KinematicsStreamingToolboxInputMessage();

      ikInputMessage.setStreamToController(controlRobot);

      for (RobotSide side : RobotSide.values)
         addRigidBodyInput(syncedRobot.getFullRobotModel().getHand(side).hashCode(), handPoses.get(side), ikInputMessage);

      if (controlArmsOnly)
      {
         lockChest(ikInputMessage);
         lockPelvis(ikInputMessage);
      }

      ikInputMessage.setTimestamp(actionTimestampNanos);

      inputPublisher.publish(ikInputMessage);
   }

   private void addRigidBodyInput(int endEffectorHashCode, RigidBodyTransformReadOnly pose, KinematicsStreamingToolboxInputMessage ikInputMessage)
   {
      KinematicsToolboxRigidBodyMessage rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
      rigidBodyMessage.setEndEffectorHashCode(endEffectorHashCode);

      rigidBodyMessage.getDesiredOrientationInWorld().set(pose.getRotation());
      rigidBodyMessage.getDesiredPositionInWorld().set(pose.getTranslation());

      WeightMatrix3D linearWeightMatrix = new WeightMatrix3D();
      rigidBodyMessage.getLinearSelectionMatrix().setXSelected(positionWeight.getX() != 0.0);
      rigidBodyMessage.getLinearSelectionMatrix().setYSelected(positionWeight.getY() != 0.0);
      linearWeightMatrix.setXAxisWeight(positionWeight.getX());
      linearWeightMatrix.setYAxisWeight(positionWeight.getY());
      rigidBodyMessage.getLinearSelectionMatrix().setZSelected(positionWeight.getZ() != 0.0);
      linearWeightMatrix.setZAxisWeight(positionWeight.getZ());
      rigidBodyMessage.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(linearWeightMatrix));

      WeightMatrix3D angularWeightMatrix = new WeightMatrix3D();
      rigidBodyMessage.getAngularSelectionMatrix().setXSelected(orientationWeight.getX() != 0.0);
      angularWeightMatrix.setXAxisWeight(orientationWeight.getX());
      rigidBodyMessage.getAngularSelectionMatrix().setYSelected(orientationWeight.getY() != 0.0);
      angularWeightMatrix.setYAxisWeight(orientationWeight.getY());
      rigidBodyMessage.getAngularSelectionMatrix().setZSelected(orientationWeight.getZ() != 0.0);
      angularWeightMatrix.setZAxisWeight(orientationWeight.getZ());
      rigidBodyMessage.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(angularWeightMatrix));

      rigidBodyMessage.setLinearRateLimitation(-1.0); // default value is used
      rigidBodyMessage.setAngularRateLimitation(-1.0); // default value is used

      rigidBodyMessage.getControlFramePositionInEndEffector().setToZero();
      rigidBodyMessage.getControlFrameOrientationInEndEffector().setToZero();

      rigidBodyMessage.setHasDesiredLinearVelocity(false);
      rigidBodyMessage.setHasDesiredAngularVelocity(false);

      ikInputMessage.getInputs().add().set(rigidBodyMessage);
   }

   private void lockChest(KinematicsStreamingToolboxInputMessage toolboxInputMessage)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getChest().hashCode());
      message.getDesiredOrientationInWorld().set(initialChestPose.getRotation());
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(0));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(100));

      toolboxInputMessage.getInputs().add().set(message);
   }

   private void lockPelvis(KinematicsStreamingToolboxInputMessage toolboxInputMessage)
   {
      KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
      message.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getPelvis().hashCode());
      message.getDesiredPositionInWorld().set(initialPelvisPose.getTranslation());
      message.getDesiredOrientationInWorld().set(initialPelvisPose.getRotation());
      message.getLinearWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(100));
      message.getAngularWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(100));

      toolboxInputMessage.getInputs().add().set(message);
   }

   public void setControlRobot(boolean controlRobot)
   {
      this.controlRobot = controlRobot;
   }
}
