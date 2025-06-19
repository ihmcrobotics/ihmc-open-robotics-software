package us.ihmc.perception.lerobot;

import toolbox_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxConfigurationMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.SpineJointName;
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
   private final SideDependentList<Pose3D> actionHandPoses;
   private final FullHumanoidRobotModel fullRobotModel;
   private final KinematicsToolboxConfigurationMessage ikSolverConfigurationMessage = new KinematicsToolboxConfigurationMessage();
   private final Vector3D positionWeight = new Vector3D(-1.0, -1.0, -1.0); // default values are used
   private final Vector3D orientationWeight = new Vector3D(-1.0, -1.0, -1.0); // default values are used
   private final ROS2Publisher<KinematicsToolboxConfigurationMessage> configurationPublisher;
   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> inputPublisher;

   public LeRobotIKStreaming(SideDependentList<Pose3D> actionHandPoses, String robotName, ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      this.actionHandPoses = actionHandPoses;
      this.fullRobotModel = fullRobotModel;

      if (robotName.toLowerCase().contains("nadia"))
      {
         ikSolverConfigurationMessage.getJointsToDeactivate().add(fullRobotModel.getSpineJoint(SpineJointName.SPINE_PITCH).hashCode());
         ikSolverConfigurationMessage.getJointsToDeactivate().add(fullRobotModel.getSpineJoint(SpineJointName.SPINE_ROLL).hashCode());
      }

      configurationPublisher = ros2Node.createPublisher(ToolboxAPIs.getInputToolboxConfigurationTopic(robotName));
      inputPublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingInputTopic(robotName));
   }

   public void update(long actionTimestampNanos)
   {
      configurationPublisher.publish(ikSolverConfigurationMessage);

      KinematicsStreamingToolboxInputMessage ikInputMessage = prepareInputMessage(actionTimestampNanos);
      inputPublisher.publish(ikInputMessage);
   }

   /**
    * @param actionTimestampNanos TODO: Need to confirm, but this should be the time for the action
    *                               coming from the policy. I don't think it has to be corresponding with
    *                               controller time at all.
    */
   public KinematicsStreamingToolboxInputMessage prepareInputMessage(long actionTimestampNanos)
   {
      KinematicsStreamingToolboxInputMessage ikInputMessage = new KinematicsStreamingToolboxInputMessage();

      for (RobotSide side : RobotSide.values)
      {
         int endEffectorHashCode = fullRobotModel.getHand(side).hashCode();

         Pose3DReadOnly handControlPose = actionHandPoses.get(side);

         double linearMomentumLimit = -1.0; // default value is used
         double angularMomentumLimit = -1.0; // default value is used
         Vector3D desiredLinearVelocity = new Vector3D(); // TODO: These can probably be zero for moving slow
         Vector3D desiredAngularVelocity = new Vector3D();

         KinematicsToolboxRigidBodyMessage rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
         rigidBodyMessage.setEndEffectorHashCode(endEffectorHashCode);

         rigidBodyMessage.getDesiredOrientationInWorld().set(handControlPose.getOrientation());
         rigidBodyMessage.getDesiredPositionInWorld().set(handControlPose.getPosition());

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

         rigidBodyMessage.setLinearRateLimitation(linearMomentumLimit);
         rigidBodyMessage.setAngularRateLimitation(angularMomentumLimit);

         rigidBodyMessage.getControlFramePositionInEndEffector().setToZero(); // TODO: Make sure this is right
         rigidBodyMessage.getControlFrameOrientationInEndEffector().setToZero();

         rigidBodyMessage.setHasDesiredLinearVelocity(true);
         rigidBodyMessage.getDesiredLinearVelocityInWorld().set(desiredLinearVelocity);
         rigidBodyMessage.setHasDesiredAngularVelocity(true);
         rigidBodyMessage.getDesiredAngularVelocityInWorld().set(desiredAngularVelocity);

         ikInputMessage.getInputs().add().set(rigidBodyMessage);
      }

      ikInputMessage.setTimestamp(actionTimestampNanos);

      return ikInputMessage;
   }
}
