package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeState;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.communication.AutonomyAPI;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;

public class ROS2BehaviorTreePublisher
{
   private ROS2IOTopicQualifier ioQualifier;
   private final BehaviorTreeStateMessage behaviorTreeMessage = new BehaviorTreeStateMessage();

   public void publish(BehaviorTreeState behaviorTreeState, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI, ROS2IOTopicQualifier outgoingQualifier)
   {
      this.ioQualifier = ioQualifier;

      behaviorTreeMessage.setNextId(behaviorTreeState.getNextID().intValue());
      behaviorTreeMessage.getBehaviorTreeTypes().clear();
      behaviorTreeMessage.getBehaviorTreeIndices().clear();
      behaviorTreeMessage.getArmJointAnglesActions().clear();
      behaviorTreeMessage.getChestOrientationActions().clear();
      behaviorTreeMessage.getFootstepPlanActions().clear();
      behaviorTreeMessage.getHandPoseActions().clear();
      behaviorTreeMessage.getHandWrenchActions().clear();
      behaviorTreeMessage.getPelvisHeightActions().clear();
      behaviorTreeMessage.getSakeHandCommandActions().clear();
      behaviorTreeMessage.getWaitDurationActions().clear();
      behaviorTreeMessage.getWalkActions().clear();

      packSceneTreeToMessage(behaviorTreeState.getRootNode());

      ros2PublishSubscribeAPI.publish(AutonomyAPI.BEAVIOR_TREE.getTopic(ioQualifier), behaviorTreeMessage);
   }

   private void packSceneTreeToMessage(BehaviorTreeNodeState behaviorTreeNode)
   {
      if (behaviorTreeNode instanceof ArmJointAnglesActionState armJointAnglesActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.ARM_JOINT_ANGLES_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getArmJointAnglesActions().size());
         armJointAnglesActionState.toMessage(behaviorTreeMessage.getArmJointAnglesActions().add());
      }
      else if (behaviorTreeNode instanceof ChestOrientationActionState chestOrientationActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getChestOrientationActions().size());
         chestOrientationActionState.toMessage(behaviorTreeMessage.getChestOrientationActions().add());
      }
      else if (behaviorTreeNode instanceof FootstepPlanActionState footstepPlanActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getFootstepPlanActions().size());
         footstepPlanActionState.toMessage(behaviorTreeMessage.getFootstepPlanActions().add());
      }
      else if (behaviorTreeNode instanceof SakeHandCommandActionState sakeHandCommandActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getSakeHandCommandActions().size());
         sakeHandCommandActionState.toMessage(behaviorTreeMessage.getSakeHandCommandActions().add());
      }
      else if (behaviorTreeNode instanceof HandPoseActionState handPoseActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.HAND_POSE_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getHandPoseActions().size());
         handPoseActionState.toMessage(behaviorTreeMessage.getHandPoseActions().add());
      }
      else if (behaviorTreeNode instanceof HandWrenchActionState handWrenchActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.HAND_WRENCH_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getHandWrenchActions().size());
         handWrenchActionState.toMessage(behaviorTreeMessage.getHandWrenchActions().add());
      }
      else if (behaviorTreeNode instanceof PelvisHeightPitchActionState pelvisHeightActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.PELVIS_HEIGHT_PITCH_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getPelvisHeightActions().size());
         pelvisHeightActionState.toMessage(behaviorTreeMessage.getPelvisHeightActions().add());
      }
      else if (behaviorTreeNode instanceof WaitDurationActionState waitDurationActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.WAIT_DURATION_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getWaitDurationActions().size());
         waitDurationActionState.toMessage(behaviorTreeMessage.getWaitDurationActions().add());
      }
      else if (behaviorTreeNode instanceof WalkActionState walkActionState)
      {
         behaviorTreeMessage.getBehaviorTreeTypes().add(BehaviorTreeStateMessage.WALK_ACTION);
         behaviorTreeMessage.getBehaviorTreeIndices().add(behaviorTreeMessage.getWalkActions().size());
         walkActionState.toMessage(behaviorTreeMessage.getWalkActions().add());
      }

      for (BehaviorTreeNodeState child : behaviorTreeNode.getChildren())
      {
         packSceneTreeToMessage(child);
      }
   }
}
