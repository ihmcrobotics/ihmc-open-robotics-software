package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.sequence.actions.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class ROS2BehaviorTreeTools
{
   public static BehaviorTreeNodeState createNodeFromMessage(ROS2BehaviorTreeSubscriptionNode subscriptionNode,
                                                             ReferenceFrameLibrary referenceFrameLibrary,
                                                             FootstepPlannerParametersBasics footstepPlannerParametersBasics)
   {
      BehaviorTreeNodeState behaviorTreeNodeState;

      byte nodeType = subscriptionNode.getType();

      if (nodeType == BehaviorTreeStateMessage.ARM_JOINT_ANGLES_ACTION)
      {
         behaviorTreeNodeState = new ArmJointAnglesActionState();
      }
      else if (nodeType == BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION)
      {
         behaviorTreeNodeState = new ChestOrientationActionState(referenceFrameLibrary);
      }
      else if (nodeType == BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION)
      {
         behaviorTreeNodeState = new FootstepPlanActionState(referenceFrameLibrary);
      }
      else if (nodeType == BehaviorTreeStateMessage.HAND_POSE_ACTION)
      {
         behaviorTreeNodeState = new HandPoseActionState(referenceFrameLibrary);
      }
      else if (nodeType == BehaviorTreeStateMessage.HAND_WRENCH_ACTION)
      {
         behaviorTreeNodeState = new HandWrenchActionState();
      }
      else if (nodeType == BehaviorTreeStateMessage.PELVIS_HEIGHT_PITCH_ACTION)
      {
         behaviorTreeNodeState = new PelvisHeightPitchActionState(referenceFrameLibrary);
      }
      else if (nodeType == BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION)
      {
         behaviorTreeNodeState = new SakeHandCommandActionState();
      }
      else if (nodeType == BehaviorTreeStateMessage.WAIT_DURATION_ACTION)
      {
         behaviorTreeNodeState = new WaitDurationActionState();
      }
      else if (nodeType == BehaviorTreeStateMessage.WALK_ACTION)
      {
         behaviorTreeNodeState = new WalkActionState(referenceFrameLibrary, footstepPlannerParametersBasics);
      }
      else
      {
         behaviorTreeNodeState = null; // ??
      }

      return behaviorTreeNodeState;
   }

   public static Class<?> getNodeStateClass(byte nodeType)
   {
      return switch (nodeType)
      {
         case BehaviorTreeStateMessage.ARM_JOINT_ANGLES_ACTION -> ArmJointAnglesActionState.class;
         case BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION -> ChestOrientationActionState.class;
         case BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION -> FootstepPlanActionState.class;
         case BehaviorTreeStateMessage.HAND_POSE_ACTION -> HandPoseActionState.class;
         case BehaviorTreeStateMessage.HAND_WRENCH_ACTION -> HandWrenchActionState.class;
         case BehaviorTreeStateMessage.PELVIS_HEIGHT_PITCH_ACTION -> PelvisHeightPitchActionState.class;
         case BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION -> SakeHandCommandActionState.class;
         case BehaviorTreeStateMessage.WAIT_DURATION_ACTION -> WaitDurationActionState.class;
         case BehaviorTreeStateMessage.WALK_ACTION -> WalkActionState.class;
         default -> BehaviorTreeNodeState.class;
      };
   }
}
