package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.BehaviorTreeStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.sequence.actions.*;

public class ROS2BehaviorTreeTools
{
   public static Class<?> getNodeStateClass(byte nodeType)
   {
      return switch (nodeType)
      {
         case BehaviorTreeStateMessage.ARM_JOINT_ANGLES_ACTION -> ArmJointAnglesActionDefinition.class;
         case BehaviorTreeStateMessage.CHEST_ORIENTATION_ACTION -> ChestOrientationActionDefinition.class;
         case BehaviorTreeStateMessage.FOOTSTEP_PLAN_ACTION -> FootstepPlanActionDefinition.class;
         case BehaviorTreeStateMessage.HAND_POSE_ACTION -> HandPoseActionDefinition.class;
         case BehaviorTreeStateMessage.HAND_WRENCH_ACTION -> HandWrenchActionDefinition.class;
         case BehaviorTreeStateMessage.PELVIS_HEIGHT_PITCH_ACTION -> PelvisHeightPitchActionDefinition.class;
         case BehaviorTreeStateMessage.SAKE_HAND_COMMAND_ACTION -> SakeHandCommandActionDefinition.class;
         case BehaviorTreeStateMessage.WAIT_DURATION_ACTION -> WaitDurationActionDefinition.class;
         case BehaviorTreeStateMessage.WALK_ACTION -> WalkActionDefinition.class;
         default -> BehaviorTreeNodeState.class;
      };
   }
}
