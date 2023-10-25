package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;

import java.util.ArrayList;
import java.util.List;

public class ROS2BehaviorTreeSubscriptionNode
{
   private byte type;
   private BehaviorTreeNodeStateMessage behaviorTreeNodeStateMessage;
   private ActionNodeStateMessage behaviorActionStateMessage;
   private ArmJointAnglesActionStateMessage armJointAnglesActionStateMessage;
   private ChestOrientationActionStateMessage chestOrientationActionStateMessage;
   private FootstepPlanActionStateMessage footstepPlanActionStateMessage;
   private SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage;
   private HandPoseActionStateMessage handPoseActionStateMessage;
   private HandWrenchActionStateMessage handWrenchActionStateMessage;
   private PelvisHeightPitchActionStateMessage pelvisHeightPitchActionStateMessage;
   private WaitDurationActionStateMessage waitDurationActionStateMessage;
   private WalkActionStateMessage walkActionStateMessage;
   private final List<ROS2BehaviorTreeSubscriptionNode> children = new ArrayList<>();

   public void clear()
   {
      type = -1;
      armJointAnglesActionStateMessage = null;
   }

   public byte getType()
   {
      return type;
   }

   public void setType(byte type)
   {
      this.type = type;
   }

   public BehaviorTreeNodeStateMessage getBehaviorTreeNodeStateMessage()
   {
      return behaviorTreeNodeStateMessage;
   }

   public void setBehaviorTreeNodeStateMessage(BehaviorTreeNodeStateMessage behaviorTreeNodeStateMessage)
   {
      this.behaviorTreeNodeStateMessage = behaviorTreeNodeStateMessage;
   }

   public ActionNodeStateMessage getActionNodeStateMessage()
   {
      return behaviorActionStateMessage;
   }

   public void setActionNodeStateMessage(ActionNodeStateMessage actionNodeStateMessage)
   {
      this.behaviorActionStateMessage = actionNodeStateMessage;
   }

   public ArmJointAnglesActionStateMessage getArmJointAnglesActionStateMessage()
   {
      return armJointAnglesActionStateMessage;
   }

   public void setArmJointAnglesActionStateMessage(ArmJointAnglesActionStateMessage armJointAnglesActionStateMessage)
   {
      this.armJointAnglesActionStateMessage = armJointAnglesActionStateMessage;
   }

   public ChestOrientationActionStateMessage getChestOrientationActionStateMessage()
   {
      return chestOrientationActionStateMessage;
   }

   public void setChestOrientationActionStateMessage(ChestOrientationActionStateMessage chestOrientationActionStateMessage)
   {
      this.chestOrientationActionStateMessage = chestOrientationActionStateMessage;
   }

   public FootstepPlanActionStateMessage getFootstepPlanActionStateMessage()
   {
      return footstepPlanActionStateMessage;
   }

   public void setFootstepPlanActionStateMessage(FootstepPlanActionStateMessage footstepPlanActionStateMessage)
   {
      this.footstepPlanActionStateMessage = footstepPlanActionStateMessage;
   }

   public SakeHandCommandActionStateMessage getSakeHandCommandActionStateMessage()
   {
      return sakeHandCommandActionStateMessage;
   }

   public void setSakeHandCommandActionStateMessage(SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage)
   {
      this.sakeHandCommandActionStateMessage = sakeHandCommandActionStateMessage;
   }

   public HandPoseActionStateMessage getHandPoseActionStateMessage()
   {
      return handPoseActionStateMessage;
   }

   public void setHandPoseActionStateMessage(HandPoseActionStateMessage handPoseActionStateMessage)
   {
      this.handPoseActionStateMessage = handPoseActionStateMessage;
   }

   public HandWrenchActionStateMessage getHandWrenchActionStateMessage()
   {
      return handWrenchActionStateMessage;
   }

   public void setHandWrenchActionStateMessage(HandWrenchActionStateMessage handWrenchActionStateMessage)
   {
      this.handWrenchActionStateMessage = handWrenchActionStateMessage;
   }

   public PelvisHeightPitchActionStateMessage getPelvisHeightPitchActionStateMessage()
   {
      return pelvisHeightPitchActionStateMessage;
   }

   public void setPelvisHeightPitchActionStateMessage(PelvisHeightPitchActionStateMessage pelvisHeightPitchActionStateMessage)
   {
      this.pelvisHeightPitchActionStateMessage = pelvisHeightPitchActionStateMessage;
   }

   public WaitDurationActionStateMessage getWaitDurationActionStateMessage()
   {
      return waitDurationActionStateMessage;
   }

   public void setWaitDurationActionStateMessage(WaitDurationActionStateMessage waitDurationActionStateMessage)
   {
      this.waitDurationActionStateMessage = waitDurationActionStateMessage;
   }

   public WalkActionStateMessage getWalkActionStateMessage()
   {
      return walkActionStateMessage;
   }

   public void setWalkActionStateMessage(WalkActionStateMessage walkActionStateMessage)
   {
      this.walkActionStateMessage = walkActionStateMessage;
   }

   public List<ROS2BehaviorTreeSubscriptionNode> getChildren()
   {
      return children;
   }
}
