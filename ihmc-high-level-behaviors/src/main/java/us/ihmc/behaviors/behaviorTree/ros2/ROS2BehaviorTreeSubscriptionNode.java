package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;

import java.util.ArrayList;
import java.util.List;

public class ROS2BehaviorTreeSubscriptionNode
{
   private byte type;
   private BehaviorTreeNodeDefinitionMessage behaviorTreeNodeDefinitionMessage;
   private BehaviorTreeNodeStateMessage behaviorTreeNodeStateMessage;
   private ActionSequenceStateMessage actionSequenceStateMessage;
   private ActionNodeStateMessage actionNodeStateMessage;
   private ArmJointAnglesActionStateMessage armJointAnglesActionStateMessage;
   private ChestOrientationActionStateMessage chestOrientationActionStateMessage;
   private FootstepPlanActionStateMessage footstepPlanActionStateMessage;
   private SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage;
   private HandPoseActionStateMessage handPoseActionStateMessage;
   private HandWrenchActionStateMessage handWrenchActionStateMessage;
   private PelvisHeightPitchActionStateMessage pelvisHeightPitchActionStateMessage;
   private WaitDurationActionStateMessage waitDurationActionStateMessage;
   private final List<ROS2BehaviorTreeSubscriptionNode> children = new ArrayList<>();

   public void clear()
   {
      type = -1;
      behaviorTreeNodeDefinitionMessage = null;
      behaviorTreeNodeStateMessage = null;
      actionSequenceStateMessage = null;
      actionNodeStateMessage = null;
      armJointAnglesActionStateMessage = null;
      chestOrientationActionStateMessage = null;
      footstepPlanActionStateMessage = null;
      sakeHandCommandActionStateMessage = null;
      handPoseActionStateMessage = null;
      handWrenchActionStateMessage = null;
      pelvisHeightPitchActionStateMessage = null;
      waitDurationActionStateMessage = null;
      children.clear();
   }

   public byte getType()
   {
      return type;
   }

   public void setType(byte type)
   {
      this.type = type;
   }

   public BehaviorTreeNodeDefinitionMessage getBehaviorTreeNodeDefinitionMessage()
   {
      return behaviorTreeNodeDefinitionMessage;
   }

   public void setBehaviorTreeNodeDefinitionMessage(BehaviorTreeNodeDefinitionMessage behaviorTreeNodeDefinitionMessage)
   {
      this.behaviorTreeNodeDefinitionMessage = behaviorTreeNodeDefinitionMessage;
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
      return actionNodeStateMessage;
   }

   public void setActionNodeStateMessage(ActionNodeStateMessage actionNodeStateMessage)
   {
      this.actionNodeStateMessage = actionNodeStateMessage;
   }

   public ActionSequenceStateMessage getActionSequenceStateMessage()
   {
      return actionSequenceStateMessage;
   }

   public void setActionSequenceStateMessage(ActionSequenceStateMessage actionSequenceStateMessage)
   {
      this.actionSequenceStateMessage = actionSequenceStateMessage;
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

   public List<ROS2BehaviorTreeSubscriptionNode> getChildren()
   {
      return children;
   }
}
