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
   private DoorTraversalStateMessage doorTraversalStateMessage;
   private TrashCanInteractionStateMessage trashCanInteractionStateMessage;
   private ActionNodeStateMessage actionNodeStateMessage;
   private ChestOrientationActionStateMessage chestOrientationActionStateMessage;
   private FootstepPlanActionStateMessage footstepPlanActionStateMessage;
   private SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage;
   private HandPoseActionStateMessage handPoseActionStateMessage;
   private HandWrenchActionStateMessage handWrenchActionStateMessage;
   private ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage;
   private PelvisHeightOrientationActionStateMessage pelvisHeightOrientationActionStateMessage;
   private WaitDurationActionStateMessage waitDurationActionStateMessage;
   private FootPoseActionStateMessage footPoseActionStateMessage;
   private final List<ROS2BehaviorTreeSubscriptionNode> children = new ArrayList<>();

   public void clear()
   {
      type = -1;
      behaviorTreeNodeDefinitionMessage = null;
      behaviorTreeNodeStateMessage = null;
      actionSequenceStateMessage = null;
      doorTraversalStateMessage = null;
      trashCanInteractionStateMessage = null;
      actionNodeStateMessage = null;
      chestOrientationActionStateMessage = null;
      footstepPlanActionStateMessage = null;
      sakeHandCommandActionStateMessage = null;
      handPoseActionStateMessage = null;
      handWrenchActionStateMessage = null;
      screwPrimitiveActionStateMessage = null;
      pelvisHeightOrientationActionStateMessage = null;
      waitDurationActionStateMessage = null;
      footPoseActionStateMessage = null;
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

   public DoorTraversalStateMessage getDoorTraversalStateMessage()
   {
      return doorTraversalStateMessage;
   }

   public void setDoorTraversalStateMessage(DoorTraversalStateMessage doorTraversalStateMessage)
   {
      this.doorTraversalStateMessage = doorTraversalStateMessage;
   }

   public TrashCanInteractionStateMessage getTrashCanInteractionStateMessage()
   {
      return trashCanInteractionStateMessage;
   }

   public void setTrashCanInteractionStateMessage(TrashCanInteractionStateMessage trashCanInteractionStateMessage)
   {
      this.trashCanInteractionStateMessage = trashCanInteractionStateMessage;
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

   public ScrewPrimitiveActionStateMessage getScrewPrimitiveActionStateMessage()
   {
      return screwPrimitiveActionStateMessage;
   }

   public void setScrewPrimitiveActionStateMessage(ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage)
   {
      this.screwPrimitiveActionStateMessage = screwPrimitiveActionStateMessage;
   }

   public PelvisHeightOrientationActionStateMessage getPelvisHeightOrientationActionStateMessage()
   {
      return pelvisHeightOrientationActionStateMessage;
   }

   public void setPelvisHeightOrientationActionStateMessage(PelvisHeightOrientationActionStateMessage pelvisHeightOrientationActionStateMessage)
   {
      this.pelvisHeightOrientationActionStateMessage = pelvisHeightOrientationActionStateMessage;
   }

   public WaitDurationActionStateMessage getWaitDurationActionStateMessage()
   {
      return waitDurationActionStateMessage;
   }

   public void setWaitDurationActionStateMessage(WaitDurationActionStateMessage waitDurationActionStateMessage)
   {
      this.waitDurationActionStateMessage = waitDurationActionStateMessage;
   }

   public FootPoseActionStateMessage getFootPoseActionStateMessage()
   {
      return footPoseActionStateMessage;
   }

   public void setFootPoseActionStateMessage(FootPoseActionStateMessage footPoseActionStateMessage)
   {
      this.footPoseActionStateMessage = footPoseActionStateMessage;
   }

   public List<ROS2BehaviorTreeSubscriptionNode> getChildren()
   {
      return children;
   }
}
