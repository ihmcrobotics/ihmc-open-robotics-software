package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.AbilityHandActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTStatusFloatArray;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class AbilityHandActionState extends ActionNodeState<AbilityHandActionDefinition>
{
   private final CRDTStatusFloatArray currentFingerPositions;
   private final CRDTStatusFloatArray desiredFingerPositions;

   public AbilityHandActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new AbilityHandActionDefinition(rootNode.getDefinition()), rootNode);

      currentFingerPositions = new CRDTStatusFloatArray(ROS2ActorDesignation.ROBOT, crdtInfo, 6);
      desiredFingerPositions = new CRDTStatusFloatArray(ROS2ActorDesignation.ROBOT, crdtInfo, 6);
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = super.hasStatus();
      hasStatus |= currentFingerPositions.pollHasStatus();
      hasStatus |= desiredFingerPositions.pollHasStatus();
      return hasStatus;
   }


   public void toMessage(AbilityHandActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      currentFingerPositions.toMessage(message.getCurrentFingerPositions());
      desiredFingerPositions.toMessage(message.getDesiredFingerPositions());

      super.toMessage(message.getState());
   }

   public void fromMessage(AbilityHandActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      currentFingerPositions.fromMessage(message.getCurrentFingerPositions());
      desiredFingerPositions.fromMessage(message.getDesiredFingerPositions());

      super.fromMessage(message.getState());
   }

   public CRDTStatusFloatArray getCurrentFingerPositions()
   {
      return currentFingerPositions;
   }

   public CRDTStatusFloatArray getDesiredFingerPositions()
   {
      return desiredFingerPositions;
   }
}
