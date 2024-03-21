package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalNotification;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalState extends BehaviorTreeNodeState<DoorTraversalDefinition>
{
   private ActionSequenceState actionSequence;
   private WaitDurationActionState waitToOpenRightHandAction;
   private ScrewPrimitiveActionState pullScrewPrimitiveAction;

   private final CRDTUnidirectionalNotification retryingPullDoorNotification;

   public DoorTraversalState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new DoorTraversalDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      retryingPullDoorNotification = new CRDTUnidirectionalNotification(ROS2ActorDesignation.ROBOT, crdtInfo, this);
   }

   @Override
   public void update()
   {
      super.update();

      actionSequence = BehaviorTreeTools.findActionSequenceAncestor(this);

      updateActionSubtree(this);
   }

   public void updateActionSubtree(BehaviorTreeNodeState<?> node)
   {
      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals("Wait to open right hand"))
            {
               waitToOpenRightHandAction = waitDurationAction;
            }
            if (actionNode instanceof ScrewPrimitiveActionState screwPrimitiveAction
                && screwPrimitiveAction.getDefinition().getName().equals("Pull Screw primitive"))
            {
               pullScrewPrimitiveAction = screwPrimitiveAction;
            }
         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   public void toMessage(DoorTraversalStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setRetryingPullDoorNotification(retryingPullDoorNotification.toMessage());
   }

   public void fromMessage(DoorTraversalStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      retryingPullDoorNotification.fromMessage(message.getRetryingPullDoorNotification());
   }

   public boolean isTreeStructureValid()
   {
      boolean isValid = actionSequence != null;
      isValid &= waitToOpenRightHandAction != null;
      isValid &= pullScrewPrimitiveAction != null;
      return isValid;
   }

   public ActionSequenceState getActionSequence()
   {
      return actionSequence;
   }

   public WaitDurationActionState getWaitToOpenRightHandAction()
   {
      return waitToOpenRightHandAction;
   }

   public ScrewPrimitiveActionState getPullScrewPrimitiveAction()
   {
      return pullScrewPrimitiveAction;
   }

   public CRDTUnidirectionalNotification getRetryingPullDoorNotification()
   {
      return retryingPullDoorNotification;
   }
}
