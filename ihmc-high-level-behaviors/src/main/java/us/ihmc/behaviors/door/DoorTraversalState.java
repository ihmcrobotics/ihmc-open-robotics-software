package us.ihmc.behaviors.door;

import behavior_msgs.msg.dds.DoorTraversalStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class DoorTraversalState extends BehaviorTreeNodeState<DoorTraversalDefinition>
{
   public static final String STABILIZE_DETECTION = "Stabilize Detection";
   public static final String WAIT_TO_OPEN_RIGHT_HAND = "Wait to open right hand";
   public static final String PULL_SCREW_PRIMITIVE = "Pull Screw primitive";

   private ActionSequenceState actionSequence;
   private WaitDurationActionState stabilizeDetectionAction;
   private WaitDurationActionState waitToOpenRightHandAction;
   private ScrewPrimitiveActionState pullScrewPrimitiveAction;

   public DoorTraversalState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new DoorTraversalDefinition(crdtInfo, saveFileDirectory), crdtInfo);
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
      stabilizeDetectionAction = null;
      waitToOpenRightHandAction = null;
      pullScrewPrimitiveAction = null;

      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {
            if (actionNode instanceof WaitDurationActionState waitAction
                && waitAction.getDefinition().getName().equals(STABILIZE_DETECTION))
            {
               stabilizeDetectionAction = waitAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(WAIT_TO_OPEN_RIGHT_HAND))
            {
               waitToOpenRightHandAction = waitDurationAction;
            }
            if (actionNode instanceof ScrewPrimitiveActionState screwPrimitiveAction
                && screwPrimitiveAction.getDefinition().getName().equals(PULL_SCREW_PRIMITIVE))
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
   }

   public void fromMessage(DoorTraversalStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }

   public boolean arePullRetryNodesPresent()
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

   public WaitDurationActionState getStabilizeDetectionAction()
   {
      return stabilizeDetectionAction;
   }

   public WaitDurationActionState getWaitToOpenRightHandAction()
   {
      return waitToOpenRightHandAction;
   }

   public ScrewPrimitiveActionState getPullScrewPrimitiveAction()
   {
      return pullScrewPrimitiveAction;
   }
}
