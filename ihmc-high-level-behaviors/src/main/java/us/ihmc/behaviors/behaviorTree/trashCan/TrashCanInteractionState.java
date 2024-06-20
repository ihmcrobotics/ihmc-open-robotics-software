package us.ihmc.behaviors.behaviorTree.trashCan;

import behavior_msgs.msg.dds.TrashCanInteractionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.FootPoseActionState;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalEnumField;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class TrashCanInteractionState extends BehaviorTreeNodeState<TrashCanInteractionDefinition>
{
   public static final String COMPUTE_STANCE = "Compute Stance";
   public static final String APPROACHING_LEFT = "Approaching Left";
   public static final String APPROACHING_FRONT = "Approaching Front";
   public static final String APPROACHING_RIGHT = "Approaching Right";
   public static final String APPROACH_LEFT = "Approach Left";
   public static final String APPROACH_FRONT = "Approach Front";
   public static final String APPROACH_RIGHT = "Approach Right";
   public static final String LEFT_FOOT_DOWN = "Left Foot Down";
   public static final String RIGHT_FOOT_DOWN = "Right Foot Down";
   public static final String END = "End";

   private ActionSequenceState actionSequence;
   private WaitDurationActionState computeStanceAction;
   private WaitDurationActionState approachingLeftAction;
   private WaitDurationActionState approachingRightAction;
   private WaitDurationActionState approachingFrontAction;
   private FootstepPlanActionState approachLeftAction;
   private FootstepPlanActionState approachRightAction;
   private FootstepPlanActionState approachFrontAction;
   private FootPoseActionState leftFootDownAction;
   private FootPoseActionState rightFootDownAction;
   private WaitDurationActionState endAction;
   private final CRDTUnidirectionalEnumField<InteractionStance> stance;

   public TrashCanInteractionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new TrashCanInteractionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      stance = new CRDTUnidirectionalEnumField(ROS2ActorDesignation.ROBOT, crdtInfo, InteractionStance.LEFT);
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
      computeStanceAction = null;
      approachingLeftAction = null;
      approachingRightAction = null;
      approachingFrontAction = null;
      approachLeftAction = null;
      approachRightAction = null;
      approachFrontAction = null;
      rightFootDownAction = null;
      leftFootDownAction = null;
      endAction = null;

      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(COMPUTE_STANCE))
            {
               computeStanceAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(APPROACHING_LEFT))
            {
               approachingLeftAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(APPROACHING_RIGHT))
            {
               approachingRightAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(APPROACHING_FRONT))
            {
               approachingFrontAction = waitDurationAction;
            }
            if (actionNode instanceof FootstepPlanActionState footstepPlanAction
                && footstepPlanAction.getDefinition().getName().equals(APPROACH_LEFT))
            {
               approachLeftAction = footstepPlanAction;
            }
            if (actionNode instanceof FootstepPlanActionState footstepPlanAction
                && footstepPlanAction.getDefinition().getName().equals(APPROACH_FRONT))
            {
               approachFrontAction = footstepPlanAction;
            }
            if (actionNode instanceof FootstepPlanActionState footstepPlanAction
                && footstepPlanAction.getDefinition().getName().equals(APPROACH_RIGHT))
            {
               approachRightAction = footstepPlanAction;
            }
            if (actionNode instanceof FootPoseActionState footPoseAction
                && footPoseAction.getDefinition().getName().equals(LEFT_FOOT_DOWN))
            {
               leftFootDownAction = footPoseAction;
            }
            if (actionNode instanceof FootPoseActionState footPoseAction
                && footPoseAction.getDefinition().getName().equals(RIGHT_FOOT_DOWN))
            {
               rightFootDownAction = footPoseAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END))
            {
               endAction = waitDurationAction;
            }
         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   public void toMessage(TrashCanInteractionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      message.setStance(stance.toMessageOrdinal());
   }

   public void fromMessage(TrashCanInteractionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      stance.fromMessageOrdinal(message.getStance(), InteractionStance.values);
   }

   public boolean areLogicNodesPresent()
   {
      boolean isValid = actionSequence != null;
      isValid &= computeStanceAction != null;
      isValid &= approachingLeftAction != null;
      isValid &= approachingRightAction != null;
      isValid &= approachingFrontAction != null;
      isValid &= approachLeftAction != null;
      isValid &= approachRightAction != null;
      isValid &= approachFrontAction != null;
      isValid &= leftFootDownAction != null;
      isValid &= endAction != null;
      return isValid;
   }

   public ActionSequenceState getActionSequence()
   {
      return actionSequence;
   }

   public WaitDurationActionState getComputeStanceAction()
   {
      return computeStanceAction;
   }

   public WaitDurationActionState getApproachingLeftAction()
   {
      return approachingLeftAction;
   }

   public WaitDurationActionState getApproachingFrontAction()
   {
      return approachingFrontAction;
   }

   public WaitDurationActionState getApproachingRightAction()
   {
      return approachingRightAction;
   }

   public FootstepPlanActionState getApproachLeftAction()
   {
      return approachLeftAction;
   }

   public FootstepPlanActionState getApproachFrontAction()
   {
      return approachFrontAction;
   }

   public FootstepPlanActionState getApproachRightAction()
   {
      return approachRightAction;
   }

   public WaitDurationActionState getEndAction()
   {
      return endAction;
   }

   public FootPoseActionState getSetLeftFootDownAction()
   {
      return leftFootDownAction;
   }

   public FootPoseActionState getSetRightFootDownAction()
   {
      return rightFootDownAction;
   }

   public CRDTUnidirectionalEnumField getStance()
   {
      return stance;
   }

   public void setStance(InteractionStance stance)
   {
      this.stance.setValue(stance);
   }
}
