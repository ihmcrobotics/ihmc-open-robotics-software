package us.ihmc.rdx.ui.behavior.sequence;

import behavior_msgs.msg.dds.ActionNodeStateMessage;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;

// FIXME: This needs to be redone
public class RDXSingleActionProgressBars
{
   private RDXActionNode<?, ?> action;
   private ActionNodeStateMessage actionNodeStateMessage;
   private FootstepPlanActionState footstepPlanActionState;
   private HandPoseActionState handPoseActionState;

   public void setAction(RDXActionNode<?, ?> action)
   {
      this.action = action;
   }

   public RDXActionNode<?, ?> getAction()
   {
      return action;
   }

   public ActionNodeStateMessage getActionNodeStateMessage()
   {
      return actionNodeStateMessage;
   }

   public void setActionNodeStateMessage(ActionNodeStateMessage actionNodeStateMessage)
   {
      this.actionNodeStateMessage = actionNodeStateMessage;
   }

   public FootstepPlanActionState getFootstepPlanActionState()
   {
      return footstepPlanActionState;
   }

   public HandPoseActionState getHandPoseActionState()
   {
      return handPoseActionState;
   }
}
