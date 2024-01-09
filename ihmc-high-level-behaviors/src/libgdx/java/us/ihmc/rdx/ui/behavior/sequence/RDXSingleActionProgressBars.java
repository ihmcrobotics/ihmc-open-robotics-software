package us.ihmc.rdx.ui.behavior.sequence;

import us.ihmc.behaviors.sequence.actions.FootstepPlanActionState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;

// FIXME: This needs to be redone
public class RDXSingleActionProgressBars
{
   private RDXActionNode<?, ?> action;
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
}
