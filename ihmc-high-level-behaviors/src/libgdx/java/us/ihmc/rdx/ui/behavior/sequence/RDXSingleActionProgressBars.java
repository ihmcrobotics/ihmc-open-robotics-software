package us.ihmc.rdx.ui.behavior.sequence;

import behavior_msgs.msg.dds.ActionNodeStateMessage;

public class RDXSingleActionProgressBars
{
   private RDXActionNode<?, ?> action;
   private ActionNodeStateMessage actionNodeStateMessage;

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
}
