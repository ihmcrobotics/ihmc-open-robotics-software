package us.ihmc.rdx.ui.behavior.sequence;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;

public class RDXSingleActionProgressBars
{
   private RDXActionNode action;
   private ActionExecutionStatusMessage actionExecutionStatusMessage;

   public void setAction(RDXActionNode action)
   {
      this.action = action;
   }

   public RDXActionNode getAction()
   {
      return action;
   }

   public void setActionExecutionStatusMessage(ActionExecutionStatusMessage actionExecutionStatusMessage)
   {
      this.actionExecutionStatusMessage = actionExecutionStatusMessage;
   }

   public ActionExecutionStatusMessage getActionExecutionStatusMessage()
   {
      return actionExecutionStatusMessage;
   }
}
