package us.ihmc.rdx.ui.behavior.editor;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;

public class RDXSingleActionProgressBars
{
   private RDXBehaviorAction action;
   private ActionExecutionStatusMessage actionExecutionStatusMessage;

   public void setAction(RDXBehaviorAction action)
   {
      this.action = action;
   }

   public RDXBehaviorAction getAction()
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
