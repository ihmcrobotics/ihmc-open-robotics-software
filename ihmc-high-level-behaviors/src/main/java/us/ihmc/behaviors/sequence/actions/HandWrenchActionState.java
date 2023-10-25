package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;

public class HandWrenchActionState extends ActionNodeState<HandWrenchActionDefinition>
{
   public HandWrenchActionState(long id)
   {
      super(id, new HandWrenchActionDefinition());
   }

   public void toMessage(HandWrenchActionStateMessage message)
   {
      super.toMessage(message.getActionState());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      super.fromMessage(message.getActionState());
   }
}
