package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandWrenchActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class HandWrenchActionState extends ActionNodeState<HandWrenchActionDefinition>
{
   public HandWrenchActionState(long id, ROS2ActorDesignation actorDesignation)
   {
      super(id, new HandWrenchActionDefinition(), actorDesignation);
   }

   public void toMessage(HandWrenchActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(HandWrenchActionStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
