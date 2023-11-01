package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.SakeHandCommandActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;

public class SakeHandCommandActionState extends ActionNodeState<SakeHandCommandActionDefinition>
{
   public SakeHandCommandActionState(long id, ROS2ActorDesignation actorDesignation)
   {
      super(id, new SakeHandCommandActionDefinition(), actorDesignation);
   }

   public void toMessage(SakeHandCommandActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(SakeHandCommandActionStateMessage message)
   {
      getDefinition().fromMessage(message.getDefinition());

      super.fromMessage(message.getState());
   }
}
