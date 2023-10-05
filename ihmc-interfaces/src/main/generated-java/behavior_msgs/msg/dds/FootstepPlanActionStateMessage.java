package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionStateMessage extends Packet<FootstepPlanActionStateMessage> implements Settable<FootstepPlanActionStateMessage>, EpsilonComparable<FootstepPlanActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorActionStateMessage action_state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage definition_;

   public FootstepPlanActionStateMessage()
   {
      action_state_ = new behavior_msgs.msg.dds.BehaviorActionStateMessage();
      definition_ = new behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage();
   }

   public FootstepPlanActionStateMessage(FootstepPlanActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.staticCopy(other.action_state_, action_state_);
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorActionStateMessage getActionState()
   {
      return action_state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<FootstepPlanActionStateMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_state_.epsilonEquals(other.action_state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionStateMessage)) return false;

      FootstepPlanActionStateMessage otherMyClass = (FootstepPlanActionStateMessage) other;

      if (!this.action_state_.equals(otherMyClass.action_state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionStateMessage {");
      builder.append("action_state=");
      builder.append(this.action_state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
