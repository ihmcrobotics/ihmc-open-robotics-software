package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PelvisHeightPitchActionStateMessage extends Packet<PelvisHeightPitchActionStateMessage> implements Settable<PelvisHeightPitchActionStateMessage>, EpsilonComparable<PelvisHeightPitchActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorActionStateMessage action_state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage definition_;

   public PelvisHeightPitchActionStateMessage()
   {
      action_state_ = new behavior_msgs.msg.dds.BehaviorActionStateMessage();
      definition_ = new behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage();
   }

   public PelvisHeightPitchActionStateMessage(PelvisHeightPitchActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(PelvisHeightPitchActionStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.staticCopy(other.action_state_, action_state_);
      behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
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
   public behavior_msgs.msg.dds.PelvisHeightPitchActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<PelvisHeightPitchActionStateMessagePubSubType> getPubSubType()
   {
      return PelvisHeightPitchActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PelvisHeightPitchActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PelvisHeightPitchActionStateMessage other, double epsilon)
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
      if(!(other instanceof PelvisHeightPitchActionStateMessage)) return false;

      PelvisHeightPitchActionStateMessage otherMyClass = (PelvisHeightPitchActionStateMessage) other;

      if (!this.action_state_.equals(otherMyClass.action_state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PelvisHeightPitchActionStateMessage {");
      builder.append("action_state=");
      builder.append(this.action_state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
