package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ChestOrientationActionStateMessage extends Packet<ChestOrientationActionStateMessage> implements Settable<ChestOrientationActionStateMessage>, EpsilonComparable<ChestOrientationActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage action_state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage definition_;

   public ChestOrientationActionStateMessage()
   {
      action_state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage();
   }

   public ChestOrientationActionStateMessage(ChestOrientationActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ChestOrientationActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.action_state_, action_state_);
      behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage getActionState()
   {
      return action_state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ChestOrientationActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<ChestOrientationActionStateMessagePubSubType> getPubSubType()
   {
      return ChestOrientationActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ChestOrientationActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ChestOrientationActionStateMessage other, double epsilon)
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
      if(!(other instanceof ChestOrientationActionStateMessage)) return false;

      ChestOrientationActionStateMessage otherMyClass = (ChestOrientationActionStateMessage) other;

      if (!this.action_state_.equals(otherMyClass.action_state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChestOrientationActionStateMessage {");
      builder.append("action_state=");
      builder.append(this.action_state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
