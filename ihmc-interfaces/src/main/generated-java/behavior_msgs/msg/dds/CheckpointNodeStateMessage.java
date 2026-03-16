package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class CheckpointNodeStateMessage extends Packet<CheckpointNodeStateMessage> implements Settable<CheckpointNodeStateMessage>, EpsilonComparable<CheckpointNodeStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.LeafNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.CheckpointNodeDefinitionMessage definition_;

   public CheckpointNodeStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.LeafNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.CheckpointNodeDefinitionMessage();
   }

   public CheckpointNodeStateMessage(CheckpointNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(CheckpointNodeStateMessage other)
   {
      behavior_msgs.msg.dds.LeafNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.CheckpointNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.LeafNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.CheckpointNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<CheckpointNodeStateMessagePubSubType> getPubSubType()
   {
      return CheckpointNodeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CheckpointNodeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CheckpointNodeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CheckpointNodeStateMessage)) return false;

      CheckpointNodeStateMessage otherMyClass = (CheckpointNodeStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CheckpointNodeStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
