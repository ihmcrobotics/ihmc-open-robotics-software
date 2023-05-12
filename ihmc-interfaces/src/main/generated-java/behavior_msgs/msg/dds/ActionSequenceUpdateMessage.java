package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionSequenceUpdateMessage extends Packet<ActionSequenceUpdateMessage> implements Settable<ActionSequenceUpdateMessage>, EpsilonComparable<ActionSequenceUpdateMessage>
{
   /**
            * Unique identifier of the sequence update used to make sure we got complete data
            */
   public long sequence_update_uuid_;
   /**
            * Number of actions within the sequence
            */
   public long sequence_size_;

   public ActionSequenceUpdateMessage()
   {
   }

   public ActionSequenceUpdateMessage(ActionSequenceUpdateMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionSequenceUpdateMessage other)
   {
      sequence_update_uuid_ = other.sequence_update_uuid_;

      sequence_size_ = other.sequence_size_;

   }

   /**
            * Unique identifier of the sequence update used to make sure we got complete data
            */
   public void setSequenceUpdateUuid(long sequence_update_uuid)
   {
      sequence_update_uuid_ = sequence_update_uuid;
   }
   /**
            * Unique identifier of the sequence update used to make sure we got complete data
            */
   public long getSequenceUpdateUuid()
   {
      return sequence_update_uuid_;
   }

   /**
            * Number of actions within the sequence
            */
   public void setSequenceSize(long sequence_size)
   {
      sequence_size_ = sequence_size;
   }
   /**
            * Number of actions within the sequence
            */
   public long getSequenceSize()
   {
      return sequence_size_;
   }


   public static Supplier<ActionSequenceUpdateMessagePubSubType> getPubSubType()
   {
      return ActionSequenceUpdateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionSequenceUpdateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionSequenceUpdateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_update_uuid_, other.sequence_update_uuid_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_size_, other.sequence_size_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionSequenceUpdateMessage)) return false;

      ActionSequenceUpdateMessage otherMyClass = (ActionSequenceUpdateMessage) other;

      if(this.sequence_update_uuid_ != otherMyClass.sequence_update_uuid_) return false;

      if(this.sequence_size_ != otherMyClass.sequence_size_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionSequenceUpdateMessage {");
      builder.append("sequence_update_uuid=");
      builder.append(this.sequence_update_uuid_);      builder.append(", ");
      builder.append("sequence_size=");
      builder.append(this.sequence_size_);
      builder.append("}");
      return builder.toString();
   }
}
