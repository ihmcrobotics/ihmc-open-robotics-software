package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to clear the delay buffers on the controller, if you sent a message with a
       * delay and now you do not want them executed, use this command.
       */
public class ClearDelayQueueMessage extends Packet<ClearDelayQueueMessage> implements Settable<ClearDelayQueueMessage>, EpsilonComparable<ClearDelayQueueMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * If only a specific message type is to be cleared, use this field.
            * The integer represents the hash code of the message simple name.
            * The hash code of a string should be computed as in java.lang.String.hashCode().
            */
   public int class_simple_name_based_hash_code_;

   /**
            * If all the messages should be cleared, then this field should be set to true.
            */
   public boolean clear_all_delay_buffers_;

   public ClearDelayQueueMessage()
   {




   }

   public ClearDelayQueueMessage(ClearDelayQueueMessage other)
   {
      this();
      set(other);
   }

   public void set(ClearDelayQueueMessage other)
   {

      sequence_id_ = other.sequence_id_;


      class_simple_name_based_hash_code_ = other.class_simple_name_based_hash_code_;


      clear_all_delay_buffers_ = other.clear_all_delay_buffers_;

   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   /**
            * If only a specific message type is to be cleared, use this field.
            * The integer represents the hash code of the message simple name.
            * The hash code of a string should be computed as in java.lang.String.hashCode().
            */
   public void setClassSimpleNameBasedHashCode(int class_simple_name_based_hash_code)
   {
      class_simple_name_based_hash_code_ = class_simple_name_based_hash_code;
   }
   /**
            * If only a specific message type is to be cleared, use this field.
            * The integer represents the hash code of the message simple name.
            * The hash code of a string should be computed as in java.lang.String.hashCode().
            */
   public int getClassSimpleNameBasedHashCode()
   {
      return class_simple_name_based_hash_code_;
   }


   /**
            * If all the messages should be cleared, then this field should be set to true.
            */
   public void setClearAllDelayBuffers(boolean clear_all_delay_buffers)
   {
      clear_all_delay_buffers_ = clear_all_delay_buffers;
   }
   /**
            * If all the messages should be cleared, then this field should be set to true.
            */
   public boolean getClearAllDelayBuffers()
   {
      return clear_all_delay_buffers_;
   }


   public static Supplier<ClearDelayQueueMessagePubSubType> getPubSubType()
   {
      return ClearDelayQueueMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ClearDelayQueueMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ClearDelayQueueMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.class_simple_name_based_hash_code_, other.class_simple_name_based_hash_code_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.clear_all_delay_buffers_, other.clear_all_delay_buffers_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ClearDelayQueueMessage)) return false;

      ClearDelayQueueMessage otherMyClass = (ClearDelayQueueMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.class_simple_name_based_hash_code_ != otherMyClass.class_simple_name_based_hash_code_) return false;


      if(this.clear_all_delay_buffers_ != otherMyClass.clear_all_delay_buffers_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ClearDelayQueueMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("class_simple_name_based_hash_code=");
      builder.append(this.class_simple_name_based_hash_code_);      builder.append(", ");

      builder.append("clear_all_delay_buffers=");
      builder.append(this.clear_all_delay_buffers_);
      builder.append("}");
      return builder.toString();
   }
}
