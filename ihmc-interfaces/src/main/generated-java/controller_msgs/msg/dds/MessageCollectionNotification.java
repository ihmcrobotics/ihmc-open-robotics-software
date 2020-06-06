package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Notifies the user that a MessageCollection has been received.
       * This is used to know when the controller is ready to start collecting the actual collection of messages.
       */
public class MessageCollectionNotification extends Packet<MessageCollectionNotification> implements Settable<MessageCollectionNotification>, EpsilonComparable<MessageCollectionNotification>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * This is the sequence ID of the MessageCollection that has just been received.
            */
   public long message_collection_sequence_id_;

   public MessageCollectionNotification()
   {



   }

   public MessageCollectionNotification(MessageCollectionNotification other)
   {
      this();
      set(other);
   }

   public void set(MessageCollectionNotification other)
   {

      sequence_id_ = other.sequence_id_;


      message_collection_sequence_id_ = other.message_collection_sequence_id_;

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
            * This is the sequence ID of the MessageCollection that has just been received.
            */
   public void setMessageCollectionSequenceId(long message_collection_sequence_id)
   {
      message_collection_sequence_id_ = message_collection_sequence_id;
   }
   /**
            * This is the sequence ID of the MessageCollection that has just been received.
            */
   public long getMessageCollectionSequenceId()
   {
      return message_collection_sequence_id_;
   }


   public static Supplier<MessageCollectionNotificationPubSubType> getPubSubType()
   {
      return MessageCollectionNotificationPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MessageCollectionNotificationPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MessageCollectionNotification other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.message_collection_sequence_id_, other.message_collection_sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MessageCollectionNotification)) return false;

      MessageCollectionNotification otherMyClass = (MessageCollectionNotification) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.message_collection_sequence_id_ != otherMyClass.message_collection_sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MessageCollectionNotification {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("message_collection_sequence_id=");
      builder.append(this.message_collection_sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
