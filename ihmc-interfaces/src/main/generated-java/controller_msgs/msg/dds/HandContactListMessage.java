package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandContactListMessage extends Packet<HandContactListMessage> implements Settable<HandContactListMessage>, EpsilonComparable<HandContactListMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Defines the list of footstep to perform.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HandContactMessage>  hand_contact_list_;

   public HandContactListMessage()
   {
      hand_contact_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HandContactMessage> (2, new controller_msgs.msg.dds.HandContactMessagePubSubType());

   }

   public HandContactListMessage(HandContactListMessage other)
   {
      this();
      set(other);
   }

   public void set(HandContactListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      hand_contact_list_.set(other.hand_contact_list_);
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
            * Defines the list of footstep to perform.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HandContactMessage>  getHandContactList()
   {
      return hand_contact_list_;
   }


   public static Supplier<HandContactListMessagePubSubType> getPubSubType()
   {
      return HandContactListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandContactListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandContactListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (this.hand_contact_list_.size() != other.hand_contact_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.hand_contact_list_.size(); i++)
         {  if (!this.hand_contact_list_.get(i).epsilonEquals(other.hand_contact_list_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandContactListMessage)) return false;

      HandContactListMessage otherMyClass = (HandContactListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.hand_contact_list_.equals(otherMyClass.hand_contact_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandContactListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("hand_contact_list=");
      builder.append(this.hand_contact_list_);
      builder.append("}");
      return builder.toString();
   }
}
