package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to notify the crutch display of the current sit-down state.
       */
public class QuixSitDownStateMessage extends Packet<QuixSitDownStateMessage> implements Settable<QuixSitDownStateMessage>, EpsilonComparable<QuixSitDownStateMessage>
{

   public static final byte WAITING = (byte) 0;

   public static final byte SIT_DOWN = (byte) 1;

   public static final byte WAITING_FOR_SEATED = (byte) 2;

   public static final byte DONE = (byte) 3;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte sit_down_state_name_ = (byte) 255;

   public QuixSitDownStateMessage()
   {



   }

   public QuixSitDownStateMessage(QuixSitDownStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixSitDownStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      sit_down_state_name_ = other.sit_down_state_name_;

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


   public void setSitDownStateName(byte sit_down_state_name)
   {
      sit_down_state_name_ = sit_down_state_name;
   }
   public byte getSitDownStateName()
   {
      return sit_down_state_name_;
   }


   public static Supplier<QuixSitDownStateMessagePubSubType> getPubSubType()
   {
      return QuixSitDownStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixSitDownStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixSitDownStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sit_down_state_name_, other.sit_down_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixSitDownStateMessage)) return false;

      QuixSitDownStateMessage otherMyClass = (QuixSitDownStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.sit_down_state_name_ != otherMyClass.sit_down_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixSitDownStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("sit_down_state_name=");
      builder.append(this.sit_down_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
