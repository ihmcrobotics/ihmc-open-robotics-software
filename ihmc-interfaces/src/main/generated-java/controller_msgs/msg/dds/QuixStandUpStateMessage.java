package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to notify the crutch display of the current stand-up state.
       */
public class QuixStandUpStateMessage extends Packet<QuixStandUpStateMessage> implements Settable<QuixStandUpStateMessage>, EpsilonComparable<QuixStandUpStateMessage>
{

   public static final byte WAITING = (byte) 0;

   public static final byte MOVE_TO_INITIAL_POSITION_FOR_STAND_UP = (byte) 1;

   public static final byte WAIT_TO_STAND_UP = (byte) 2;

   public static final byte STAND_UP = (byte) 3;

   public static final byte DONE = (byte) 4;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte stand_up_state_name_ = (byte) 255;

   public QuixStandUpStateMessage()
   {



   }

   public QuixStandUpStateMessage(QuixStandUpStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixStandUpStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      stand_up_state_name_ = other.stand_up_state_name_;

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


   public void setStandUpStateName(byte stand_up_state_name)
   {
      stand_up_state_name_ = stand_up_state_name;
   }
   public byte getStandUpStateName()
   {
      return stand_up_state_name_;
   }


   public static Supplier<QuixStandUpStateMessagePubSubType> getPubSubType()
   {
      return QuixStandUpStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixStandUpStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixStandUpStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stand_up_state_name_, other.stand_up_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixStandUpStateMessage)) return false;

      QuixStandUpStateMessage otherMyClass = (QuixStandUpStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.stand_up_state_name_ != otherMyClass.stand_up_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixStandUpStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("stand_up_state_name=");
      builder.append(this.stand_up_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
