package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to notify the crutch display of the current flat-ground walking state.
       */
public class QuixFlatGroundStateMessage extends Packet<QuixFlatGroundStateMessage> implements Settable<QuixFlatGroundStateMessage>, EpsilonComparable<QuixFlatGroundStateMessage>
{

   public static final byte STANDING = (byte) 0;

   public static final byte TO_WALKING_LEFT_SUPPORT = (byte) 1;

   public static final byte TO_WALKING_RIGHT_SUPPORT = (byte) 2;

   public static final byte TOE_OFF_TO_LEFT_SUPPORT = (byte) 3;

   public static final byte TOE_OFF_TO_RIGHT_SUPPORT = (byte) 4;

   public static final byte WALKING_LEFT_SUPPORT = (byte) 5;

   public static final byte WALKING_RIGHT_SUPPORT = (byte) 6;

   public static final byte TRANSFER_TO_STANDING = (byte) 7;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies which state the controller should transition into.
            */
   public byte flat_ground_state_name_ = (byte) 255;

   public QuixFlatGroundStateMessage()
   {



   }

   public QuixFlatGroundStateMessage(QuixFlatGroundStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixFlatGroundStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      flat_ground_state_name_ = other.flat_ground_state_name_;

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
            * Specifies which state the controller should transition into.
            */
   public void setFlatGroundStateName(byte flat_ground_state_name)
   {
      flat_ground_state_name_ = flat_ground_state_name;
   }
   /**
            * Specifies which state the controller should transition into.
            */
   public byte getFlatGroundStateName()
   {
      return flat_ground_state_name_;
   }


   public static Supplier<QuixFlatGroundStateMessagePubSubType> getPubSubType()
   {
      return QuixFlatGroundStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixFlatGroundStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixFlatGroundStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flat_ground_state_name_, other.flat_ground_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixFlatGroundStateMessage)) return false;

      QuixFlatGroundStateMessage otherMyClass = (QuixFlatGroundStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.flat_ground_state_name_ != otherMyClass.flat_ground_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixFlatGroundStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("flat_ground_state_name=");
      builder.append(this.flat_ground_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
