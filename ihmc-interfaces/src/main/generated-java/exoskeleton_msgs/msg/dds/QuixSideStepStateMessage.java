package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to notify the crutch display of the current side-step walking state.
       */
public class QuixSideStepStateMessage extends Packet<QuixSideStepStateMessage> implements Settable<QuixSideStepStateMessage>, EpsilonComparable<QuixSideStepStateMessage>
{
   public static final byte STANDING = (byte) 0;
   public static final byte TO_SWING_OUT_LEFT_SUPPORT = (byte) 1;
   public static final byte TO_SWING_OUT_RIGHT_SUPPORT = (byte) 2;
   public static final byte SWING_OUT_LEFT_SUPPORT = (byte) 3;
   public static final byte SWING_OUT_RIGHT_SUPPORT = (byte) 4;
   public static final byte TO_SWING_CLOSE_LEFT_SUPPORT = (byte) 5;
   public static final byte TO_SWING_CLOSE_RIGHT_SUPPORT = (byte) 6;
   public static final byte SWING_CLOSE_LEFT_SUPPORT = (byte) 7;
   public static final byte SWING_CLOSE_RIGHT_SUPPORT = (byte) 8;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies which state the controller should transition into.
            */
   public byte side_step_state_name_ = (byte) 255;

   public QuixSideStepStateMessage()
   {
   }

   public QuixSideStepStateMessage(QuixSideStepStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixSideStepStateMessage other)
   {
      sequence_id_ = other.sequence_id_;

      side_step_state_name_ = other.side_step_state_name_;

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
   public void setSideStepStateName(byte side_step_state_name)
   {
      side_step_state_name_ = side_step_state_name;
   }
   /**
            * Specifies which state the controller should transition into.
            */
   public byte getSideStepStateName()
   {
      return side_step_state_name_;
   }


   public static Supplier<QuixSideStepStateMessagePubSubType> getPubSubType()
   {
      return QuixSideStepStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixSideStepStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixSideStepStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.side_step_state_name_, other.side_step_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixSideStepStateMessage)) return false;

      QuixSideStepStateMessage otherMyClass = (QuixSideStepStateMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.side_step_state_name_ != otherMyClass.side_step_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixSideStepStateMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("side_step_state_name=");
      builder.append(this.side_step_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
