package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to notify the crutch display of the current slope state.
       */
public class QuixSlopeStateMessage extends Packet<QuixSlopeStateMessage> implements Settable<QuixSlopeStateMessage>, EpsilonComparable<QuixSlopeStateMessage>
{

   public static final byte STANDING = (byte) 0;

   public static final byte TO_WALKING_LEFT_SUPPORT = (byte) 1;

   public static final byte TO_WALKING_RIGHT_SUPPORT = (byte) 2;

   public static final byte WALKING_LEFT_SUPPORT = (byte) 3;

   public static final byte WALKING_RIGHT_SUPPORT = (byte) 4;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte slope_state_name_ = (byte) 255;

   public QuixSlopeStateMessage()
   {



   }

   public QuixSlopeStateMessage(QuixSlopeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixSlopeStateMessage other)
   {

      sequence_id_ = other.sequence_id_;


      slope_state_name_ = other.slope_state_name_;

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


   public void setSlopeStateName(byte slope_state_name)
   {
      slope_state_name_ = slope_state_name;
   }
   public byte getSlopeStateName()
   {
      return slope_state_name_;
   }


   public static Supplier<QuixSlopeStateMessagePubSubType> getPubSubType()
   {
      return QuixSlopeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixSlopeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixSlopeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.slope_state_name_, other.slope_state_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixSlopeStateMessage)) return false;

      QuixSlopeStateMessage otherMyClass = (QuixSlopeStateMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.slope_state_name_ != otherMyClass.slope_state_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixSlopeStateMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("slope_state_name=");
      builder.append(this.slope_state_name_);
      builder.append("}");
      return builder.toString();
   }
}
