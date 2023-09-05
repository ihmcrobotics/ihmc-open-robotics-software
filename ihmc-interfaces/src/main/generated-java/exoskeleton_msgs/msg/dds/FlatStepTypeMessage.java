package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       */
public class FlatStepTypeMessage extends Packet<FlatStepTypeMessage> implements Settable<FlatStepTypeMessage>, EpsilonComparable<FlatStepTypeMessage>
{
   public static final byte SQUARE = (byte) 0;
   public static final byte SHORT_FORWARD = (byte) 1;
   public static final byte MEDIUM_FORWARD = (byte) 2;
   public static final byte LONG_FORWARD = (byte) 3;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte flat_step_type_name_ = (byte) 255;

   public FlatStepTypeMessage()
   {
   }

   public FlatStepTypeMessage(FlatStepTypeMessage other)
   {
      this();
      set(other);
   }

   public void set(FlatStepTypeMessage other)
   {
      sequence_id_ = other.sequence_id_;

      flat_step_type_name_ = other.flat_step_type_name_;

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

   public void setFlatStepTypeName(byte flat_step_type_name)
   {
      flat_step_type_name_ = flat_step_type_name;
   }
   public byte getFlatStepTypeName()
   {
      return flat_step_type_name_;
   }


   public static Supplier<FlatStepTypeMessagePubSubType> getPubSubType()
   {
      return FlatStepTypeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FlatStepTypeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FlatStepTypeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.flat_step_type_name_, other.flat_step_type_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FlatStepTypeMessage)) return false;

      FlatStepTypeMessage otherMyClass = (FlatStepTypeMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.flat_step_type_name_ != otherMyClass.flat_step_type_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FlatStepTypeMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("flat_step_type_name=");
      builder.append(this.flat_step_type_name_);
      builder.append("}");
      return builder.toString();
   }
}
