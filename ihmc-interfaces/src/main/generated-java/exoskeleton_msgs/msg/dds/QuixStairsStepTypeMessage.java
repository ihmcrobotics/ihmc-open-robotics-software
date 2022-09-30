package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       */
public class QuixStairsStepTypeMessage extends Packet<QuixStairsStepTypeMessage> implements Settable<QuixStairsStepTypeMessage>, EpsilonComparable<QuixStairsStepTypeMessage>
{
   public static final byte FLAT = (byte) 0;
   public static final byte UP = (byte) 0;
   public static final byte DOWN = (byte) 0;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte stairs_step_type_name_ = (byte) 255;

   public QuixStairsStepTypeMessage()
   {
   }

   public QuixStairsStepTypeMessage(QuixStairsStepTypeMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixStairsStepTypeMessage other)
   {
      sequence_id_ = other.sequence_id_;

      stairs_step_type_name_ = other.stairs_step_type_name_;

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

   public void setStairsStepTypeName(byte stairs_step_type_name)
   {
      stairs_step_type_name_ = stairs_step_type_name;
   }
   public byte getStairsStepTypeName()
   {
      return stairs_step_type_name_;
   }


   public static Supplier<QuixStairsStepTypeMessagePubSubType> getPubSubType()
   {
      return QuixStairsStepTypeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixStairsStepTypeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixStairsStepTypeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stairs_step_type_name_, other.stairs_step_type_name_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixStairsStepTypeMessage)) return false;

      QuixStairsStepTypeMessage otherMyClass = (QuixStairsStepTypeMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.stairs_step_type_name_ != otherMyClass.stairs_step_type_name_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixStairsStepTypeMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("stairs_step_type_name=");
      builder.append(this.stairs_step_type_name_);
      builder.append("}");
      return builder.toString();
   }
}
