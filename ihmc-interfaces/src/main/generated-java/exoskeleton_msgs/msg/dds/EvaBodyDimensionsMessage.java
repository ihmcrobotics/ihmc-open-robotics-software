package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used for syncing the body dimension config files in eva's resources with eva's local config files
       */
public class EvaBodyDimensionsMessage extends Packet<EvaBodyDimensionsMessage> implements Settable<EvaBodyDimensionsMessage>, EpsilonComparable<EvaBodyDimensionsMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.idl.IDLSequence.Byte  file_name_;
   /**
            * This is the wrench at the foot, with respect to the sole frame
            */
   public double thigh_length_;
   public double shank_length_;
   public long shoe_size_oridinal_;

   public EvaBodyDimensionsMessage()
   {
      file_name_ = new us.ihmc.idl.IDLSequence.Byte (2048, "type_9");

   }

   public EvaBodyDimensionsMessage(EvaBodyDimensionsMessage other)
   {
      this();
      set(other);
   }

   public void set(EvaBodyDimensionsMessage other)
   {
      sequence_id_ = other.sequence_id_;

      file_name_.set(other.file_name_);
      thigh_length_ = other.thigh_length_;

      shank_length_ = other.shank_length_;

      shoe_size_oridinal_ = other.shoe_size_oridinal_;

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


   public us.ihmc.idl.IDLSequence.Byte  getFileName()
   {
      return file_name_;
   }

   /**
            * This is the wrench at the foot, with respect to the sole frame
            */
   public void setThighLength(double thigh_length)
   {
      thigh_length_ = thigh_length;
   }
   /**
            * This is the wrench at the foot, with respect to the sole frame
            */
   public double getThighLength()
   {
      return thigh_length_;
   }

   public void setShankLength(double shank_length)
   {
      shank_length_ = shank_length;
   }
   public double getShankLength()
   {
      return shank_length_;
   }

   public void setShoeSizeOridinal(long shoe_size_oridinal)
   {
      shoe_size_oridinal_ = shoe_size_oridinal;
   }
   public long getShoeSizeOridinal()
   {
      return shoe_size_oridinal_;
   }


   public static Supplier<EvaBodyDimensionsMessagePubSubType> getPubSubType()
   {
      return EvaBodyDimensionsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EvaBodyDimensionsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EvaBodyDimensionsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.file_name_, other.file_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.thigh_length_, other.thigh_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shank_length_, other.shank_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shoe_size_oridinal_, other.shoe_size_oridinal_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EvaBodyDimensionsMessage)) return false;

      EvaBodyDimensionsMessage otherMyClass = (EvaBodyDimensionsMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.file_name_.equals(otherMyClass.file_name_)) return false;
      if(this.thigh_length_ != otherMyClass.thigh_length_) return false;

      if(this.shank_length_ != otherMyClass.shank_length_) return false;

      if(this.shoe_size_oridinal_ != otherMyClass.shoe_size_oridinal_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EvaBodyDimensionsMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("file_name=");
      builder.append(this.file_name_);      builder.append(", ");
      builder.append("thigh_length=");
      builder.append(this.thigh_length_);      builder.append(", ");
      builder.append("shank_length=");
      builder.append(this.shank_length_);      builder.append(", ");
      builder.append("shoe_size_oridinal=");
      builder.append(this.shoe_size_oridinal_);
      builder.append("}");
      return builder.toString();
   }
}
