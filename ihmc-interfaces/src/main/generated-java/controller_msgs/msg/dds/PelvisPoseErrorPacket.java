package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message part of the localization module
       */
public class PelvisPoseErrorPacket extends Packet<PelvisPoseErrorPacket> implements Settable<PelvisPoseErrorPacket>, EpsilonComparable<PelvisPoseErrorPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public float residual_error_;

   public float total_error_;

   public boolean has_map_been_reset_;

   public PelvisPoseErrorPacket()
   {





   }

   public PelvisPoseErrorPacket(PelvisPoseErrorPacket other)
   {
      this();
      set(other);
   }

   public void set(PelvisPoseErrorPacket other)
   {

      sequence_id_ = other.sequence_id_;


      residual_error_ = other.residual_error_;


      total_error_ = other.total_error_;


      has_map_been_reset_ = other.has_map_been_reset_;

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


   public void setResidualError(float residual_error)
   {
      residual_error_ = residual_error;
   }
   public float getResidualError()
   {
      return residual_error_;
   }


   public void setTotalError(float total_error)
   {
      total_error_ = total_error;
   }
   public float getTotalError()
   {
      return total_error_;
   }


   public void setHasMapBeenReset(boolean has_map_been_reset)
   {
      has_map_been_reset_ = has_map_been_reset;
   }
   public boolean getHasMapBeenReset()
   {
      return has_map_been_reset_;
   }


   public static Supplier<PelvisPoseErrorPacketPubSubType> getPubSubType()
   {
      return PelvisPoseErrorPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PelvisPoseErrorPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PelvisPoseErrorPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.residual_error_, other.residual_error_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_error_, other.total_error_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_map_been_reset_, other.has_map_been_reset_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PelvisPoseErrorPacket)) return false;

      PelvisPoseErrorPacket otherMyClass = (PelvisPoseErrorPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.residual_error_ != otherMyClass.residual_error_) return false;


      if(this.total_error_ != otherMyClass.total_error_) return false;


      if(this.has_map_been_reset_ != otherMyClass.has_map_been_reset_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PelvisPoseErrorPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("residual_error=");
      builder.append(this.residual_error_);      builder.append(", ");

      builder.append("total_error=");
      builder.append(this.total_error_);      builder.append(", ");

      builder.append("has_map_been_reset=");
      builder.append(this.has_map_been_reset_);
      builder.append("}");
      return builder.toString();
   }
}
