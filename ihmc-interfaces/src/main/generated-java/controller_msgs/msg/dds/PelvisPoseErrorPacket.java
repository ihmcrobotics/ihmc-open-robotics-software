package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Message part of the localization module
 */
public class PelvisPoseErrorPacket extends Packet<PelvisPoseErrorPacket> implements Settable<PelvisPoseErrorPacket>, EpsilonComparable<PelvisPoseErrorPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public float residual_error_;
   public float total_error_;
   public boolean has_map_been_reset_;

   public PelvisPoseErrorPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public PelvisPoseErrorPacket(PelvisPoseErrorPacket other)
   {
      this();
      set(other);
   }

   public void set(PelvisPoseErrorPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      residual_error_ = other.residual_error_;

      total_error_ = other.total_error_;

      has_map_been_reset_ = other.has_map_been_reset_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
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

   @Override
   public boolean epsilonEquals(PelvisPoseErrorPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.residual_error_, other.residual_error_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_error_, other.total_error_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.has_map_been_reset_, other.has_map_been_reset_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof PelvisPoseErrorPacket))
         return false;

      PelvisPoseErrorPacket otherMyClass = (PelvisPoseErrorPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.residual_error_ != otherMyClass.residual_error_)
         return false;

      if (this.total_error_ != otherMyClass.total_error_)
         return false;

      if (this.has_map_been_reset_ != otherMyClass.has_map_been_reset_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PelvisPoseErrorPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("residual_error=");
      builder.append(this.residual_error_);
      builder.append(", ");
      builder.append("total_error=");
      builder.append(this.total_error_);
      builder.append(", ");
      builder.append("has_map_been_reset=");
      builder.append(this.has_map_been_reset_);
      builder.append("}");
      return builder.toString();
   }
}
