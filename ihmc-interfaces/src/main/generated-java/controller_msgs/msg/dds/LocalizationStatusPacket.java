package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Message part of the localization module
 */
public class LocalizationStatusPacket extends Packet<LocalizationStatusPacket>
      implements Settable<LocalizationStatusPacket>, EpsilonComparable<LocalizationStatusPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public double overlap_;
   public java.lang.StringBuilder status_;

   public LocalizationStatusPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      status_ = new java.lang.StringBuilder(255);
   }

   public LocalizationStatusPacket(LocalizationStatusPacket other)
   {
      this();
      set(other);
   }

   public void set(LocalizationStatusPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      overlap_ = other.overlap_;

      status_.setLength(0);
      status_.append(other.status_);

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setOverlap(double overlap)
   {
      overlap_ = overlap;
   }

   public double getOverlap()
   {
      return overlap_;
   }

   public void setStatus(java.lang.String status)
   {
      status_.setLength(0);
      status_.append(status);
   }

   public java.lang.String getStatusAsString()
   {
      return getStatus().toString();
   }

   public java.lang.StringBuilder getStatus()
   {
      return status_;
   }

   @Override
   public boolean epsilonEquals(LocalizationStatusPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.overlap_, other.overlap_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.status_, other.status_, epsilon))
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
      if (!(other instanceof LocalizationStatusPacket))
         return false;

      LocalizationStatusPacket otherMyClass = (LocalizationStatusPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.overlap_ != otherMyClass.overlap_)
         return false;

      if (!us.ihmc.idl.IDLTools.equals(this.status_, otherMyClass.status_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LocalizationStatusPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("overlap=");
      builder.append(this.overlap_);
      builder.append(", ");
      builder.append("status=");
      builder.append(this.status_);
      builder.append("}");
      return builder.toString();
   }
}
