package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message part of the localization module
 */
public class LocalizationStatusPacket implements Settable<LocalizationStatusPacket>, EpsilonComparable<LocalizationStatusPacket>
{
   private double overlap_;
   private java.lang.StringBuilder status_;

   public LocalizationStatusPacket()
   {

      status_ = new java.lang.StringBuilder(255);
   }

   public LocalizationStatusPacket(LocalizationStatusPacket other)
   {
      set(other);
   }

   public void set(LocalizationStatusPacket other)
   {
      overlap_ = other.overlap_;

      status_.setLength(0);
      status_.append(other.status_);
   }

   public double getOverlap()
   {
      return overlap_;
   }

   public void setOverlap(double overlap)
   {
      overlap_ = overlap;
   }

   public java.lang.String getStatusAsString()
   {
      return getStatus().toString();
   }

   public java.lang.StringBuilder getStatus()
   {
      return status_;
   }

   public void setStatus(String status)
   {
      status_.setLength(0);
      status_.append(status);
   }

   @Override
   public boolean epsilonEquals(LocalizationStatusPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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
      builder.append("overlap=");
      builder.append(this.overlap_);

      builder.append(", ");
      builder.append("status=");
      builder.append(this.status_);

      builder.append("}");
      return builder.toString();
   }
}