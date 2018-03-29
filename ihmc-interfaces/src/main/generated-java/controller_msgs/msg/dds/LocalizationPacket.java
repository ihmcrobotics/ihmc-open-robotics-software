package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Message part of the localization module
 */
public class LocalizationPacket extends Packet<LocalizationPacket> implements Settable<LocalizationPacket>, EpsilonComparable<LocalizationPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public boolean reset_;
   public boolean toggle_;

   public LocalizationPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public LocalizationPacket(LocalizationPacket other)
   {
      this();
      set(other);
   }

   public void set(LocalizationPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      reset_ = other.reset_;

      toggle_ = other.toggle_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setReset(boolean reset)
   {
      reset_ = reset;
   }

   public boolean getReset()
   {
      return reset_;
   }

   public void setToggle(boolean toggle)
   {
      toggle_ = toggle;
   }

   public boolean getToggle()
   {
      return toggle_;
   }

   @Override
   public boolean epsilonEquals(LocalizationPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.reset_, other.reset_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.toggle_, other.toggle_, epsilon))
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
      if (!(other instanceof LocalizationPacket))
         return false;

      LocalizationPacket otherMyClass = (LocalizationPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.reset_ != otherMyClass.reset_)
         return false;

      if (this.toggle_ != otherMyClass.toggle_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LocalizationPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("reset=");
      builder.append(this.reset_);
      builder.append(", ");
      builder.append("toggle=");
      builder.append(this.toggle_);
      builder.append("}");
      return builder.toString();
   }
}
