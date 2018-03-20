package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message part of the localization module
 */
public class LocalizationPacket extends Packet<LocalizationPacket> implements Settable<LocalizationPacket>, EpsilonComparable<LocalizationPacket>
{
   public boolean reset_;
   public boolean toggle_;

   public LocalizationPacket()
   {

   }

   public LocalizationPacket(LocalizationPacket other)
   {
      set(other);
   }

   public void set(LocalizationPacket other)
   {
      reset_ = other.reset_;

      toggle_ = other.toggle_;
   }

   public boolean getReset()
   {
      return reset_;
   }

   public void setReset(boolean reset)
   {
      reset_ = reset;
   }

   public boolean getToggle()
   {
      return toggle_;
   }

   public void setToggle(boolean toggle)
   {
      toggle_ = toggle;
   }

   @Override
   public boolean epsilonEquals(LocalizationPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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
      builder.append("reset=");
      builder.append(this.reset_);

      builder.append(", ");
      builder.append("toggle=");
      builder.append(this.toggle_);

      builder.append("}");
      return builder.toString();
   }
}