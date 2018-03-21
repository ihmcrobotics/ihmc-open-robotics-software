package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class UIConnectedPacket extends Packet<UIConnectedPacket> implements Settable<UIConnectedPacket>, EpsilonComparable<UIConnectedPacket>
{
   public boolean unused_placeholder_field_;

   public UIConnectedPacket()
   {
   }

   public UIConnectedPacket(UIConnectedPacket other)
   {
      set(other);
   }

   public void set(UIConnectedPacket other)
   {
      unused_placeholder_field_ = other.unused_placeholder_field_;
   }

   public boolean getUnusedPlaceholderField()
   {
      return unused_placeholder_field_;
   }

   public void setUnusedPlaceholderField(boolean unused_placeholder_field)
   {
      unused_placeholder_field_ = unused_placeholder_field;
   }

   @Override
   public boolean epsilonEquals(UIConnectedPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.unused_placeholder_field_, other.unused_placeholder_field_, epsilon))
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
      if (!(other instanceof UIConnectedPacket))
         return false;

      UIConnectedPacket otherMyClass = (UIConnectedPacket) other;

      if (this.unused_placeholder_field_ != otherMyClass.unused_placeholder_field_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UIConnectedPacket {");
      builder.append("unused_placeholder_field=");
      builder.append(this.unused_placeholder_field_);

      builder.append("}");
      return builder.toString();
   }
}