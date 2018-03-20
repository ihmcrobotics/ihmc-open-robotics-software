package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class DrillDetectionPacket extends Packet<DrillDetectionPacket> implements Settable<DrillDetectionPacket>, EpsilonComparable<DrillDetectionPacket>
{
   public boolean is_drill_on_;

   public DrillDetectionPacket()
   {
   }

   public DrillDetectionPacket(DrillDetectionPacket other)
   {
      set(other);
   }

   public void set(DrillDetectionPacket other)
   {
      is_drill_on_ = other.is_drill_on_;
   }

   public boolean getIsDrillOn()
   {
      return is_drill_on_;
   }

   public void setIsDrillOn(boolean is_drill_on)
   {
      is_drill_on_ = is_drill_on;
   }

   @Override
   public boolean epsilonEquals(DrillDetectionPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_drill_on_, other.is_drill_on_, epsilon))
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
      if (!(other instanceof DrillDetectionPacket))
         return false;

      DrillDetectionPacket otherMyClass = (DrillDetectionPacket) other;

      if (this.is_drill_on_ != otherMyClass.is_drill_on_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DrillDetectionPacket {");
      builder.append("is_drill_on=");
      builder.append(this.is_drill_on_);

      builder.append("}");
      return builder.toString();
   }
}