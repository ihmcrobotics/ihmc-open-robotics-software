package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class DrillDetectionPacket extends Packet<DrillDetectionPacket> implements Settable<DrillDetectionPacket>, EpsilonComparable<DrillDetectionPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public boolean is_drill_on_;

   public DrillDetectionPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public DrillDetectionPacket(DrillDetectionPacket other)
   {
      this();
      set(other);
   }

   public void set(DrillDetectionPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      is_drill_on_ = other.is_drill_on_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setIsDrillOn(boolean is_drill_on)
   {
      is_drill_on_ = is_drill_on;
   }

   public boolean getIsDrillOn()
   {
      return is_drill_on_;
   }

   @Override
   public boolean epsilonEquals(DrillDetectionPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.is_drill_on_ != otherMyClass.is_drill_on_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DrillDetectionPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("is_drill_on=");
      builder.append(this.is_drill_on_);
      builder.append("}");
      return builder.toString();
   }
}
