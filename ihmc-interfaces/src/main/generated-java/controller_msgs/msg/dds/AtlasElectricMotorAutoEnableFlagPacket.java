package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Atlas specific message.
 */
public class AtlasElectricMotorAutoEnableFlagPacket extends Packet<AtlasElectricMotorAutoEnableFlagPacket>
      implements Settable<AtlasElectricMotorAutoEnableFlagPacket>, EpsilonComparable<AtlasElectricMotorAutoEnableFlagPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public boolean should_auto_enable_;

   public AtlasElectricMotorAutoEnableFlagPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public AtlasElectricMotorAutoEnableFlagPacket(AtlasElectricMotorAutoEnableFlagPacket other)
   {
      this();
      set(other);
   }

   public void set(AtlasElectricMotorAutoEnableFlagPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      should_auto_enable_ = other.should_auto_enable_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setShouldAutoEnable(boolean should_auto_enable)
   {
      should_auto_enable_ = should_auto_enable;
   }

   public boolean getShouldAutoEnable()
   {
      return should_auto_enable_;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorAutoEnableFlagPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.should_auto_enable_, other.should_auto_enable_, epsilon))
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
      if (!(other instanceof AtlasElectricMotorAutoEnableFlagPacket))
         return false;

      AtlasElectricMotorAutoEnableFlagPacket otherMyClass = (AtlasElectricMotorAutoEnableFlagPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.should_auto_enable_ != otherMyClass.should_auto_enable_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasElectricMotorAutoEnableFlagPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("should_auto_enable=");
      builder.append(this.should_auto_enable_);
      builder.append("}");
      return builder.toString();
   }
}
