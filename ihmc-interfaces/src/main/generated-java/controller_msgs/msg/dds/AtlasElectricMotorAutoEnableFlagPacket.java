package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Atlas specific message.
 */
public class AtlasElectricMotorAutoEnableFlagPacket extends Packet<AtlasElectricMotorAutoEnableFlagPacket>
      implements Settable<AtlasElectricMotorAutoEnableFlagPacket>, EpsilonComparable<AtlasElectricMotorAutoEnableFlagPacket>
{
   public boolean should_auto_enable_;

   public AtlasElectricMotorAutoEnableFlagPacket()
   {
   }

   public AtlasElectricMotorAutoEnableFlagPacket(AtlasElectricMotorAutoEnableFlagPacket other)
   {
      set(other);
   }

   public void set(AtlasElectricMotorAutoEnableFlagPacket other)
   {
      should_auto_enable_ = other.should_auto_enable_;
   }

   public boolean getShouldAutoEnable()
   {
      return should_auto_enable_;
   }

   public void setShouldAutoEnable(boolean should_auto_enable)
   {
      should_auto_enable_ = should_auto_enable;
   }

   @Override
   public boolean epsilonEquals(AtlasElectricMotorAutoEnableFlagPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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

      if (this.should_auto_enable_ != otherMyClass.should_auto_enable_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasElectricMotorAutoEnableFlagPacket {");
      builder.append("should_auto_enable=");
      builder.append(this.should_auto_enable_);

      builder.append("}");
      return builder.toString();
   }
}
