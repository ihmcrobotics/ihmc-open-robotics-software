package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class UwbSwitch extends Packet<UwbSwitch> implements Settable<UwbSwitch>, EpsilonComparable<UwbSwitch>
{
   public byte enabled_;

   public UwbSwitch()
   {
   }

   public UwbSwitch(UwbSwitch other)
   {
      this();
      set(other);
   }

   public void set(UwbSwitch other)
   {
      enabled_ = other.enabled_;

   }

   public void setEnabled(byte enabled)
   {
      enabled_ = enabled;
   }
   public byte getEnabled()
   {
      return enabled_;
   }


   public static Supplier<UwbSwitchPubSubType> getPubSubType()
   {
      return UwbSwitchPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return UwbSwitchPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(UwbSwitch other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.enabled_, other.enabled_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof UwbSwitch)) return false;

      UwbSwitch otherMyClass = (UwbSwitch) other;

      if(this.enabled_ != otherMyClass.enabled_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("UwbSwitch {");
      builder.append("enabled=");
      builder.append(this.enabled_);
      builder.append("}");
      return builder.toString();
   }
}
