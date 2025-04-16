package unitree_go_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class InterfaceConfig extends Packet<InterfaceConfig> implements Settable<InterfaceConfig>, EpsilonComparable<InterfaceConfig>
{
   public byte mode_;
   public byte value_;
   public byte[] reserve_;

   public InterfaceConfig()
   {
      reserve_ = new byte[2];

   }

   public InterfaceConfig(InterfaceConfig other)
   {
      this();
      set(other);
   }

   public void set(InterfaceConfig other)
   {
      mode_ = other.mode_;

      value_ = other.value_;

      for(int i1 = 0; i1 < reserve_.length; ++i1)
      {
            reserve_[i1] = other.reserve_[i1];

      }

   }

   public void setMode(byte mode)
   {
      mode_ = mode;
   }
   public byte getMode()
   {
      return mode_;
   }

   public void setValue(byte value)
   {
      value_ = value;
   }
   public byte getValue()
   {
      return value_;
   }


   public byte[] getReserve()
   {
      return reserve_;
   }


   public static Supplier<InterfaceConfigPubSubType> getPubSubType()
   {
      return InterfaceConfigPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return InterfaceConfigPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(InterfaceConfig other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mode_, other.mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_, other.value_, epsilon)) return false;

      for(int i3 = 0; i3 < reserve_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.reserve_[i3], other.reserve_[i3], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof InterfaceConfig)) return false;

      InterfaceConfig otherMyClass = (InterfaceConfig) other;

      if(this.mode_ != otherMyClass.mode_) return false;

      if(this.value_ != otherMyClass.value_) return false;

      for(int i5 = 0; i5 < reserve_.length; ++i5)
      {
                if(this.reserve_[i5] != otherMyClass.reserve_[i5]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("InterfaceConfig {");
      builder.append("mode=");
      builder.append(this.mode_);      builder.append(", ");
      builder.append("value=");
      builder.append(this.value_);      builder.append(", ");
      builder.append("reserve=");
      builder.append(java.util.Arrays.toString(this.reserve_));
      builder.append("}");
      return builder.toString();
   }
}
