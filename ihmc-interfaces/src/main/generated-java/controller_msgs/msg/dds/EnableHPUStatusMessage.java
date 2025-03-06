package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class EnableHPUStatusMessage extends Packet<EnableHPUStatusMessage> implements Settable<EnableHPUStatusMessage>, EpsilonComparable<EnableHPUStatusMessage>
{
   /**
            * True if hydaulic power unit (HPU) is enabled and false if it is turned off
            */
   public boolean hpu_enabled_;

   public EnableHPUStatusMessage()
   {
   }

   public EnableHPUStatusMessage(EnableHPUStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(EnableHPUStatusMessage other)
   {
      hpu_enabled_ = other.hpu_enabled_;

   }

   /**
            * True if hydaulic power unit (HPU) is enabled and false if it is turned off
            */
   public void setHpuEnabled(boolean hpu_enabled)
   {
      hpu_enabled_ = hpu_enabled;
   }
   /**
            * True if hydaulic power unit (HPU) is enabled and false if it is turned off
            */
   public boolean getHpuEnabled()
   {
      return hpu_enabled_;
   }


   public static Supplier<EnableHPUStatusMessagePubSubType> getPubSubType()
   {
      return EnableHPUStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EnableHPUStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EnableHPUStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hpu_enabled_, other.hpu_enabled_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EnableHPUStatusMessage)) return false;

      EnableHPUStatusMessage otherMyClass = (EnableHPUStatusMessage) other;

      if(this.hpu_enabled_ != otherMyClass.hpu_enabled_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EnableHPUStatusMessage {");
      builder.append("hpu_enabled=");
      builder.append(this.hpu_enabled_);
      builder.append("}");
      return builder.toString();
   }
}
