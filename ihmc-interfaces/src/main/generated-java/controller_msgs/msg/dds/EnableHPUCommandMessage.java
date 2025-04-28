package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class EnableHPUCommandMessage extends Packet<EnableHPUCommandMessage> implements Settable<EnableHPUCommandMessage>, EpsilonComparable<EnableHPUCommandMessage>
{
   /**
            * True to enable hydaulic power unit (HPU) and false to turn HPU off immediately (not graceful)
            */
   public boolean enable_hpu_;

   public EnableHPUCommandMessage()
   {
   }

   public EnableHPUCommandMessage(EnableHPUCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(EnableHPUCommandMessage other)
   {
      enable_hpu_ = other.enable_hpu_;

   }

   /**
            * True to enable hydaulic power unit (HPU) and false to turn HPU off immediately (not graceful)
            */
   public void setEnableHpu(boolean enable_hpu)
   {
      enable_hpu_ = enable_hpu;
   }
   /**
            * True to enable hydaulic power unit (HPU) and false to turn HPU off immediately (not graceful)
            */
   public boolean getEnableHpu()
   {
      return enable_hpu_;
   }


   public static Supplier<EnableHPUCommandMessagePubSubType> getPubSubType()
   {
      return EnableHPUCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EnableHPUCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EnableHPUCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_hpu_, other.enable_hpu_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EnableHPUCommandMessage)) return false;

      EnableHPUCommandMessage otherMyClass = (EnableHPUCommandMessage) other;

      if(this.enable_hpu_ != otherMyClass.enable_hpu_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EnableHPUCommandMessage {");
      builder.append("enable_hpu=");
      builder.append(this.enable_hpu_);
      builder.append("}");
      return builder.toString();
   }
}
