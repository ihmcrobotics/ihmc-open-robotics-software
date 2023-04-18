package mission_control_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SystemRebootMessage extends Packet<SystemRebootMessage> implements Settable<SystemRebootMessage>, EpsilonComparable<SystemRebootMessage>
{
   public boolean unused_placeholder_field_;

   public SystemRebootMessage()
   {
   }

   public SystemRebootMessage(SystemRebootMessage other)
   {
      this();
      set(other);
   }

   public void set(SystemRebootMessage other)
   {
      unused_placeholder_field_ = other.unused_placeholder_field_;

   }

   public void setUnusedPlaceholderField(boolean unused_placeholder_field)
   {
      unused_placeholder_field_ = unused_placeholder_field;
   }
   public boolean getUnusedPlaceholderField()
   {
      return unused_placeholder_field_;
   }


   public static Supplier<SystemRebootMessagePubSubType> getPubSubType()
   {
      return SystemRebootMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SystemRebootMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SystemRebootMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.unused_placeholder_field_, other.unused_placeholder_field_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SystemRebootMessage)) return false;

      SystemRebootMessage otherMyClass = (SystemRebootMessage) other;

      if(this.unused_placeholder_field_ != otherMyClass.unused_placeholder_field_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SystemRebootMessage {");
      builder.append("unused_placeholder_field=");
      builder.append(this.unused_placeholder_field_);
      builder.append("}");
      return builder.toString();
   }
}
