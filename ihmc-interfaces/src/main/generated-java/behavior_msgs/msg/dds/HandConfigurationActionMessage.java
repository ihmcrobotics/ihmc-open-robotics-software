package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandConfigurationActionMessage extends Packet<HandConfigurationActionMessage> implements Settable<HandConfigurationActionMessage>, EpsilonComparable<HandConfigurationActionMessage>
{
   public boolean unused_placeholder_field_;

   public HandConfigurationActionMessage()
   {
   }

   public HandConfigurationActionMessage(HandConfigurationActionMessage other)
   {
      this();
      set(other);
   }

   public void set(HandConfigurationActionMessage other)
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


   public static Supplier<HandConfigurationActionMessagePubSubType> getPubSubType()
   {
      return HandConfigurationActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandConfigurationActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandConfigurationActionMessage other, double epsilon)
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
      if(!(other instanceof HandConfigurationActionMessage)) return false;

      HandConfigurationActionMessage otherMyClass = (HandConfigurationActionMessage) other;

      if(this.unused_placeholder_field_ != otherMyClass.unused_placeholder_field_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandConfigurationActionMessage {");
      builder.append("unused_placeholder_field=");
      builder.append(this.unused_placeholder_field_);
      builder.append("}");
      return builder.toString();
   }
}
