package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RCommandMessage extends Packet<AI2RCommandMessage> implements Settable<AI2RCommandMessage>, EpsilonComparable<AI2RCommandMessage>
{
   public boolean unused_placeholder_field_;

   public AI2RCommandMessage()
   {
   }

   public AI2RCommandMessage(AI2RCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RCommandMessage other)
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


   public static Supplier<AI2RCommandMessagePubSubType> getPubSubType()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RCommandMessage other, double epsilon)
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
      if(!(other instanceof AI2RCommandMessage)) return false;

      AI2RCommandMessage otherMyClass = (AI2RCommandMessage) other;

      if(this.unused_placeholder_field_ != otherMyClass.unused_placeholder_field_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RCommandMessage {");
      builder.append("unused_placeholder_field=");
      builder.append(this.unused_placeholder_field_);
      builder.append("}");
      return builder.toString();
   }
}
