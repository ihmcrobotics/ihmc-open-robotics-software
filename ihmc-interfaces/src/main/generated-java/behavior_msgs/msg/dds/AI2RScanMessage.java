package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RScanMessage extends Packet<AI2RScanMessage> implements Settable<AI2RScanMessage>, EpsilonComparable<AI2RScanMessage>
{
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  object_names_;

   public AI2RScanMessage()
   {
      object_names_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (200, "type_d");
   }

   public AI2RScanMessage(AI2RScanMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RScanMessage other)
   {
      object_names_.set(other.object_names_);
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getObjectNames()
   {
      return object_names_;
   }


   public static Supplier<AI2RScanMessagePubSubType> getPubSubType()
   {
      return AI2RScanMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RScanMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RScanMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.object_names_, other.object_names_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RScanMessage)) return false;

      AI2RScanMessage otherMyClass = (AI2RScanMessage) other;

      if (!this.object_names_.equals(otherMyClass.object_names_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RScanMessage {");
      builder.append("object_names=");
      builder.append(this.object_names_);
      builder.append("}");
      return builder.toString();
   }
}
