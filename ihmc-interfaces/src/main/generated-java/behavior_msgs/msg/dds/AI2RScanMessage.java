package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RScanMessage extends Packet<AI2RScanMessage> implements Settable<AI2RScanMessage>, EpsilonComparable<AI2RScanMessage>
{
   /**
            * Target reference frames (objects) to look for
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  target_objects_;

   public AI2RScanMessage()
   {
      target_objects_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");
   }

   public AI2RScanMessage(AI2RScanMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RScanMessage other)
   {
      target_objects_.set(other.target_objects_);
   }


   /**
            * Target reference frames (objects) to look for
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getTargetObjects()
   {
      return target_objects_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.target_objects_, other.target_objects_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RScanMessage)) return false;

      AI2RScanMessage otherMyClass = (AI2RScanMessage) other;

      if (!this.target_objects_.equals(otherMyClass.target_objects_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RScanMessage {");
      builder.append("target_objects=");
      builder.append(this.target_objects_);
      builder.append("}");
      return builder.toString();
   }
}
