package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RScanMessage extends Packet<AI2RScanMessage> implements Settable<AI2RScanMessage>, EpsilonComparable<AI2RScanMessage>
{
   /**
            * Target reference frame (object) to look for
            */
   public java.lang.StringBuilder target_object_;

   public AI2RScanMessage()
   {
      target_object_ = new java.lang.StringBuilder(255);
   }

   public AI2RScanMessage(AI2RScanMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RScanMessage other)
   {
      target_object_.setLength(0);
      target_object_.append(other.target_object_);
   }

   /**
            * Target reference frame (object) to look for
            */
   public void setTargetObject(java.lang.String target_object)
   {
      target_object_.setLength(0);
      target_object_.append(target_object);
   }

   /**
            * Target reference frame (object) to look for
            */
   public java.lang.String getTargetObjectAsString()
   {
      return getTargetObject().toString();
   }
   /**
            * Target reference frame (object) to look for
            */
   public java.lang.StringBuilder getTargetObject()
   {
      return target_object_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.target_object_, other.target_object_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RScanMessage)) return false;

      AI2RScanMessage otherMyClass = (AI2RScanMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.target_object_, otherMyClass.target_object_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RScanMessage {");
      builder.append("target_object=");
      builder.append(this.target_object_);
      builder.append("}");
      return builder.toString();
   }
}
