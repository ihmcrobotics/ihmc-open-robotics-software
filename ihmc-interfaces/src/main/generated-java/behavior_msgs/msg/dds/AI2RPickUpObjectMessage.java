package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RPickUpObjectMessage extends Packet<AI2RPickUpObjectMessage> implements Settable<AI2RPickUpObjectMessage>, EpsilonComparable<AI2RPickUpObjectMessage>
{
   /**
            * Reference frame (object) to pick up
            */
   public java.lang.StringBuilder object_name_;

   public AI2RPickUpObjectMessage()
   {
      object_name_ = new java.lang.StringBuilder(255);
   }

   public AI2RPickUpObjectMessage(AI2RPickUpObjectMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RPickUpObjectMessage other)
   {
      object_name_.setLength(0);
      object_name_.append(other.object_name_);
   }

   /**
            * Reference frame (object) to pick up
            */
   public void setObjectName(java.lang.String object_name)
   {
      object_name_.setLength(0);
      object_name_.append(object_name);
   }

   /**
            * Reference frame (object) to pick up
            */
   public java.lang.String getObjectNameAsString()
   {
      return getObjectName().toString();
   }
   /**
            * Reference frame (object) to pick up
            */
   public java.lang.StringBuilder getObjectName()
   {
      return object_name_;
   }


   public static Supplier<AI2RPickUpObjectMessagePubSubType> getPubSubType()
   {
      return AI2RPickUpObjectMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RPickUpObjectMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RPickUpObjectMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_name_, other.object_name_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RPickUpObjectMessage)) return false;

      AI2RPickUpObjectMessage otherMyClass = (AI2RPickUpObjectMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.object_name_, otherMyClass.object_name_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RPickUpObjectMessage {");
      builder.append("object_name=");
      builder.append(this.object_name_);
      builder.append("}");
      return builder.toString();
   }
}
