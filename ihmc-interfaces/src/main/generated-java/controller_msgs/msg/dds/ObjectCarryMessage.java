package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ObjectCarryMessage extends Packet<ObjectCarryMessage> implements Settable<ObjectCarryMessage>, EpsilonComparable<ObjectCarryMessage>
{
   public boolean pickup_object_;

   public ObjectCarryMessage()
   {
   }

   public ObjectCarryMessage(ObjectCarryMessage other)
   {
      this();
      set(other);
   }

   public void set(ObjectCarryMessage other)
   {
      pickup_object_ = other.pickup_object_;

   }

   public void setPickupObject(boolean pickup_object)
   {
      pickup_object_ = pickup_object;
   }
   public boolean getPickupObject()
   {
      return pickup_object_;
   }


   public static Supplier<ObjectCarryMessagePubSubType> getPubSubType()
   {
      return ObjectCarryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ObjectCarryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ObjectCarryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.pickup_object_, other.pickup_object_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ObjectCarryMessage)) return false;

      ObjectCarryMessage otherMyClass = (ObjectCarryMessage) other;

      if(this.pickup_object_ != otherMyClass.pickup_object_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ObjectCarryMessage {");
      builder.append("pickup_object=");
      builder.append(this.pickup_object_);
      builder.append("}");
      return builder.toString();
   }
}
