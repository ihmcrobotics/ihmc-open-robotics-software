package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RReceiveObjectMessage extends Packet<AI2RReceiveObjectMessage> implements Settable<AI2RReceiveObjectMessage>, EpsilonComparable<AI2RReceiveObjectMessage>
{
   /**
            * Reference frame (object) to receive
            */
   public java.lang.StringBuilder object_name_;
   /**
            * Which hand to receive the object
            */
   public byte side_ = (byte) 255;
   /**
            * The maximum distance hand-object allowed to trigger grasping
            */
   public double distance_to_object_;

   public AI2RReceiveObjectMessage()
   {
      object_name_ = new java.lang.StringBuilder(255);
   }

   public AI2RReceiveObjectMessage(AI2RReceiveObjectMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RReceiveObjectMessage other)
   {
      object_name_.setLength(0);
      object_name_.append(other.object_name_);

      side_ = other.side_;

      distance_to_object_ = other.distance_to_object_;

   }

   /**
            * Reference frame (object) to receive
            */
   public void setObjectName(java.lang.String object_name)
   {
      object_name_.setLength(0);
      object_name_.append(object_name);
   }

   /**
            * Reference frame (object) to receive
            */
   public java.lang.String getObjectNameAsString()
   {
      return getObjectName().toString();
   }
   /**
            * Reference frame (object) to receive
            */
   public java.lang.StringBuilder getObjectName()
   {
      return object_name_;
   }

   /**
            * Which hand to receive the object
            */
   public void setSide(byte side)
   {
      side_ = side;
   }
   /**
            * Which hand to receive the object
            */
   public byte getSide()
   {
      return side_;
   }

   /**
            * The maximum distance hand-object allowed to trigger grasping
            */
   public void setDistanceToObject(double distance_to_object)
   {
      distance_to_object_ = distance_to_object;
   }
   /**
            * The maximum distance hand-object allowed to trigger grasping
            */
   public double getDistanceToObject()
   {
      return distance_to_object_;
   }


   public static Supplier<AI2RReceiveObjectMessagePubSubType> getPubSubType()
   {
      return AI2RReceiveObjectMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RReceiveObjectMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RReceiveObjectMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_name_, other.object_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.side_, other.side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_to_object_, other.distance_to_object_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RReceiveObjectMessage)) return false;

      AI2RReceiveObjectMessage otherMyClass = (AI2RReceiveObjectMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.object_name_, otherMyClass.object_name_)) return false;

      if(this.side_ != otherMyClass.side_) return false;

      if(this.distance_to_object_ != otherMyClass.distance_to_object_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RReceiveObjectMessage {");
      builder.append("object_name=");
      builder.append(this.object_name_);      builder.append(", ");
      builder.append("side=");
      builder.append(this.side_);      builder.append(", ");
      builder.append("distance_to_object=");
      builder.append(this.distance_to_object_);
      builder.append("}");
      return builder.toString();
   }
}
