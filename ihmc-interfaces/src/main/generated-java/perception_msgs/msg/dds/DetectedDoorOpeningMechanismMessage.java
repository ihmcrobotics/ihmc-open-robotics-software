package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message representing the DoorOpeningMechanism
       */
public class DetectedDoorOpeningMechanismMessage extends Packet<DetectedDoorOpeningMechanismMessage> implements Settable<DetectedDoorOpeningMechanismMessage>, EpsilonComparable<DetectedDoorOpeningMechanismMessage>
{
   public java.lang.StringBuilder name_;
   /**
            * Ordinal of DoorSide
            */
   public byte side_;
   /**
            * Pose of the opening mechanism.
            * May contain NaN in the position and/or orientation values if unknown.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;

   public DetectedDoorOpeningMechanismMessage()
   {
      name_ = new java.lang.StringBuilder(255);
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public DetectedDoorOpeningMechanismMessage(DetectedDoorOpeningMechanismMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectedDoorOpeningMechanismMessage other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      side_ = other.side_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
   }

   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   /**
            * Ordinal of DoorSide
            */
   public void setSide(byte side)
   {
      side_ = side;
   }
   /**
            * Ordinal of DoorSide
            */
   public byte getSide()
   {
      return side_;
   }


   /**
            * Pose of the opening mechanism.
            * May contain NaN in the position and/or orientation values if unknown.
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
   }


   public static Supplier<DetectedDoorOpeningMechanismMessagePubSubType> getPubSubType()
   {
      return DetectedDoorOpeningMechanismMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectedDoorOpeningMechanismMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectedDoorOpeningMechanismMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.side_, other.side_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedDoorOpeningMechanismMessage)) return false;

      DetectedDoorOpeningMechanismMessage otherMyClass = (DetectedDoorOpeningMechanismMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if(this.side_ != otherMyClass.side_) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedDoorOpeningMechanismMessage {");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("side=");
      builder.append(this.side_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
