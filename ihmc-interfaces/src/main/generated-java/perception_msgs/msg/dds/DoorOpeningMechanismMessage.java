package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DoorOpeningMechanismMessage extends Packet<DoorOpeningMechanismMessage> implements Settable<DoorOpeningMechanismMessage>, EpsilonComparable<DoorOpeningMechanismMessage>
{
   /**
            * The type of door opening mechanism (see DoorOpeningMechanism.DoorOpeningMechanismType)
            */
   public byte type_;
   /**
            * The side of the door the opening mechanism is mounted to
            * true  PUSH
            * false PULL
            */
   public boolean door_side_;
   /**
            * The transform of the opening mechanisms frame
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform mechanism_transform_to_world_;
   /**
            * ID of the persistent detection associated with this opening mechanism. May be null.
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage persistent_detection_id_;

   public DoorOpeningMechanismMessage()
   {
      mechanism_transform_to_world_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
      persistent_detection_id_ = new ihmc_common_msgs.msg.dds.UUIDMessage();
   }

   public DoorOpeningMechanismMessage(DoorOpeningMechanismMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorOpeningMechanismMessage other)
   {
      type_ = other.type_;

      door_side_ = other.door_side_;

      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.mechanism_transform_to_world_, mechanism_transform_to_world_);
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.staticCopy(other.persistent_detection_id_, persistent_detection_id_);
   }

   /**
            * The type of door opening mechanism (see DoorOpeningMechanism.DoorOpeningMechanismType)
            */
   public void setType(byte type)
   {
      type_ = type;
   }
   /**
            * The type of door opening mechanism (see DoorOpeningMechanism.DoorOpeningMechanismType)
            */
   public byte getType()
   {
      return type_;
   }

   /**
            * The side of the door the opening mechanism is mounted to
            * true  PUSH
            * false PULL
            */
   public void setDoorSide(boolean door_side)
   {
      door_side_ = door_side;
   }
   /**
            * The side of the door the opening mechanism is mounted to
            * true  PUSH
            * false PULL
            */
   public boolean getDoorSide()
   {
      return door_side_;
   }


   /**
            * The transform of the opening mechanisms frame
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform getMechanismTransformToWorld()
   {
      return mechanism_transform_to_world_;
   }


   /**
            * ID of the persistent detection associated with this opening mechanism. May be null.
            */
   public ihmc_common_msgs.msg.dds.UUIDMessage getPersistentDetectionId()
   {
      return persistent_detection_id_;
   }


   public static Supplier<DoorOpeningMechanismMessagePubSubType> getPubSubType()
   {
      return DoorOpeningMechanismMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorOpeningMechanismMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorOpeningMechanismMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.type_, other.type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.door_side_, other.door_side_, epsilon)) return false;

      if (!this.mechanism_transform_to_world_.epsilonEquals(other.mechanism_transform_to_world_, epsilon)) return false;
      if (!this.persistent_detection_id_.epsilonEquals(other.persistent_detection_id_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DoorOpeningMechanismMessage)) return false;

      DoorOpeningMechanismMessage otherMyClass = (DoorOpeningMechanismMessage) other;

      if(this.type_ != otherMyClass.type_) return false;

      if(this.door_side_ != otherMyClass.door_side_) return false;

      if (!this.mechanism_transform_to_world_.equals(otherMyClass.mechanism_transform_to_world_)) return false;
      if (!this.persistent_detection_id_.equals(otherMyClass.persistent_detection_id_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorOpeningMechanismMessage {");
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("door_side=");
      builder.append(this.door_side_);      builder.append(", ");
      builder.append("mechanism_transform_to_world=");
      builder.append(this.mechanism_transform_to_world_);      builder.append(", ");
      builder.append("persistent_detection_id=");
      builder.append(this.persistent_detection_id_);
      builder.append("}");
      return builder.toString();
   }
}
