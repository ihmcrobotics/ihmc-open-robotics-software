package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class DoorLocationPacket extends Packet<DoorLocationPacket> implements Settable<DoorLocationPacket>, EpsilonComparable<DoorLocationPacket>
{

   public static final byte UNKNOWN_TYPE = (byte) 0;

   public static final byte PUSH_HANDLE_LEFT = (byte) 1;

   public static final byte PUSH_HANDLE_RIGHT = (byte) 2;

   public static final byte PULL_HANDLE_LEFT = (byte) 3;

   public static final byte PULL_HANDLE_RIGHT = (byte) 4;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.euclid.geometry.Pose3D door_transform_to_world_;

   public byte detected_door_type_ = (byte) 255;

   /**
            * Some behaviors will average the door location to remove noise,
            * trustedPosition allows someone to tell the behavior that this is an accurate location and and can does not need to be averaged,
            * this is useful for sending door locations manually from the ui*/
   public boolean trusted_position_;

   public DoorLocationPacket()
   {


      door_transform_to_world_ = new us.ihmc.euclid.geometry.Pose3D();



   }

   public DoorLocationPacket(DoorLocationPacket other)
   {
      this();
      set(other);
   }

   public void set(DoorLocationPacket other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.door_transform_to_world_, door_transform_to_world_);

      detected_door_type_ = other.detected_door_type_;


      trusted_position_ = other.trusted_position_;

   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }



   public us.ihmc.euclid.geometry.Pose3D getDoorTransformToWorld()
   {
      return door_transform_to_world_;
   }


   public void setDetectedDoorType(byte detected_door_type)
   {
      detected_door_type_ = detected_door_type;
   }
   public byte getDetectedDoorType()
   {
      return detected_door_type_;
   }


   /**
            * Some behaviors will average the door location to remove noise,
            * trustedPosition allows someone to tell the behavior that this is an accurate location and and can does not need to be averaged,
            * this is useful for sending door locations manually from the ui*/
   public void setTrustedPosition(boolean trusted_position)
   {
      trusted_position_ = trusted_position;
   }
   /**
            * Some behaviors will average the door location to remove noise,
            * trustedPosition allows someone to tell the behavior that this is an accurate location and and can does not need to be averaged,
            * this is useful for sending door locations manually from the ui*/
   public boolean getTrustedPosition()
   {
      return trusted_position_;
   }


   public static Supplier<DoorLocationPacketPubSubType> getPubSubType()
   {
      return DoorLocationPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorLocationPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorLocationPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.door_transform_to_world_.epsilonEquals(other.door_transform_to_world_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.detected_door_type_, other.detected_door_type_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.trusted_position_, other.trusted_position_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DoorLocationPacket)) return false;

      DoorLocationPacket otherMyClass = (DoorLocationPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.door_transform_to_world_.equals(otherMyClass.door_transform_to_world_)) return false;

      if(this.detected_door_type_ != otherMyClass.detected_door_type_) return false;


      if(this.trusted_position_ != otherMyClass.trusted_position_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorLocationPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("door_transform_to_world=");
      builder.append(this.door_transform_to_world_);      builder.append(", ");

      builder.append("detected_door_type=");
      builder.append(this.detected_door_type_);      builder.append(", ");

      builder.append("trusted_position=");
      builder.append(this.trusted_position_);
      builder.append("}");
      return builder.toString();
   }
}
