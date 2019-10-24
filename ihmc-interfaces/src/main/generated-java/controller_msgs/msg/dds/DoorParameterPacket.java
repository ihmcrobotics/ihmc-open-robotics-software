package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC humanoid behavior module.
       */
public class DoorParameterPacket extends Packet<DoorParameterPacket> implements Settable<DoorParameterPacket>, EpsilonComparable<DoorParameterPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.euclid.tuple3D.Point3D hinged_point_on_ground_;
   public us.ihmc.euclid.tuple3D.Point3D end_point_on_ground_;
   public double door_height_;
   public us.ihmc.euclid.geometry.Pose3D door_handle_transform_to_world_;

   public DoorParameterPacket()
   {
      hinged_point_on_ground_ = new us.ihmc.euclid.tuple3D.Point3D();
      end_point_on_ground_ = new us.ihmc.euclid.tuple3D.Point3D();
      door_handle_transform_to_world_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public DoorParameterPacket(DoorParameterPacket other)
   {
      this();
      set(other);
   }

   public void set(DoorParameterPacket other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.hinged_point_on_ground_, hinged_point_on_ground_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.end_point_on_ground_, end_point_on_ground_);
      door_height_ = other.door_height_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.door_handle_transform_to_world_, door_handle_transform_to_world_);
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


   public us.ihmc.euclid.tuple3D.Point3D getHingedPointOnGround()
   {
      return hinged_point_on_ground_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getEndPointOnGround()
   {
      return end_point_on_ground_;
   }

   public void setDoorHeight(double door_height)
   {
      door_height_ = door_height;
   }
   public double getDoorHeight()
   {
      return door_height_;
   }


   public us.ihmc.euclid.geometry.Pose3D getDoorHandleTransformToWorld()
   {
      return door_handle_transform_to_world_;
   }


   public static Supplier<DoorParameterPacketPubSubType> getPubSubType()
   {
      return DoorParameterPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorParameterPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorParameterPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.hinged_point_on_ground_.epsilonEquals(other.hinged_point_on_ground_, epsilon)) return false;
      if (!this.end_point_on_ground_.epsilonEquals(other.end_point_on_ground_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.door_height_, other.door_height_, epsilon)) return false;

      if (!this.door_handle_transform_to_world_.epsilonEquals(other.door_handle_transform_to_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DoorParameterPacket)) return false;

      DoorParameterPacket otherMyClass = (DoorParameterPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.hinged_point_on_ground_.equals(otherMyClass.hinged_point_on_ground_)) return false;
      if (!this.end_point_on_ground_.equals(otherMyClass.end_point_on_ground_)) return false;
      if(this.door_height_ != otherMyClass.door_height_) return false;

      if (!this.door_handle_transform_to_world_.equals(otherMyClass.door_handle_transform_to_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorParameterPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("hinged_point_on_ground=");
      builder.append(this.hinged_point_on_ground_);      builder.append(", ");
      builder.append("end_point_on_ground=");
      builder.append(this.end_point_on_ground_);      builder.append(", ");
      builder.append("door_height=");
      builder.append(this.door_height_);      builder.append(", ");
      builder.append("door_handle_transform_to_world=");
      builder.append(this.door_handle_transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
