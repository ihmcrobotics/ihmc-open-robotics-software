package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPostProcessingPacket extends Packet<FootstepPostProcessingPacket> implements Settable<FootstepPostProcessingPacket>, EpsilonComparable<FootstepPostProcessingPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public us.ihmc.euclid.tuple3D.Point3D left_foot_position_in_world_;
   public us.ihmc.euclid.tuple3D.Point3D right_foot_position_in_world_;
   public us.ihmc.euclid.tuple4D.Quaternion left_foot_orientation_in_world_;
   public us.ihmc.euclid.tuple4D.Quaternion right_foot_orientation_in_world_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  left_foot_contact_points_2d_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  right_foot_contact_points_2d_;
   public controller_msgs.msg.dds.FootstepDataListMessage footstep_data_list_;
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_;

   public FootstepPostProcessingPacket()
   {
      left_foot_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      right_foot_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();
      left_foot_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      right_foot_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();
      left_foot_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());
      right_foot_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());
      footstep_data_list_ = new controller_msgs.msg.dds.FootstepDataListMessage();
      planar_regions_list_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();

   }

   public FootstepPostProcessingPacket(FootstepPostProcessingPacket other)
   {
      this();
      set(other);
   }

   public void set(FootstepPostProcessingPacket other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.left_foot_position_in_world_, left_foot_position_in_world_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.right_foot_position_in_world_, right_foot_position_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.left_foot_orientation_in_world_, left_foot_orientation_in_world_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.right_foot_orientation_in_world_, right_foot_orientation_in_world_);
      left_foot_contact_points_2d_.set(other.left_foot_contact_points_2d_);
      right_foot_contact_points_2d_.set(other.right_foot_contact_points_2d_);
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.footstep_data_list_, footstep_data_list_);
      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_, planar_regions_list_);
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


   public us.ihmc.euclid.tuple3D.Point3D getLeftFootPositionInWorld()
   {
      return left_foot_position_in_world_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getRightFootPositionInWorld()
   {
      return right_foot_position_in_world_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getLeftFootOrientationInWorld()
   {
      return left_foot_orientation_in_world_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getRightFootOrientationInWorld()
   {
      return right_foot_orientation_in_world_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getLeftFootContactPoints2d()
   {
      return left_foot_contact_points_2d_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getRightFootContactPoints2d()
   {
      return right_foot_contact_points_2d_;
   }


   public controller_msgs.msg.dds.FootstepDataListMessage getFootstepDataList()
   {
      return footstep_data_list_;
   }


   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsList()
   {
      return planar_regions_list_;
   }


   public static Supplier<FootstepPostProcessingPacketPubSubType> getPubSubType()
   {
      return FootstepPostProcessingPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPostProcessingPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPostProcessingPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.left_foot_position_in_world_.epsilonEquals(other.left_foot_position_in_world_, epsilon)) return false;
      if (!this.right_foot_position_in_world_.epsilonEquals(other.right_foot_position_in_world_, epsilon)) return false;
      if (!this.left_foot_orientation_in_world_.epsilonEquals(other.left_foot_orientation_in_world_, epsilon)) return false;
      if (!this.right_foot_orientation_in_world_.epsilonEquals(other.right_foot_orientation_in_world_, epsilon)) return false;
      if (this.left_foot_contact_points_2d_.size() != other.left_foot_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.left_foot_contact_points_2d_.size(); i++)
         {  if (!this.left_foot_contact_points_2d_.get(i).epsilonEquals(other.left_foot_contact_points_2d_.get(i), epsilon)) return false; }
      }

      if (this.right_foot_contact_points_2d_.size() != other.right_foot_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.right_foot_contact_points_2d_.size(); i++)
         {  if (!this.right_foot_contact_points_2d_.get(i).epsilonEquals(other.right_foot_contact_points_2d_.get(i), epsilon)) return false; }
      }

      if (!this.footstep_data_list_.epsilonEquals(other.footstep_data_list_, epsilon)) return false;
      if (!this.planar_regions_list_.epsilonEquals(other.planar_regions_list_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPostProcessingPacket)) return false;

      FootstepPostProcessingPacket otherMyClass = (FootstepPostProcessingPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.left_foot_position_in_world_.equals(otherMyClass.left_foot_position_in_world_)) return false;
      if (!this.right_foot_position_in_world_.equals(otherMyClass.right_foot_position_in_world_)) return false;
      if (!this.left_foot_orientation_in_world_.equals(otherMyClass.left_foot_orientation_in_world_)) return false;
      if (!this.right_foot_orientation_in_world_.equals(otherMyClass.right_foot_orientation_in_world_)) return false;
      if (!this.left_foot_contact_points_2d_.equals(otherMyClass.left_foot_contact_points_2d_)) return false;
      if (!this.right_foot_contact_points_2d_.equals(otherMyClass.right_foot_contact_points_2d_)) return false;
      if (!this.footstep_data_list_.equals(otherMyClass.footstep_data_list_)) return false;
      if (!this.planar_regions_list_.equals(otherMyClass.planar_regions_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPostProcessingPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("left_foot_position_in_world=");
      builder.append(this.left_foot_position_in_world_);      builder.append(", ");
      builder.append("right_foot_position_in_world=");
      builder.append(this.right_foot_position_in_world_);      builder.append(", ");
      builder.append("left_foot_orientation_in_world=");
      builder.append(this.left_foot_orientation_in_world_);      builder.append(", ");
      builder.append("right_foot_orientation_in_world=");
      builder.append(this.right_foot_orientation_in_world_);      builder.append(", ");
      builder.append("left_foot_contact_points_2d=");
      builder.append(this.left_foot_contact_points_2d_);      builder.append(", ");
      builder.append("right_foot_contact_points_2d=");
      builder.append(this.right_foot_contact_points_2d_);      builder.append(", ");
      builder.append("footstep_data_list=");
      builder.append(this.footstep_data_list_);      builder.append(", ");
      builder.append("planar_regions_list=");
      builder.append(this.planar_regions_list_);
      builder.append("}");
      return builder.toString();
   }
}
