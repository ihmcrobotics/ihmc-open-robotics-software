package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC environment modeling process.
       * This message contains a list of steppable regions.
       * It is defined by an origin, a normal, and a concave hull.
       */
public class SteppableRegionsListMessage extends Packet<SteppableRegionsListMessage> implements Settable<SteppableRegionsListMessage>, EpsilonComparable<SteppableRegionsListMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Approximate last update time
            */
   public ihmc_common_msgs.msg.dds.InstantMessage last_updated_;
   /**
            * Foot yaw for the corresponding regions
            */
   public double foot_yaw_;
   /**
            * ID number referring to each steppable region, it should be unique.
            */
   public us.ihmc.idl.IDLSequence.Integer  region_id_;
   /**
            * The coordinates in world of the origin of each steppable region
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  region_origin_;
   /**
            * The orientation with respect to world of each steppable region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion>  region_orientation_;
   /**
            * The normal expressed in world of each steppable region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  region_normal_;
   /**
            * Indicates for each planar region the number of vertices for the concave hull.
            */
   public us.ihmc.idl.IDLSequence.Integer  concave_hulls_size_;
   /**
            * Single buffer used to store for each planar the vertices of the concave hull and the convex polygons.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  vertex_buffer_;
   /**
            * Height map message used for each of the concave hulls
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.HeightMapMessage>  local_height_map_;

   public SteppableRegionsListMessage()
   {
      last_updated_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      region_id_ = new us.ihmc.idl.IDLSequence.Integer (1000, "type_2");

      region_origin_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (1000, new geometry_msgs.msg.dds.PointPubSubType());
      region_orientation_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion> (1000, new geometry_msgs.msg.dds.QuaternionPubSubType());
      region_normal_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (1000, new geometry_msgs.msg.dds.Vector3PubSubType());
      concave_hulls_size_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");

      vertex_buffer_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (3000, new geometry_msgs.msg.dds.PointPubSubType());
      local_height_map_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.HeightMapMessage> (100, new perception_msgs.msg.dds.HeightMapMessagePubSubType());

   }

   public SteppableRegionsListMessage(SteppableRegionsListMessage other)
   {
      this();
      set(other);
   }

   public void set(SteppableRegionsListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.last_updated_, last_updated_);
      foot_yaw_ = other.foot_yaw_;

      region_id_.set(other.region_id_);
      region_origin_.set(other.region_origin_);
      region_orientation_.set(other.region_orientation_);
      region_normal_.set(other.region_normal_);
      concave_hulls_size_.set(other.concave_hulls_size_);
      vertex_buffer_.set(other.vertex_buffer_);
      local_height_map_.set(other.local_height_map_);
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


   /**
            * Approximate last update time
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getLastUpdated()
   {
      return last_updated_;
   }

   /**
            * Foot yaw for the corresponding regions
            */
   public void setFootYaw(double foot_yaw)
   {
      foot_yaw_ = foot_yaw;
   }
   /**
            * Foot yaw for the corresponding regions
            */
   public double getFootYaw()
   {
      return foot_yaw_;
   }


   /**
            * ID number referring to each steppable region, it should be unique.
            */
   public us.ihmc.idl.IDLSequence.Integer  getRegionId()
   {
      return region_id_;
   }


   /**
            * The coordinates in world of the origin of each steppable region
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getRegionOrigin()
   {
      return region_origin_;
   }


   /**
            * The orientation with respect to world of each steppable region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple4D.Quaternion>  getRegionOrientation()
   {
      return region_orientation_;
   }


   /**
            * The normal expressed in world of each steppable region.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getRegionNormal()
   {
      return region_normal_;
   }


   /**
            * Indicates for each planar region the number of vertices for the concave hull.
            */
   public us.ihmc.idl.IDLSequence.Integer  getConcaveHullsSize()
   {
      return concave_hulls_size_;
   }


   /**
            * Single buffer used to store for each planar the vertices of the concave hull and the convex polygons.
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getVertexBuffer()
   {
      return vertex_buffer_;
   }


   /**
            * Height map message used for each of the concave hulls
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.HeightMapMessage>  getLocalHeightMap()
   {
      return local_height_map_;
   }


   public static Supplier<SteppableRegionsListMessagePubSubType> getPubSubType()
   {
      return SteppableRegionsListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SteppableRegionsListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SteppableRegionsListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.last_updated_.epsilonEquals(other.last_updated_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_yaw_, other.foot_yaw_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.region_id_, other.region_id_, epsilon)) return false;

      if (this.region_origin_.size() != other.region_origin_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_origin_.size(); i++)
         {  if (!this.region_origin_.get(i).epsilonEquals(other.region_origin_.get(i), epsilon)) return false; }
      }

      if (this.region_orientation_.size() != other.region_orientation_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_orientation_.size(); i++)
         {  if (!this.region_orientation_.get(i).epsilonEquals(other.region_orientation_.get(i), epsilon)) return false; }
      }

      if (this.region_normal_.size() != other.region_normal_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.region_normal_.size(); i++)
         {  if (!this.region_normal_.get(i).epsilonEquals(other.region_normal_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.concave_hulls_size_, other.concave_hulls_size_, epsilon)) return false;

      if (this.vertex_buffer_.size() != other.vertex_buffer_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.vertex_buffer_.size(); i++)
         {  if (!this.vertex_buffer_.get(i).epsilonEquals(other.vertex_buffer_.get(i), epsilon)) return false; }
      }

      if (this.local_height_map_.size() != other.local_height_map_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.local_height_map_.size(); i++)
         {  if (!this.local_height_map_.get(i).epsilonEquals(other.local_height_map_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SteppableRegionsListMessage)) return false;

      SteppableRegionsListMessage otherMyClass = (SteppableRegionsListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.last_updated_.equals(otherMyClass.last_updated_)) return false;
      if(this.foot_yaw_ != otherMyClass.foot_yaw_) return false;

      if (!this.region_id_.equals(otherMyClass.region_id_)) return false;
      if (!this.region_origin_.equals(otherMyClass.region_origin_)) return false;
      if (!this.region_orientation_.equals(otherMyClass.region_orientation_)) return false;
      if (!this.region_normal_.equals(otherMyClass.region_normal_)) return false;
      if (!this.concave_hulls_size_.equals(otherMyClass.concave_hulls_size_)) return false;
      if (!this.vertex_buffer_.equals(otherMyClass.vertex_buffer_)) return false;
      if (!this.local_height_map_.equals(otherMyClass.local_height_map_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SteppableRegionsListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("last_updated=");
      builder.append(this.last_updated_);      builder.append(", ");
      builder.append("foot_yaw=");
      builder.append(this.foot_yaw_);      builder.append(", ");
      builder.append("region_id=");
      builder.append(this.region_id_);      builder.append(", ");
      builder.append("region_origin=");
      builder.append(this.region_origin_);      builder.append(", ");
      builder.append("region_orientation=");
      builder.append(this.region_orientation_);      builder.append(", ");
      builder.append("region_normal=");
      builder.append(this.region_normal_);      builder.append(", ");
      builder.append("concave_hulls_size=");
      builder.append(this.concave_hulls_size_);      builder.append(", ");
      builder.append("vertex_buffer=");
      builder.append(this.vertex_buffer_);      builder.append(", ");
      builder.append("local_height_map=");
      builder.append(this.local_height_map_);
      builder.append("}");
      return builder.toString();
   }
}
