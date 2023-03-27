package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC robot environment awareness module.
       * This message contains a single planar region.
       * It is defined by an origin, a normal, and a concave hull.
       * For easier processing, a set of convex polygons approximating the concave hull are also provided.
       */
public class SteppableRegionMessage extends Packet<SteppableRegionMessage> implements Settable<SteppableRegionMessage>, EpsilonComparable<SteppableRegionMessage>
{
   public static final int NO_REGION_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Approximate last update time
            */
   public ihmc_common_msgs.msg.dds.InstantMessage last_updated_;
   public int region_id_ = -1;
   public double foot_yaw_;
   public us.ihmc.euclid.tuple3D.Point3D region_origin_;
   public us.ihmc.euclid.tuple4D.Quaternion region_orientation_;
   public us.ihmc.euclid.tuple3D.Vector3D region_normal_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  vertex_buffer_;
   public int concave_hull_size_;
   public perception_msgs.msg.dds.HeightMapMessage local_height_map_;

   public SteppableRegionMessage()
   {
      last_updated_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      region_origin_ = new us.ihmc.euclid.tuple3D.Point3D();
      region_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      region_normal_ = new us.ihmc.euclid.tuple3D.Vector3D();
      vertex_buffer_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (1000, new geometry_msgs.msg.dds.PointPubSubType());
      local_height_map_ = new perception_msgs.msg.dds.HeightMapMessage();

   }

   public SteppableRegionMessage(SteppableRegionMessage other)
   {
      this();
      set(other);
   }

   public void set(SteppableRegionMessage other)
   {
      sequence_id_ = other.sequence_id_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.last_updated_, last_updated_);
      region_id_ = other.region_id_;

      foot_yaw_ = other.foot_yaw_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.region_origin_, region_origin_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.region_orientation_, region_orientation_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.region_normal_, region_normal_);
      vertex_buffer_.set(other.vertex_buffer_);
      concave_hull_size_ = other.concave_hull_size_;

      perception_msgs.msg.dds.HeightMapMessagePubSubType.staticCopy(other.local_height_map_, local_height_map_);
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

   public void setRegionId(int region_id)
   {
      region_id_ = region_id;
   }
   public int getRegionId()
   {
      return region_id_;
   }

   public void setFootYaw(double foot_yaw)
   {
      foot_yaw_ = foot_yaw;
   }
   public double getFootYaw()
   {
      return foot_yaw_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getRegionOrigin()
   {
      return region_origin_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getRegionOrientation()
   {
      return region_orientation_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getRegionNormal()
   {
      return region_normal_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getVertexBuffer()
   {
      return vertex_buffer_;
   }

   public void setConcaveHullSize(int concave_hull_size)
   {
      concave_hull_size_ = concave_hull_size;
   }
   public int getConcaveHullSize()
   {
      return concave_hull_size_;
   }


   public perception_msgs.msg.dds.HeightMapMessage getLocalHeightMap()
   {
      return local_height_map_;
   }


   public static Supplier<SteppableRegionMessagePubSubType> getPubSubType()
   {
      return SteppableRegionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SteppableRegionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SteppableRegionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.last_updated_.epsilonEquals(other.last_updated_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.region_id_, other.region_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foot_yaw_, other.foot_yaw_, epsilon)) return false;

      if (!this.region_origin_.epsilonEquals(other.region_origin_, epsilon)) return false;
      if (!this.region_orientation_.epsilonEquals(other.region_orientation_, epsilon)) return false;
      if (!this.region_normal_.epsilonEquals(other.region_normal_, epsilon)) return false;
      if (this.vertex_buffer_.size() != other.vertex_buffer_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.vertex_buffer_.size(); i++)
         {  if (!this.vertex_buffer_.get(i).epsilonEquals(other.vertex_buffer_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.concave_hull_size_, other.concave_hull_size_, epsilon)) return false;

      if (!this.local_height_map_.epsilonEquals(other.local_height_map_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SteppableRegionMessage)) return false;

      SteppableRegionMessage otherMyClass = (SteppableRegionMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.last_updated_.equals(otherMyClass.last_updated_)) return false;
      if(this.region_id_ != otherMyClass.region_id_) return false;

      if(this.foot_yaw_ != otherMyClass.foot_yaw_) return false;

      if (!this.region_origin_.equals(otherMyClass.region_origin_)) return false;
      if (!this.region_orientation_.equals(otherMyClass.region_orientation_)) return false;
      if (!this.region_normal_.equals(otherMyClass.region_normal_)) return false;
      if (!this.vertex_buffer_.equals(otherMyClass.vertex_buffer_)) return false;
      if(this.concave_hull_size_ != otherMyClass.concave_hull_size_) return false;

      if (!this.local_height_map_.equals(otherMyClass.local_height_map_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SteppableRegionMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("last_updated=");
      builder.append(this.last_updated_);      builder.append(", ");
      builder.append("region_id=");
      builder.append(this.region_id_);      builder.append(", ");
      builder.append("foot_yaw=");
      builder.append(this.foot_yaw_);      builder.append(", ");
      builder.append("region_origin=");
      builder.append(this.region_origin_);      builder.append(", ");
      builder.append("region_orientation=");
      builder.append(this.region_orientation_);      builder.append(", ");
      builder.append("region_normal=");
      builder.append(this.region_normal_);      builder.append(", ");
      builder.append("vertex_buffer=");
      builder.append(this.vertex_buffer_);      builder.append(", ");
      builder.append("concave_hull_size=");
      builder.append(this.concave_hull_size_);      builder.append(", ");
      builder.append("local_height_map=");
      builder.append(this.local_height_map_);
      builder.append("}");
      return builder.toString();
   }
}
