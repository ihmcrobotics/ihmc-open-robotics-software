package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC perception stack.
       * This message contains a list of planar regions along with the sensor pose in world frame.
       * A planar region is a finite area that lies on a 3D plane and that is delimited by a concave hull.
       * As concave hulls are complex to manipulate, a set of convex polygons are also provided, altogether they approximate the area of the planar region.
       */
public class FramePlanarRegionsListMessage extends Packet<FramePlanarRegionsListMessage> implements Settable<FramePlanarRegionsListMessage>, EpsilonComparable<FramePlanarRegionsListMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Position of sensor when the planar region list was generated
            */
   public us.ihmc.euclid.tuple3D.Point3D sensor_position_;
   /**
            * Orientation of sensor when the planar region list was generated
            */
   public us.ihmc.euclid.tuple4D.Quaternion sensor_orientation_;
   /**
            * The PlanarRegionsList associated with the message.
            */
   public perception_msgs.msg.dds.PlanarRegionsListMessage planar_regions_;

   public FramePlanarRegionsListMessage()
   {
      sensor_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      sensor_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      planar_regions_ = new perception_msgs.msg.dds.PlanarRegionsListMessage();
   }

   public FramePlanarRegionsListMessage(FramePlanarRegionsListMessage other)
   {
      this();
      set(other);
   }

   public void set(FramePlanarRegionsListMessage other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.sensor_position_, sensor_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.sensor_orientation_, sensor_orientation_);
      perception_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_, planar_regions_);
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
            * Position of sensor when the planar region list was generated
            */
   public us.ihmc.euclid.tuple3D.Point3D getSensorPosition()
   {
      return sensor_position_;
   }


   /**
            * Orientation of sensor when the planar region list was generated
            */
   public us.ihmc.euclid.tuple4D.Quaternion getSensorOrientation()
   {
      return sensor_orientation_;
   }


   /**
            * The PlanarRegionsList associated with the message.
            */
   public perception_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegions()
   {
      return planar_regions_;
   }


   public static Supplier<FramePlanarRegionsListMessagePubSubType> getPubSubType()
   {
      return FramePlanarRegionsListMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FramePlanarRegionsListMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FramePlanarRegionsListMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.sensor_position_.epsilonEquals(other.sensor_position_, epsilon)) return false;
      if (!this.sensor_orientation_.epsilonEquals(other.sensor_orientation_, epsilon)) return false;
      if (!this.planar_regions_.epsilonEquals(other.planar_regions_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FramePlanarRegionsListMessage)) return false;

      FramePlanarRegionsListMessage otherMyClass = (FramePlanarRegionsListMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.sensor_position_.equals(otherMyClass.sensor_position_)) return false;
      if (!this.sensor_orientation_.equals(otherMyClass.sensor_orientation_)) return false;
      if (!this.planar_regions_.equals(otherMyClass.planar_regions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FramePlanarRegionsListMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("sensor_position=");
      builder.append(this.sensor_position_);      builder.append(", ");
      builder.append("sensor_orientation=");
      builder.append(this.sensor_orientation_);      builder.append(", ");
      builder.append("planar_regions=");
      builder.append(this.planar_regions_);
      builder.append("}");
      return builder.toString();
   }
}
