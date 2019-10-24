package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Should disappear for the ROS equivalent.
       */
public class StereoVisionPointCloudMessage extends Packet<StereoVisionPointCloudMessage> implements Settable<StereoVisionPointCloudMessage>, EpsilonComparable<StereoVisionPointCloudMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public long timestamp_;
   public us.ihmc.euclid.tuple3D.Point3D sensor_position_;
   public us.ihmc.euclid.tuple4D.Quaternion sensor_orientation_;
   public us.ihmc.idl.IDLSequence.Float  point_cloud_;
   public us.ihmc.idl.IDLSequence.Integer  colors_;

   public StereoVisionPointCloudMessage()
   {
      sensor_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      sensor_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      point_cloud_ = new us.ihmc.idl.IDLSequence.Float (600000, "type_5");

      colors_ = new us.ihmc.idl.IDLSequence.Integer (200000, "type_2");

   }

   public StereoVisionPointCloudMessage(StereoVisionPointCloudMessage other)
   {
      this();
      set(other);
   }

   public void set(StereoVisionPointCloudMessage other)
   {
      sequence_id_ = other.sequence_id_;

      timestamp_ = other.timestamp_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.sensor_position_, sensor_position_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.sensor_orientation_, sensor_orientation_);
      point_cloud_.set(other.point_cloud_);
      colors_.set(other.colors_);
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

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   public long getTimestamp()
   {
      return timestamp_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getSensorPosition()
   {
      return sensor_position_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getSensorOrientation()
   {
      return sensor_orientation_;
   }


   public us.ihmc.idl.IDLSequence.Float  getPointCloud()
   {
      return point_cloud_;
   }


   public us.ihmc.idl.IDLSequence.Integer  getColors()
   {
      return colors_;
   }


   public static Supplier<StereoVisionPointCloudMessagePubSubType> getPubSubType()
   {
      return StereoVisionPointCloudMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StereoVisionPointCloudMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StereoVisionPointCloudMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!this.sensor_position_.epsilonEquals(other.sensor_position_, epsilon)) return false;
      if (!this.sensor_orientation_.epsilonEquals(other.sensor_orientation_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.point_cloud_, other.point_cloud_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.colors_, other.colors_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StereoVisionPointCloudMessage)) return false;

      StereoVisionPointCloudMessage otherMyClass = (StereoVisionPointCloudMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if (!this.sensor_position_.equals(otherMyClass.sensor_position_)) return false;
      if (!this.sensor_orientation_.equals(otherMyClass.sensor_orientation_)) return false;
      if (!this.point_cloud_.equals(otherMyClass.point_cloud_)) return false;
      if (!this.colors_.equals(otherMyClass.colors_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StereoVisionPointCloudMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("sensor_position=");
      builder.append(this.sensor_position_);      builder.append(", ");
      builder.append("sensor_orientation=");
      builder.append(this.sensor_orientation_);      builder.append(", ");
      builder.append("point_cloud=");
      builder.append(this.point_cloud_);      builder.append(", ");
      builder.append("colors=");
      builder.append(this.colors_);
      builder.append("}");
      return builder.toString();
   }
}
