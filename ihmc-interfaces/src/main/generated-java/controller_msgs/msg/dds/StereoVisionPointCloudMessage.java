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
   /**
            * Specifies whether the pointcloud is expressed in the local coordinate system of the sensor or expressed in world frame.
            */
   public boolean is_data_local_to_sensor_;
   /**
            * There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
            * The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
            * The confidence of the sensor pose represents the quality of the pose estimation.
            */
   public double sensor_pose_confidence_ = 1.0;
   /**
            * There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
            * The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
            * The confidence of the point cloud represents the quality of the collected point cloud data.
            */
   public double point_cloud_confidence_ = 1.0;
   /**
            * The center location of the bounding box of all the points.
            * The location of each point in the pointcloud is with respect to this location.
            */
   public us.ihmc.euclid.tuple3D.Point3D point_cloud_center_;
   /**
            * The pointcloud is down to a given resolution defined in meters.
            */
   public double resolution_;
   /**
            * The number of points in this frame.
            */
   public int number_of_points_;
   /**
            * The pointcloud. The points are stored as 16-bits signed integers.
            * To retrieve the actual coordinate use: "(point_cloud.get() + 0.5) * resolution".
            * The size of this byte buffer is: "number_of_points * 3 * 2".
            * See us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression for more info on the compression protocol.
            */
   public us.ihmc.idl.IDLSequence.Byte  point_cloud_;
   /**
            * The compressed colors.
            * The color for each point in the pointcloud. The colors are stored as 3 bytes: (R, G, B).
            * See us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression for more info on the compression protocol.
            */
   public us.ihmc.idl.IDLSequence.Byte  colors_;
   /**
            * Whether point_cloud and colors have been compressed using the LZ4Compression or not.
            */
   public boolean lz4_compressed_;

   public StereoVisionPointCloudMessage()
   {
      sensor_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      sensor_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      point_cloud_center_ = new us.ihmc.euclid.tuple3D.Point3D();
      point_cloud_ = new us.ihmc.idl.IDLSequence.Byte (20000000, "type_9");

      colors_ = new us.ihmc.idl.IDLSequence.Byte (7000000, "type_9");

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
      is_data_local_to_sensor_ = other.is_data_local_to_sensor_;

      sensor_pose_confidence_ = other.sensor_pose_confidence_;

      point_cloud_confidence_ = other.point_cloud_confidence_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.point_cloud_center_, point_cloud_center_);
      resolution_ = other.resolution_;

      number_of_points_ = other.number_of_points_;

      point_cloud_.set(other.point_cloud_);
      colors_.set(other.colors_);
      lz4_compressed_ = other.lz4_compressed_;

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

   /**
            * Specifies whether the pointcloud is expressed in the local coordinate system of the sensor or expressed in world frame.
            */
   public void setIsDataLocalToSensor(boolean is_data_local_to_sensor)
   {
      is_data_local_to_sensor_ = is_data_local_to_sensor;
   }
   /**
            * Specifies whether the pointcloud is expressed in the local coordinate system of the sensor or expressed in world frame.
            */
   public boolean getIsDataLocalToSensor()
   {
      return is_data_local_to_sensor_;
   }

   /**
            * There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
            * The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
            * The confidence of the sensor pose represents the quality of the pose estimation.
            */
   public void setSensorPoseConfidence(double sensor_pose_confidence)
   {
      sensor_pose_confidence_ = sensor_pose_confidence;
   }
   /**
            * There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
            * The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
            * The confidence of the sensor pose represents the quality of the pose estimation.
            */
   public double getSensorPoseConfidence()
   {
      return sensor_pose_confidence_;
   }

   /**
            * There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
            * The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
            * The confidence of the point cloud represents the quality of the collected point cloud data.
            */
   public void setPointCloudConfidence(double point_cloud_confidence)
   {
      point_cloud_confidence_ = point_cloud_confidence;
   }
   /**
            * There are two types of confidence value noticing the quality of the data for sensor pose and point cloud.
            * The range of confidence is from 0.0 with the worst quality to 1.0 with the best quality.
            * The confidence of the point cloud represents the quality of the collected point cloud data.
            */
   public double getPointCloudConfidence()
   {
      return point_cloud_confidence_;
   }


   /**
            * The center location of the bounding box of all the points.
            * The location of each point in the pointcloud is with respect to this location.
            */
   public us.ihmc.euclid.tuple3D.Point3D getPointCloudCenter()
   {
      return point_cloud_center_;
   }

   /**
            * The pointcloud is down to a given resolution defined in meters.
            */
   public void setResolution(double resolution)
   {
      resolution_ = resolution;
   }
   /**
            * The pointcloud is down to a given resolution defined in meters.
            */
   public double getResolution()
   {
      return resolution_;
   }

   /**
            * The number of points in this frame.
            */
   public void setNumberOfPoints(int number_of_points)
   {
      number_of_points_ = number_of_points;
   }
   /**
            * The number of points in this frame.
            */
   public int getNumberOfPoints()
   {
      return number_of_points_;
   }


   /**
            * The pointcloud. The points are stored as 16-bits signed integers.
            * To retrieve the actual coordinate use: "(point_cloud.get() + 0.5) * resolution".
            * The size of this byte buffer is: "number_of_points * 3 * 2".
            * See us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression for more info on the compression protocol.
            */
   public us.ihmc.idl.IDLSequence.Byte  getPointCloud()
   {
      return point_cloud_;
   }


   /**
            * The compressed colors.
            * The color for each point in the pointcloud. The colors are stored as 3 bytes: (R, G, B).
            * See us.ihmc.robotEnvironmentAwareness.communication.converters.StereoPointCloudCompression for more info on the compression protocol.
            */
   public us.ihmc.idl.IDLSequence.Byte  getColors()
   {
      return colors_;
   }

   /**
            * Whether point_cloud and colors have been compressed using the LZ4Compression or not.
            */
   public void setLz4Compressed(boolean lz4_compressed)
   {
      lz4_compressed_ = lz4_compressed;
   }
   /**
            * Whether point_cloud and colors have been compressed using the LZ4Compression or not.
            */
   public boolean getLz4Compressed()
   {
      return lz4_compressed_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_data_local_to_sensor_, other.is_data_local_to_sensor_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sensor_pose_confidence_, other.sensor_pose_confidence_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.point_cloud_confidence_, other.point_cloud_confidence_, epsilon)) return false;

      if (!this.point_cloud_center_.epsilonEquals(other.point_cloud_center_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.resolution_, other.resolution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_points_, other.number_of_points_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.point_cloud_, other.point_cloud_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.colors_, other.colors_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.lz4_compressed_, other.lz4_compressed_, epsilon)) return false;


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
      if(this.is_data_local_to_sensor_ != otherMyClass.is_data_local_to_sensor_) return false;

      if(this.sensor_pose_confidence_ != otherMyClass.sensor_pose_confidence_) return false;

      if(this.point_cloud_confidence_ != otherMyClass.point_cloud_confidence_) return false;

      if (!this.point_cloud_center_.equals(otherMyClass.point_cloud_center_)) return false;
      if(this.resolution_ != otherMyClass.resolution_) return false;

      if(this.number_of_points_ != otherMyClass.number_of_points_) return false;

      if (!this.point_cloud_.equals(otherMyClass.point_cloud_)) return false;
      if (!this.colors_.equals(otherMyClass.colors_)) return false;
      if(this.lz4_compressed_ != otherMyClass.lz4_compressed_) return false;


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
      builder.append("is_data_local_to_sensor=");
      builder.append(this.is_data_local_to_sensor_);      builder.append(", ");
      builder.append("sensor_pose_confidence=");
      builder.append(this.sensor_pose_confidence_);      builder.append(", ");
      builder.append("point_cloud_confidence=");
      builder.append(this.point_cloud_confidence_);      builder.append(", ");
      builder.append("point_cloud_center=");
      builder.append(this.point_cloud_center_);      builder.append(", ");
      builder.append("resolution=");
      builder.append(this.resolution_);      builder.append(", ");
      builder.append("number_of_points=");
      builder.append(this.number_of_points_);      builder.append(", ");
      builder.append("point_cloud=");
      builder.append(this.point_cloud_);      builder.append(", ");
      builder.append("colors=");
      builder.append(this.colors_);      builder.append(", ");
      builder.append("lz4_compressed=");
      builder.append(this.lz4_compressed_);
      builder.append("}");
      return builder.toString();
   }
}
