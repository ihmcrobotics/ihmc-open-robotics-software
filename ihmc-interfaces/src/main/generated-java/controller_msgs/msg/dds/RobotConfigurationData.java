package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message carries the general robot information such as the joints' state and IMU and force sensors' measurement.
       * It is published frequently from the IHMC state estimator.
       */
public class RobotConfigurationData extends Packet<RobotConfigurationData> implements Settable<RobotConfigurationData>, EpsilonComparable<RobotConfigurationData>
{
   public static final byte ROBOT_MOTION_STATUS_UNKNOWN = (byte) 0;
   public static final byte ROBOT_MOTION_STATUS_STANDING = (byte) 1;
   public static final byte ROBOT_MOTION_STATUS_IN_MOTION = (byte) 2;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Time in nanoseconds of the clock hanging on the wall.
            * Takes into account leap seconds/years and is updated by the NTP server (thus can jump backwards).
            * The wall time is usually used in ROS1 for synchronizing timestamps of different time sources (computers, sensors, etc.)
            */
   public long wall_time_;
   /**
            * Time in nanoseconds that represents the absolute elapsed wall-clock time since some arbitrary, fixed point in the past.
            * It is not affected by changes in the system time-of-day clock.
            * This time is usually computed from a real-time process and can be used for reliably computing the time elapsed between two events.
            */
   public long monotonic_time_;
   /**
            * Platform dependent.
            * Time signal in nanoseconds that can be used to synchronize two time sources.
            */
   public long sync_timestamp_;
   public int joint_name_hash_;
   public us.ihmc.idl.IDLSequence.Float  joint_angles_;
   public us.ihmc.idl.IDLSequence.Float  joint_velocities_;
   public us.ihmc.idl.IDLSequence.Float  joint_torques_;
   public us.ihmc.euclid.tuple3D.Point3D root_position_;
   /**
            * The estimated yaw of the root joint in the world frame. This angle is not clamped to [-pi, pi].
            */
   public float unclamped_root_yaw_;
   public us.ihmc.euclid.tuple4D.Quaternion root_orientation_;
   public us.ihmc.euclid.tuple3D.Vector3D pelvis_linear_velocity_;
   public us.ihmc.euclid.tuple3D.Vector3D pelvis_angular_velocity_;
   public us.ihmc.euclid.tuple3D.Vector3D pelvis_linear_acceleration_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SpatialVectorMessage>  force_sensor_data_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.IMUPacket>  imu_sensor_data_;
   public byte robot_motion_status_ = (byte) 255;
   public int last_received_packet_type_id_;
   public long last_received_packet_unique_id_;
   public long last_received_packet_robot_timestamp_;

   public RobotConfigurationData()
   {
      joint_angles_ = new us.ihmc.idl.IDLSequence.Float (50, "type_5");

      joint_velocities_ = new us.ihmc.idl.IDLSequence.Float (50, "type_5");

      joint_torques_ = new us.ihmc.idl.IDLSequence.Float (50, "type_5");

      root_position_ = new us.ihmc.euclid.tuple3D.Point3D();
      root_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      pelvis_linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      pelvis_angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      pelvis_linear_acceleration_ = new us.ihmc.euclid.tuple3D.Vector3D();
      force_sensor_data_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SpatialVectorMessage> (50, new controller_msgs.msg.dds.SpatialVectorMessagePubSubType());
      imu_sensor_data_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.IMUPacket> (50, new controller_msgs.msg.dds.IMUPacketPubSubType());

   }

   public RobotConfigurationData(RobotConfigurationData other)
   {
      this();
      set(other);
   }

   public void set(RobotConfigurationData other)
   {
      sequence_id_ = other.sequence_id_;

      wall_time_ = other.wall_time_;

      monotonic_time_ = other.monotonic_time_;

      sync_timestamp_ = other.sync_timestamp_;

      joint_name_hash_ = other.joint_name_hash_;

      joint_angles_.set(other.joint_angles_);
      joint_velocities_.set(other.joint_velocities_);
      joint_torques_.set(other.joint_torques_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.root_position_, root_position_);
      unclamped_root_yaw_ = other.unclamped_root_yaw_;

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.root_orientation_, root_orientation_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.pelvis_linear_velocity_, pelvis_linear_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.pelvis_angular_velocity_, pelvis_angular_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.pelvis_linear_acceleration_, pelvis_linear_acceleration_);
      force_sensor_data_.set(other.force_sensor_data_);
      imu_sensor_data_.set(other.imu_sensor_data_);
      robot_motion_status_ = other.robot_motion_status_;

      last_received_packet_type_id_ = other.last_received_packet_type_id_;

      last_received_packet_unique_id_ = other.last_received_packet_unique_id_;

      last_received_packet_robot_timestamp_ = other.last_received_packet_robot_timestamp_;

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
            * Time in nanoseconds of the clock hanging on the wall.
            * Takes into account leap seconds/years and is updated by the NTP server (thus can jump backwards).
            * The wall time is usually used in ROS1 for synchronizing timestamps of different time sources (computers, sensors, etc.)
            */
   public void setWallTime(long wall_time)
   {
      wall_time_ = wall_time;
   }
   /**
            * Time in nanoseconds of the clock hanging on the wall.
            * Takes into account leap seconds/years and is updated by the NTP server (thus can jump backwards).
            * The wall time is usually used in ROS1 for synchronizing timestamps of different time sources (computers, sensors, etc.)
            */
   public long getWallTime()
   {
      return wall_time_;
   }

   /**
            * Time in nanoseconds that represents the absolute elapsed wall-clock time since some arbitrary, fixed point in the past.
            * It is not affected by changes in the system time-of-day clock.
            * This time is usually computed from a real-time process and can be used for reliably computing the time elapsed between two events.
            */
   public void setMonotonicTime(long monotonic_time)
   {
      monotonic_time_ = monotonic_time;
   }
   /**
            * Time in nanoseconds that represents the absolute elapsed wall-clock time since some arbitrary, fixed point in the past.
            * It is not affected by changes in the system time-of-day clock.
            * This time is usually computed from a real-time process and can be used for reliably computing the time elapsed between two events.
            */
   public long getMonotonicTime()
   {
      return monotonic_time_;
   }

   /**
            * Platform dependent.
            * Time signal in nanoseconds that can be used to synchronize two time sources.
            */
   public void setSyncTimestamp(long sync_timestamp)
   {
      sync_timestamp_ = sync_timestamp;
   }
   /**
            * Platform dependent.
            * Time signal in nanoseconds that can be used to synchronize two time sources.
            */
   public long getSyncTimestamp()
   {
      return sync_timestamp_;
   }

   public void setJointNameHash(int joint_name_hash)
   {
      joint_name_hash_ = joint_name_hash;
   }
   public int getJointNameHash()
   {
      return joint_name_hash_;
   }


   public us.ihmc.idl.IDLSequence.Float  getJointAngles()
   {
      return joint_angles_;
   }


   public us.ihmc.idl.IDLSequence.Float  getJointVelocities()
   {
      return joint_velocities_;
   }


   public us.ihmc.idl.IDLSequence.Float  getJointTorques()
   {
      return joint_torques_;
   }


   public us.ihmc.euclid.tuple3D.Point3D getRootPosition()
   {
      return root_position_;
   }

   /**
            * The estimated yaw of the root joint in the world frame. This angle is not clamped to [-pi, pi].
            */
   public void setUnclampedRootYaw(float unclamped_root_yaw)
   {
      unclamped_root_yaw_ = unclamped_root_yaw;
   }
   /**
            * The estimated yaw of the root joint in the world frame. This angle is not clamped to [-pi, pi].
            */
   public float getUnclampedRootYaw()
   {
      return unclamped_root_yaw_;
   }


   public us.ihmc.euclid.tuple4D.Quaternion getRootOrientation()
   {
      return root_orientation_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getPelvisLinearVelocity()
   {
      return pelvis_linear_velocity_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getPelvisAngularVelocity()
   {
      return pelvis_angular_velocity_;
   }


   public us.ihmc.euclid.tuple3D.Vector3D getPelvisLinearAcceleration()
   {
      return pelvis_linear_acceleration_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.SpatialVectorMessage>  getForceSensorData()
   {
      return force_sensor_data_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.IMUPacket>  getImuSensorData()
   {
      return imu_sensor_data_;
   }

   public void setRobotMotionStatus(byte robot_motion_status)
   {
      robot_motion_status_ = robot_motion_status;
   }
   public byte getRobotMotionStatus()
   {
      return robot_motion_status_;
   }

   public void setLastReceivedPacketTypeId(int last_received_packet_type_id)
   {
      last_received_packet_type_id_ = last_received_packet_type_id;
   }
   public int getLastReceivedPacketTypeId()
   {
      return last_received_packet_type_id_;
   }

   public void setLastReceivedPacketUniqueId(long last_received_packet_unique_id)
   {
      last_received_packet_unique_id_ = last_received_packet_unique_id;
   }
   public long getLastReceivedPacketUniqueId()
   {
      return last_received_packet_unique_id_;
   }

   public void setLastReceivedPacketRobotTimestamp(long last_received_packet_robot_timestamp)
   {
      last_received_packet_robot_timestamp_ = last_received_packet_robot_timestamp;
   }
   public long getLastReceivedPacketRobotTimestamp()
   {
      return last_received_packet_robot_timestamp_;
   }


   public static Supplier<RobotConfigurationDataPubSubType> getPubSubType()
   {
      return RobotConfigurationDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RobotConfigurationDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RobotConfigurationData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wall_time_, other.wall_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.monotonic_time_, other.monotonic_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sync_timestamp_, other.sync_timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_name_hash_, other.joint_name_hash_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_angles_, other.joint_angles_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_velocities_, other.joint_velocities_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_torques_, other.joint_torques_, epsilon)) return false;

      if (!this.root_position_.epsilonEquals(other.root_position_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.unclamped_root_yaw_, other.unclamped_root_yaw_, epsilon)) return false;

      if (!this.root_orientation_.epsilonEquals(other.root_orientation_, epsilon)) return false;
      if (!this.pelvis_linear_velocity_.epsilonEquals(other.pelvis_linear_velocity_, epsilon)) return false;
      if (!this.pelvis_angular_velocity_.epsilonEquals(other.pelvis_angular_velocity_, epsilon)) return false;
      if (!this.pelvis_linear_acceleration_.epsilonEquals(other.pelvis_linear_acceleration_, epsilon)) return false;
      if (this.force_sensor_data_.size() != other.force_sensor_data_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.force_sensor_data_.size(); i++)
         {  if (!this.force_sensor_data_.get(i).epsilonEquals(other.force_sensor_data_.get(i), epsilon)) return false; }
      }

      if (this.imu_sensor_data_.size() != other.imu_sensor_data_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.imu_sensor_data_.size(); i++)
         {  if (!this.imu_sensor_data_.get(i).epsilonEquals(other.imu_sensor_data_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_motion_status_, other.robot_motion_status_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.last_received_packet_type_id_, other.last_received_packet_type_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.last_received_packet_unique_id_, other.last_received_packet_unique_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.last_received_packet_robot_timestamp_, other.last_received_packet_robot_timestamp_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RobotConfigurationData)) return false;

      RobotConfigurationData otherMyClass = (RobotConfigurationData) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.wall_time_ != otherMyClass.wall_time_) return false;

      if(this.monotonic_time_ != otherMyClass.monotonic_time_) return false;

      if(this.sync_timestamp_ != otherMyClass.sync_timestamp_) return false;

      if(this.joint_name_hash_ != otherMyClass.joint_name_hash_) return false;

      if (!this.joint_angles_.equals(otherMyClass.joint_angles_)) return false;
      if (!this.joint_velocities_.equals(otherMyClass.joint_velocities_)) return false;
      if (!this.joint_torques_.equals(otherMyClass.joint_torques_)) return false;
      if (!this.root_position_.equals(otherMyClass.root_position_)) return false;
      if(this.unclamped_root_yaw_ != otherMyClass.unclamped_root_yaw_) return false;

      if (!this.root_orientation_.equals(otherMyClass.root_orientation_)) return false;
      if (!this.pelvis_linear_velocity_.equals(otherMyClass.pelvis_linear_velocity_)) return false;
      if (!this.pelvis_angular_velocity_.equals(otherMyClass.pelvis_angular_velocity_)) return false;
      if (!this.pelvis_linear_acceleration_.equals(otherMyClass.pelvis_linear_acceleration_)) return false;
      if (!this.force_sensor_data_.equals(otherMyClass.force_sensor_data_)) return false;
      if (!this.imu_sensor_data_.equals(otherMyClass.imu_sensor_data_)) return false;
      if(this.robot_motion_status_ != otherMyClass.robot_motion_status_) return false;

      if(this.last_received_packet_type_id_ != otherMyClass.last_received_packet_type_id_) return false;

      if(this.last_received_packet_unique_id_ != otherMyClass.last_received_packet_unique_id_) return false;

      if(this.last_received_packet_robot_timestamp_ != otherMyClass.last_received_packet_robot_timestamp_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotConfigurationData {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("wall_time=");
      builder.append(this.wall_time_);      builder.append(", ");
      builder.append("monotonic_time=");
      builder.append(this.monotonic_time_);      builder.append(", ");
      builder.append("sync_timestamp=");
      builder.append(this.sync_timestamp_);      builder.append(", ");
      builder.append("joint_name_hash=");
      builder.append(this.joint_name_hash_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(this.joint_angles_);      builder.append(", ");
      builder.append("joint_velocities=");
      builder.append(this.joint_velocities_);      builder.append(", ");
      builder.append("joint_torques=");
      builder.append(this.joint_torques_);      builder.append(", ");
      builder.append("root_position=");
      builder.append(this.root_position_);      builder.append(", ");
      builder.append("unclamped_root_yaw=");
      builder.append(this.unclamped_root_yaw_);      builder.append(", ");
      builder.append("root_orientation=");
      builder.append(this.root_orientation_);      builder.append(", ");
      builder.append("pelvis_linear_velocity=");
      builder.append(this.pelvis_linear_velocity_);      builder.append(", ");
      builder.append("pelvis_angular_velocity=");
      builder.append(this.pelvis_angular_velocity_);      builder.append(", ");
      builder.append("pelvis_linear_acceleration=");
      builder.append(this.pelvis_linear_acceleration_);      builder.append(", ");
      builder.append("force_sensor_data=");
      builder.append(this.force_sensor_data_);      builder.append(", ");
      builder.append("imu_sensor_data=");
      builder.append(this.imu_sensor_data_);      builder.append(", ");
      builder.append("robot_motion_status=");
      builder.append(this.robot_motion_status_);      builder.append(", ");
      builder.append("last_received_packet_type_id=");
      builder.append(this.last_received_packet_type_id_);      builder.append(", ");
      builder.append("last_received_packet_unique_id=");
      builder.append(this.last_received_packet_unique_id_);      builder.append(", ");
      builder.append("last_received_packet_robot_timestamp=");
      builder.append(this.last_received_packet_robot_timestamp_);
      builder.append("}");
      return builder.toString();
   }
}
