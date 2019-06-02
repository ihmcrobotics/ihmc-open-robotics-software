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
            * Absolute timestamp (UTC). This is usually used with ROS and is different from the controller up-time and machine up-time.
            */
   public long timestamp_;
   /**
            * Timestamp representing the controller up-time.
            */
   public long controller_timestamp_;
   public long sensor_head_pps_timestamp_;
   public int joint_name_hash_;
   public us.ihmc.idl.IDLSequence.Float  joint_angles_;
   public us.ihmc.idl.IDLSequence.Float  joint_velocities_;
   public us.ihmc.idl.IDLSequence.Float  joint_torques_;
   public us.ihmc.euclid.tuple3D.Vector3D root_translation_;
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

      root_translation_ = new us.ihmc.euclid.tuple3D.Vector3D();
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

      timestamp_ = other.timestamp_;

      controller_timestamp_ = other.controller_timestamp_;

      sensor_head_pps_timestamp_ = other.sensor_head_pps_timestamp_;

      joint_name_hash_ = other.joint_name_hash_;

      joint_angles_.set(other.joint_angles_);
      joint_velocities_.set(other.joint_velocities_);
      joint_torques_.set(other.joint_torques_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.root_translation_, root_translation_);
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
            * Absolute timestamp (UTC). This is usually used with ROS and is different from the controller up-time and machine up-time.
            */
   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }
   /**
            * Absolute timestamp (UTC). This is usually used with ROS and is different from the controller up-time and machine up-time.
            */
   public long getTimestamp()
   {
      return timestamp_;
   }

   /**
            * Timestamp representing the controller up-time.
            */
   public void setControllerTimestamp(long controller_timestamp)
   {
      controller_timestamp_ = controller_timestamp;
   }
   /**
            * Timestamp representing the controller up-time.
            */
   public long getControllerTimestamp()
   {
      return controller_timestamp_;
   }

   public void setSensorHeadPpsTimestamp(long sensor_head_pps_timestamp)
   {
      sensor_head_pps_timestamp_ = sensor_head_pps_timestamp;
   }
   public long getSensorHeadPpsTimestamp()
   {
      return sensor_head_pps_timestamp_;
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


   public us.ihmc.euclid.tuple3D.Vector3D getRootTranslation()
   {
      return root_translation_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.controller_timestamp_, other.controller_timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sensor_head_pps_timestamp_, other.sensor_head_pps_timestamp_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_name_hash_, other.joint_name_hash_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_angles_, other.joint_angles_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_velocities_, other.joint_velocities_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.joint_torques_, other.joint_torques_, epsilon)) return false;

      if (!this.root_translation_.epsilonEquals(other.root_translation_, epsilon)) return false;
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

      if(this.timestamp_ != otherMyClass.timestamp_) return false;

      if(this.controller_timestamp_ != otherMyClass.controller_timestamp_) return false;

      if(this.sensor_head_pps_timestamp_ != otherMyClass.sensor_head_pps_timestamp_) return false;

      if(this.joint_name_hash_ != otherMyClass.joint_name_hash_) return false;

      if (!this.joint_angles_.equals(otherMyClass.joint_angles_)) return false;
      if (!this.joint_velocities_.equals(otherMyClass.joint_velocities_)) return false;
      if (!this.joint_torques_.equals(otherMyClass.joint_torques_)) return false;
      if (!this.root_translation_.equals(otherMyClass.root_translation_)) return false;
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
      builder.append("timestamp=");
      builder.append(this.timestamp_);      builder.append(", ");
      builder.append("controller_timestamp=");
      builder.append(this.controller_timestamp_);      builder.append(", ");
      builder.append("sensor_head_pps_timestamp=");
      builder.append(this.sensor_head_pps_timestamp_);      builder.append(", ");
      builder.append("joint_name_hash=");
      builder.append(this.joint_name_hash_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(this.joint_angles_);      builder.append(", ");
      builder.append("joint_velocities=");
      builder.append(this.joint_velocities_);      builder.append(", ");
      builder.append("joint_torques=");
      builder.append(this.joint_torques_);      builder.append(", ");
      builder.append("root_translation=");
      builder.append(this.root_translation_);      builder.append(", ");
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
