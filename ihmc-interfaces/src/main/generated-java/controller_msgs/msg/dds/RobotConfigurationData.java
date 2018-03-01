package controller_msgs.msg.dds;

/**
 * Definition of the class "RobotConfigurationData" defined in RobotConfigurationData_.idl.
 *
 * This file was automatically generated from RobotConfigurationData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit RobotConfigurationData_.idl instead.
 */
public class RobotConfigurationData
{
   private std_msgs.msg.dds.Header header_;
   private long dropped_messages_;
   private long sensor_head_pps_timestamp_;
   private int joint_name_hash_;
   private us.ihmc.idl.IDLSequence.Float joint_angles_;
   private us.ihmc.idl.IDLSequence.Float joint_velocities_;
   private us.ihmc.idl.IDLSequence.Float joint_torques_;
   private us.ihmc.idl.IDLSequence.Object<geometry_msgs.msg.dds.Wrench> force_sensor_data_;
   private us.ihmc.idl.IDLSequence.Object<sensor_msgs.msg.dds.Imu> imu_sensor_data_;
   private us.ihmc.euclid.tuple3D.Vector3D root_translation_;
   private us.ihmc.euclid.tuple4D.Quaternion root_orientation_;
   private us.ihmc.euclid.tuple3D.Vector3D pelvis_linear_velocity_;
   private us.ihmc.euclid.tuple3D.Vector3D pelvis_angular_velocity_;
   private us.ihmc.euclid.tuple3D.Vector3D pelvis_linear_acceleration_;
   private int robot_motion_status_;
   private int last_received_packet_type_id_;
   private long last_received_packet_unique_id_;
   private long last_received_packet_robot_timestamp_;

   public RobotConfigurationData()
   {
      header_ = new std_msgs.msg.dds.Header();
      joint_angles_ = new us.ihmc.idl.IDLSequence.Float(50, "type_5");
      joint_velocities_ = new us.ihmc.idl.IDLSequence.Float(50, "type_5");
      joint_torques_ = new us.ihmc.idl.IDLSequence.Float(50, "type_5");
      force_sensor_data_ = new us.ihmc.idl.IDLSequence.Object<geometry_msgs.msg.dds.Wrench>(50, geometry_msgs.msg.dds.Wrench.class,
                                                                                            new geometry_msgs.msg.dds.WrenchPubSubType());

      imu_sensor_data_ = new us.ihmc.idl.IDLSequence.Object<sensor_msgs.msg.dds.Imu>(5, sensor_msgs.msg.dds.Imu.class, new sensor_msgs.msg.dds.ImuPubSubType());

      root_translation_ = new us.ihmc.euclid.tuple3D.Vector3D();
      root_orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      pelvis_linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      pelvis_angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      pelvis_linear_acceleration_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public void set(RobotConfigurationData other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      dropped_messages_ = other.dropped_messages_;
      sensor_head_pps_timestamp_ = other.sensor_head_pps_timestamp_;
      joint_name_hash_ = other.joint_name_hash_;
      joint_angles_.set(other.joint_angles_);
      joint_velocities_.set(other.joint_velocities_);
      joint_torques_.set(other.joint_torques_);
      force_sensor_data_.set(other.force_sensor_data_);
      imu_sensor_data_.set(other.imu_sensor_data_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.root_translation_, root_translation_);
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.root_orientation_, root_orientation_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.pelvis_linear_velocity_, pelvis_linear_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.pelvis_angular_velocity_, pelvis_angular_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.pelvis_linear_acceleration_, pelvis_linear_acceleration_);
      robot_motion_status_ = other.robot_motion_status_;
      last_received_packet_type_id_ = other.last_received_packet_type_id_;
      last_received_packet_unique_id_ = other.last_received_packet_unique_id_;
      last_received_packet_robot_timestamp_ = other.last_received_packet_robot_timestamp_;
   }

   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public long getDropped_messages()
   {
      return dropped_messages_;
   }

   public void setDropped_messages(long dropped_messages)
   {
      dropped_messages_ = dropped_messages;
   }

   public long getSensor_head_pps_timestamp()
   {
      return sensor_head_pps_timestamp_;
   }

   public void setSensor_head_pps_timestamp(long sensor_head_pps_timestamp)
   {
      sensor_head_pps_timestamp_ = sensor_head_pps_timestamp;
   }

   public int getJoint_name_hash()
   {
      return joint_name_hash_;
   }

   public void setJoint_name_hash(int joint_name_hash)
   {
      joint_name_hash_ = joint_name_hash;
   }

   public us.ihmc.idl.IDLSequence.Float getJoint_angles()
   {
      return joint_angles_;
   }

   public us.ihmc.idl.IDLSequence.Float getJoint_velocities()
   {
      return joint_velocities_;
   }

   public us.ihmc.idl.IDLSequence.Float getJoint_torques()
   {
      return joint_torques_;
   }

   public us.ihmc.idl.IDLSequence.Object<geometry_msgs.msg.dds.Wrench> getForce_sensor_data()
   {
      return force_sensor_data_;
   }

   public us.ihmc.idl.IDLSequence.Object<sensor_msgs.msg.dds.Imu> getImu_sensor_data()
   {
      return imu_sensor_data_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getRoot_translation()
   {
      return root_translation_;
   }

   public us.ihmc.euclid.tuple4D.Quaternion getRoot_orientation()
   {
      return root_orientation_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getPelvis_linear_velocity()
   {
      return pelvis_linear_velocity_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getPelvis_angular_velocity()
   {
      return pelvis_angular_velocity_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getPelvis_linear_acceleration()
   {
      return pelvis_linear_acceleration_;
   }

   public int getRobot_motion_status()
   {
      return robot_motion_status_;
   }

   public void setRobot_motion_status(int robot_motion_status)
   {
      robot_motion_status_ = robot_motion_status;
   }

   public int getLast_received_packet_type_id()
   {
      return last_received_packet_type_id_;
   }

   public void setLast_received_packet_type_id(int last_received_packet_type_id)
   {
      last_received_packet_type_id_ = last_received_packet_type_id;
   }

   public long getLast_received_packet_unique_id()
   {
      return last_received_packet_unique_id_;
   }

   public void setLast_received_packet_unique_id(long last_received_packet_unique_id)
   {
      last_received_packet_unique_id_ = last_received_packet_unique_id;
   }

   public long getLast_received_packet_robot_timestamp()
   {
      return last_received_packet_robot_timestamp_;
   }

   public void setLast_received_packet_robot_timestamp(long last_received_packet_robot_timestamp)
   {
      last_received_packet_robot_timestamp_ = last_received_packet_robot_timestamp;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof RobotConfigurationData))
         return false;
      RobotConfigurationData otherMyClass = (RobotConfigurationData) other;
      boolean returnedValue = true;

      returnedValue &= this.header_.equals(otherMyClass.header_);

      returnedValue &= this.dropped_messages_ == otherMyClass.dropped_messages_;

      returnedValue &= this.sensor_head_pps_timestamp_ == otherMyClass.sensor_head_pps_timestamp_;

      returnedValue &= this.joint_name_hash_ == otherMyClass.joint_name_hash_;

      returnedValue &= this.joint_angles_.equals(otherMyClass.joint_angles_);

      returnedValue &= this.joint_velocities_.equals(otherMyClass.joint_velocities_);

      returnedValue &= this.joint_torques_.equals(otherMyClass.joint_torques_);

      returnedValue &= this.force_sensor_data_.equals(otherMyClass.force_sensor_data_);

      returnedValue &= this.imu_sensor_data_.equals(otherMyClass.imu_sensor_data_);

      returnedValue &= this.root_translation_.equals(otherMyClass.root_translation_);

      returnedValue &= this.root_orientation_.equals(otherMyClass.root_orientation_);

      returnedValue &= this.pelvis_linear_velocity_.equals(otherMyClass.pelvis_linear_velocity_);

      returnedValue &= this.pelvis_angular_velocity_.equals(otherMyClass.pelvis_angular_velocity_);

      returnedValue &= this.pelvis_linear_acceleration_.equals(otherMyClass.pelvis_linear_acceleration_);

      returnedValue &= this.robot_motion_status_ == otherMyClass.robot_motion_status_;

      returnedValue &= this.last_received_packet_type_id_ == otherMyClass.last_received_packet_type_id_;

      returnedValue &= this.last_received_packet_unique_id_ == otherMyClass.last_received_packet_unique_id_;

      returnedValue &= this.last_received_packet_robot_timestamp_ == otherMyClass.last_received_packet_robot_timestamp_;

      return returnedValue;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotConfigurationData {");
      builder.append("header=");
      builder.append(this.header_);

      builder.append(", ");
      builder.append("dropped_messages=");
      builder.append(this.dropped_messages_);

      builder.append(", ");
      builder.append("sensor_head_pps_timestamp=");
      builder.append(this.sensor_head_pps_timestamp_);

      builder.append(", ");
      builder.append("joint_name_hash=");
      builder.append(this.joint_name_hash_);

      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(this.joint_angles_);

      builder.append(", ");
      builder.append("joint_velocities=");
      builder.append(this.joint_velocities_);

      builder.append(", ");
      builder.append("joint_torques=");
      builder.append(this.joint_torques_);

      builder.append(", ");
      builder.append("force_sensor_data=");
      builder.append(this.force_sensor_data_);

      builder.append(", ");
      builder.append("imu_sensor_data=");
      builder.append(this.imu_sensor_data_);

      builder.append(", ");
      builder.append("root_translation=");
      builder.append(this.root_translation_);

      builder.append(", ");
      builder.append("root_orientation=");
      builder.append(this.root_orientation_);

      builder.append(", ");
      builder.append("pelvis_linear_velocity=");
      builder.append(this.pelvis_linear_velocity_);

      builder.append(", ");
      builder.append("pelvis_angular_velocity=");
      builder.append(this.pelvis_angular_velocity_);

      builder.append(", ");
      builder.append("pelvis_linear_acceleration=");
      builder.append(this.pelvis_linear_acceleration_);

      builder.append(", ");
      builder.append("robot_motion_status=");
      builder.append(this.robot_motion_status_);

      builder.append(", ");
      builder.append("last_received_packet_type_id=");
      builder.append(this.last_received_packet_type_id_);

      builder.append(", ");
      builder.append("last_received_packet_unique_id=");
      builder.append(this.last_received_packet_unique_id_);

      builder.append(", ");
      builder.append("last_received_packet_robot_timestamp=");
      builder.append(this.last_received_packet_robot_timestamp_);

      builder.append("}");
      return builder.toString();
   }
}