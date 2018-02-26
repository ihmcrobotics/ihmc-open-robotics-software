package controller_msgs.msg.dds;

/**
 * Topic data type of the struct "RobotConfigurationData" defined in "RobotConfigurationData_.idl". Use this class to provide the TopicDataType to a Participant.
 *
 * This file was automatically generated from RobotConfigurationData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit RobotConfigurationData_.idl instead.
 */
public class RobotConfigurationDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RobotConfigurationData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RobotConfigurationData_";
   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   public RobotConfigurationDataPubSubType()
   {

   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (50 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (50 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (50 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < 50; ++a)
      {
         current_alignment += geometry_msgs.msg.dds.WrenchPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < 5; ++a)
      {
         current_alignment += sensor_msgs.msg.dds.ImuPubSubType.getMaxCdrSerializedSize(current_alignment);
      }

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotConfigurationData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotConfigurationData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += std_msgs.msg.dds.HeaderPubSubType.getCdrSerializedSize(data.getHeader(), current_alignment);
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJoint_angles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJoint_velocities().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJoint_torques().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getForce_sensor_data().size(); ++a)
      {
         current_alignment += geometry_msgs.msg.dds.WrenchPubSubType.getCdrSerializedSize(data.getForce_sensor_data().get(a), current_alignment);
      }

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for (int a = 0; a < data.getImu_sensor_data().size(); ++a)
      {
         current_alignment += sensor_msgs.msg.dds.ImuPubSubType.getCdrSerializedSize(data.getImu_sensor_data().get(a), current_alignment);
      }

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRoot_translation(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRoot_orientation(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getPelvis_linear_velocity(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getPelvis_angular_velocity(), current_alignment);
      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getPelvis_linear_acceleration(), current_alignment);
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.CDR cdr)
   {

      std_msgs.msg.dds.HeaderPubSubType.write(data.getHeader(), cdr);

      cdr.write_type_11(data.getDropped_messages());

      cdr.write_type_11(data.getSensor_head_pps_timestamp());

      cdr.write_type_2(data.getJoint_name_hash());

      if (data.getJoint_angles().size() <= 50)
         cdr.write_type_e(data.getJoint_angles());
      else
         throw new RuntimeException("joint_angles field exceeds the maximum length");

      if (data.getJoint_velocities().size() <= 50)
         cdr.write_type_e(data.getJoint_velocities());
      else
         throw new RuntimeException("joint_velocities field exceeds the maximum length");

      if (data.getJoint_torques().size() <= 50)
         cdr.write_type_e(data.getJoint_torques());
      else
         throw new RuntimeException("joint_torques field exceeds the maximum length");

      if (data.getForce_sensor_data().size() <= 50)
         cdr.write_type_e(data.getForce_sensor_data());
      else
         throw new RuntimeException("force_sensor_data field exceeds the maximum length");

      if (data.getImu_sensor_data().size() <= 5)
         cdr.write_type_e(data.getImu_sensor_data());
      else
         throw new RuntimeException("imu_sensor_data field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRoot_translation(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRoot_orientation(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getPelvis_linear_velocity(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getPelvis_angular_velocity(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getPelvis_linear_acceleration(), cdr);

      cdr.write_type_2(data.getRobot_motion_status());

      cdr.write_type_2(data.getLast_received_packet_type_id());

      cdr.write_type_11(data.getLast_received_packet_unique_id());

      cdr.write_type_11(data.getLast_received_packet_robot_timestamp());
   }

   public static void read(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.CDR cdr)
   {

      std_msgs.msg.dds.HeaderPubSubType.read(data.getHeader(), cdr);

      data.setDropped_messages(cdr.read_type_11());

      data.setSensor_head_pps_timestamp(cdr.read_type_11());

      data.setJoint_name_hash(cdr.read_type_2());

      cdr.read_type_e(data.getJoint_angles());

      cdr.read_type_e(data.getJoint_velocities());

      cdr.read_type_e(data.getJoint_torques());

      cdr.read_type_e(data.getForce_sensor_data());

      cdr.read_type_e(data.getImu_sensor_data());

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRoot_translation(), cdr);

      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRoot_orientation(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getPelvis_linear_velocity(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getPelvis_angular_velocity(), cdr);

      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getPelvis_linear_acceleration(), cdr);

      data.setRobot_motion_status(cdr.read_type_2());

      data.setLast_received_packet_type_id(cdr.read_type_2());

      data.setLast_received_packet_unique_id(cdr.read_type_11());

      data.setLast_received_packet_robot_timestamp(cdr.read_type_11());
   }

   public static void staticCopy(controller_msgs.msg.dds.RobotConfigurationData src, controller_msgs.msg.dds.RobotConfigurationData dest)
   {
      dest.set(src);
   }

   @Override
   public void serialize(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload)
         throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RobotConfigurationData data)
         throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      ser.write_type_11("dropped_messages", data.getDropped_messages());

      ser.write_type_11("sensor_head_pps_timestamp", data.getSensor_head_pps_timestamp());

      ser.write_type_2("joint_name_hash", data.getJoint_name_hash());

      ser.write_type_e("joint_angles", data.getJoint_angles());

      ser.write_type_e("joint_velocities", data.getJoint_velocities());

      ser.write_type_e("joint_torques", data.getJoint_torques());

      ser.write_type_e("force_sensor_data", data.getForce_sensor_data());

      ser.write_type_e("imu_sensor_data", data.getImu_sensor_data());

      ser.write_type_a("root_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRoot_translation());

      ser.write_type_a("root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRoot_orientation());

      ser.write_type_a("pelvis_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvis_linear_velocity());

      ser.write_type_a("pelvis_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvis_angular_velocity());

      ser.write_type_a("pelvis_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvis_linear_acceleration());

      ser.write_type_2("robot_motion_status", data.getRobot_motion_status());

      ser.write_type_2("last_received_packet_type_id", data.getLast_received_packet_type_id());

      ser.write_type_11("last_received_packet_unique_id", data.getLast_received_packet_unique_id());

      ser.write_type_11("last_received_packet_robot_timestamp", data.getLast_received_packet_robot_timestamp());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RobotConfigurationData data)
   {
      ser.read_type_a("header", new std_msgs.msg.dds.HeaderPubSubType(), data.getHeader());

      data.setDropped_messages(ser.read_type_11("dropped_messages"));

      data.setSensor_head_pps_timestamp(ser.read_type_11("sensor_head_pps_timestamp"));

      data.setJoint_name_hash(ser.read_type_2("joint_name_hash"));

      ser.read_type_e("joint_angles", data.getJoint_angles());

      ser.read_type_e("joint_velocities", data.getJoint_velocities());

      ser.read_type_e("joint_torques", data.getJoint_torques());

      ser.read_type_e("force_sensor_data", data.getForce_sensor_data());

      ser.read_type_e("imu_sensor_data", data.getImu_sensor_data());

      ser.read_type_a("root_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRoot_translation());

      ser.read_type_a("root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRoot_orientation());

      ser.read_type_a("pelvis_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvis_linear_velocity());

      ser.read_type_a("pelvis_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvis_angular_velocity());

      ser.read_type_a("pelvis_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvis_linear_acceleration());

      data.setRobot_motion_status(ser.read_type_2("robot_motion_status"));

      data.setLast_received_packet_type_id(ser.read_type_2("last_received_packet_type_id"));

      data.setLast_received_packet_unique_id(ser.read_type_11("last_received_packet_unique_id"));

      data.setLast_received_packet_robot_timestamp(ser.read_type_11("last_received_packet_robot_timestamp"));
   }

   @Override
   public controller_msgs.msg.dds.RobotConfigurationData createData()
   {
      return new controller_msgs.msg.dds.RobotConfigurationData();
   }

   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }

   public void serialize(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }

   public void copy(controller_msgs.msg.dds.RobotConfigurationData src, controller_msgs.msg.dds.RobotConfigurationData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotConfigurationDataPubSubType newInstance()
   {
      return new RobotConfigurationDataPubSubType();
   }
}