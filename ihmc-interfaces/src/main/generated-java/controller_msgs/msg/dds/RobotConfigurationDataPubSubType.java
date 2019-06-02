package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RobotConfigurationData" defined in "RobotConfigurationData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RobotConfigurationData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RobotConfigurationData_.idl instead.
*
*/
public class RobotConfigurationDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RobotConfigurationData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RobotConfigurationData_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RobotConfigurationData data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (50 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (50 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (50 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.SpatialVectorMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.IMUPacketPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointVelocities().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointTorques().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRootTranslation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRootOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getPelvisLinearVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getPelvisAngularVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getPelvisLinearAcceleration(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getForceSensorData().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.SpatialVectorMessagePubSubType.getCdrSerializedSize(data.getForceSensorData().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getImuSensorData().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.IMUPacketPubSubType.getCdrSerializedSize(data.getImuSensorData().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_11(data.getTimestamp());

      cdr.write_type_11(data.getControllerTimestamp());

      cdr.write_type_11(data.getSensorHeadPpsTimestamp());

      cdr.write_type_2(data.getJointNameHash());

      if(data.getJointAngles().size() <= 50)
      cdr.write_type_e(data.getJointAngles());else
          throw new RuntimeException("joint_angles field exceeds the maximum length");

      if(data.getJointVelocities().size() <= 50)
      cdr.write_type_e(data.getJointVelocities());else
          throw new RuntimeException("joint_velocities field exceeds the maximum length");

      if(data.getJointTorques().size() <= 50)
      cdr.write_type_e(data.getJointTorques());else
          throw new RuntimeException("joint_torques field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getRootTranslation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getRootOrientation(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getPelvisLinearVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getPelvisAngularVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getPelvisLinearAcceleration(), cdr);
      if(data.getForceSensorData().size() <= 50)
      cdr.write_type_e(data.getForceSensorData());else
          throw new RuntimeException("force_sensor_data field exceeds the maximum length");

      if(data.getImuSensorData().size() <= 50)
      cdr.write_type_e(data.getImuSensorData());else
          throw new RuntimeException("imu_sensor_data field exceeds the maximum length");

      cdr.write_type_9(data.getRobotMotionStatus());

      cdr.write_type_2(data.getLastReceivedPacketTypeId());

      cdr.write_type_11(data.getLastReceivedPacketUniqueId());

      cdr.write_type_11(data.getLastReceivedPacketRobotTimestamp());

   }

   public static void read(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setTimestamp(cdr.read_type_11());
      	
      data.setControllerTimestamp(cdr.read_type_11());
      	
      data.setSensorHeadPpsTimestamp(cdr.read_type_11());
      	
      data.setJointNameHash(cdr.read_type_2());
      	
      cdr.read_type_e(data.getJointAngles());	
      cdr.read_type_e(data.getJointVelocities());	
      cdr.read_type_e(data.getJointTorques());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getRootTranslation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getRootOrientation(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getPelvisLinearVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getPelvisAngularVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getPelvisLinearAcceleration(), cdr);	
      cdr.read_type_e(data.getForceSensorData());	
      cdr.read_type_e(data.getImuSensorData());	
      data.setRobotMotionStatus(cdr.read_type_9());
      	
      data.setLastReceivedPacketTypeId(cdr.read_type_2());
      	
      data.setLastReceivedPacketUniqueId(cdr.read_type_11());
      	
      data.setLastReceivedPacketRobotTimestamp(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RobotConfigurationData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_11("timestamp", data.getTimestamp());
      ser.write_type_11("controller_timestamp", data.getControllerTimestamp());
      ser.write_type_11("sensor_head_pps_timestamp", data.getSensorHeadPpsTimestamp());
      ser.write_type_2("joint_name_hash", data.getJointNameHash());
      ser.write_type_e("joint_angles", data.getJointAngles());
      ser.write_type_e("joint_velocities", data.getJointVelocities());
      ser.write_type_e("joint_torques", data.getJointTorques());
      ser.write_type_a("root_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRootTranslation());

      ser.write_type_a("root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRootOrientation());

      ser.write_type_a("pelvis_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvisLinearVelocity());

      ser.write_type_a("pelvis_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvisAngularVelocity());

      ser.write_type_a("pelvis_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvisLinearAcceleration());

      ser.write_type_e("force_sensor_data", data.getForceSensorData());
      ser.write_type_e("imu_sensor_data", data.getImuSensorData());
      ser.write_type_9("robot_motion_status", data.getRobotMotionStatus());
      ser.write_type_2("last_received_packet_type_id", data.getLastReceivedPacketTypeId());
      ser.write_type_11("last_received_packet_unique_id", data.getLastReceivedPacketUniqueId());
      ser.write_type_11("last_received_packet_robot_timestamp", data.getLastReceivedPacketRobotTimestamp());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RobotConfigurationData data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setTimestamp(ser.read_type_11("timestamp"));
      data.setControllerTimestamp(ser.read_type_11("controller_timestamp"));
      data.setSensorHeadPpsTimestamp(ser.read_type_11("sensor_head_pps_timestamp"));
      data.setJointNameHash(ser.read_type_2("joint_name_hash"));
      ser.read_type_e("joint_angles", data.getJointAngles());
      ser.read_type_e("joint_velocities", data.getJointVelocities());
      ser.read_type_e("joint_torques", data.getJointTorques());
      ser.read_type_a("root_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getRootTranslation());

      ser.read_type_a("root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getRootOrientation());

      ser.read_type_a("pelvis_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvisLinearVelocity());

      ser.read_type_a("pelvis_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvisAngularVelocity());

      ser.read_type_a("pelvis_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getPelvisLinearAcceleration());

      ser.read_type_e("force_sensor_data", data.getForceSensorData());
      ser.read_type_e("imu_sensor_data", data.getImuSensorData());
      data.setRobotMotionStatus(ser.read_type_9("robot_motion_status"));
      data.setLastReceivedPacketTypeId(ser.read_type_2("last_received_packet_type_id"));
      data.setLastReceivedPacketUniqueId(ser.read_type_11("last_received_packet_unique_id"));
      data.setLastReceivedPacketRobotTimestamp(ser.read_type_11("last_received_packet_robot_timestamp"));
   }

   public static void staticCopy(controller_msgs.msg.dds.RobotConfigurationData src, controller_msgs.msg.dds.RobotConfigurationData dest)
   {
      dest.set(src);
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
