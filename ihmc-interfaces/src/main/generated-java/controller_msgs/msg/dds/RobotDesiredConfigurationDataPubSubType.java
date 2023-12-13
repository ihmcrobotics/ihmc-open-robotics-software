package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "RobotDesiredConfigurationData" defined in "RobotDesiredConfigurationData_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from RobotDesiredConfigurationData_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit RobotDesiredConfigurationData_.idl instead.
*
*/
public class RobotDesiredConfigurationDataPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.RobotDesiredConfigurationData>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::RobotDesiredConfigurationData_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "171affad54155ebeb2af0d25df347b95ec8389f0a27c05132c4b09429981e853";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.RobotDesiredConfigurationData data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.RobotDesiredConfigurationData data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.JointDesiredOutputMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotDesiredConfigurationData data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.RobotDesiredConfigurationData data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getJointDesiredOutputList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.JointDesiredOutputMessagePubSubType.getCdrSerializedSize(data.getJointDesiredOutputList().get(i0), current_alignment);}

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootJointTranslation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredRootJointOrientation(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootJointLinearVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootJointAngularVelocity(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootJointLinearAcceleration(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootJointAngularAcceleration(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.RobotDesiredConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_11(data.getWallTime());

      cdr.write_type_2(data.getJointNameHash());

      if(data.getJointDesiredOutputList().size() <= 50)
      cdr.write_type_e(data.getJointDesiredOutputList());else
          throw new RuntimeException("joint_desired_output_list field exceeds the maximum length");

      cdr.write_type_7(data.getHasDesiredRootJointPositionData());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootJointTranslation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredRootJointOrientation(), cdr);
      cdr.write_type_7(data.getHasDesiredRootJointVelocityData());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootJointLinearVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootJointAngularVelocity(), cdr);
      cdr.write_type_7(data.getHasDesiredRootJointAccelerationData());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootJointLinearAcceleration(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootJointAngularAcceleration(), cdr);
   }

   public static void read(controller_msgs.msg.dds.RobotDesiredConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      data.setWallTime(cdr.read_type_11());
      	
      data.setJointNameHash(cdr.read_type_2());
      	
      cdr.read_type_e(data.getJointDesiredOutputList());	
      data.setHasDesiredRootJointPositionData(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootJointTranslation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredRootJointOrientation(), cdr);	
      data.setHasDesiredRootJointVelocityData(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootJointLinearVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootJointAngularVelocity(), cdr);	
      data.setHasDesiredRootJointAccelerationData(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootJointLinearAcceleration(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootJointAngularAcceleration(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.RobotDesiredConfigurationData data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_11("wall_time", data.getWallTime());
      ser.write_type_2("joint_name_hash", data.getJointNameHash());
      ser.write_type_e("joint_desired_output_list", data.getJointDesiredOutputList());
      ser.write_type_7("has_desired_root_joint_position_data", data.getHasDesiredRootJointPositionData());
      ser.write_type_a("desired_root_joint_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointTranslation());

      ser.write_type_a("desired_root_joint_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootJointOrientation());

      ser.write_type_7("has_desired_root_joint_velocity_data", data.getHasDesiredRootJointVelocityData());
      ser.write_type_a("desired_root_joint_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointLinearVelocity());

      ser.write_type_a("desired_root_joint_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointAngularVelocity());

      ser.write_type_7("has_desired_root_joint_acceleration_data", data.getHasDesiredRootJointAccelerationData());
      ser.write_type_a("desired_root_joint_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointLinearAcceleration());

      ser.write_type_a("desired_root_joint_angular_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointAngularAcceleration());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.RobotDesiredConfigurationData data)
   {
      data.setWallTime(ser.read_type_11("wall_time"));
      data.setJointNameHash(ser.read_type_2("joint_name_hash"));
      ser.read_type_e("joint_desired_output_list", data.getJointDesiredOutputList());
      data.setHasDesiredRootJointPositionData(ser.read_type_7("has_desired_root_joint_position_data"));
      ser.read_type_a("desired_root_joint_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointTranslation());

      ser.read_type_a("desired_root_joint_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootJointOrientation());

      data.setHasDesiredRootJointVelocityData(ser.read_type_7("has_desired_root_joint_velocity_data"));
      ser.read_type_a("desired_root_joint_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointLinearVelocity());

      ser.read_type_a("desired_root_joint_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointAngularVelocity());

      data.setHasDesiredRootJointAccelerationData(ser.read_type_7("has_desired_root_joint_acceleration_data"));
      ser.read_type_a("desired_root_joint_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointLinearAcceleration());

      ser.read_type_a("desired_root_joint_angular_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootJointAngularAcceleration());

   }

   public static void staticCopy(controller_msgs.msg.dds.RobotDesiredConfigurationData src, controller_msgs.msg.dds.RobotDesiredConfigurationData dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.RobotDesiredConfigurationData createData()
   {
      return new controller_msgs.msg.dds.RobotDesiredConfigurationData();
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
   
   public void serialize(controller_msgs.msg.dds.RobotDesiredConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.RobotDesiredConfigurationData data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.RobotDesiredConfigurationData src, controller_msgs.msg.dds.RobotDesiredConfigurationData dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public RobotDesiredConfigurationDataPubSubType newInstance()
   {
      return new RobotDesiredConfigurationDataPubSubType();
   }
}
