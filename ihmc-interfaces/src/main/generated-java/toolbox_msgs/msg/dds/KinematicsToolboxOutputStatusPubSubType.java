package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxOutputStatus" defined in "KinematicsToolboxOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxOutputStatus_.idl instead.
*
*/
public class KinematicsToolboxOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxOutputStatus_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0cacd2ed6635e87b311f9cef0f6b5877e4be44cf679fb40536df9f3bda811ec0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 32; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDesiredJointAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredRootPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredRootOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDesiredJointVelocities().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootLinearVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootAngularVelocity(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDesiredJointAccelerations().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootLinearAcceleration(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootAngularAcceleration(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getSupportRegion().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSupportRegion().get(i0), current_alignment);}

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getCurrentToolboxState());

      cdr.write_type_2(data.getJointNameHash());

      if(data.getDesiredJointAngles().size() <= 100)
      cdr.write_type_e(data.getDesiredJointAngles());else
          throw new RuntimeException("desired_joint_angles field exceeds the maximum length");

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredRootPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredRootOrientation(), cdr);
      if(data.getDesiredJointVelocities().size() <= 100)
      cdr.write_type_e(data.getDesiredJointVelocities());else
          throw new RuntimeException("desired_joint_velocities field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootLinearVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootAngularVelocity(), cdr);
      if(data.getDesiredJointAccelerations().size() <= 100)
      cdr.write_type_e(data.getDesiredJointAccelerations());else
          throw new RuntimeException("desired_joint_accelerations field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootLinearAcceleration(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootAngularAcceleration(), cdr);
      if(data.getSupportRegion().size() <= 32)
      cdr.write_type_e(data.getSupportRegion());else
          throw new RuntimeException("support_region field exceeds the maximum length");

      cdr.write_type_6(data.getSolutionQuality());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCurrentToolboxState(cdr.read_type_9());
      	
      data.setJointNameHash(cdr.read_type_2());
      	
      cdr.read_type_e(data.getDesiredJointAngles());	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredRootPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredRootOrientation(), cdr);	
      cdr.read_type_e(data.getDesiredJointVelocities());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootLinearVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootAngularVelocity(), cdr);	
      cdr.read_type_e(data.getDesiredJointAccelerations());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootLinearAcceleration(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootAngularAcceleration(), cdr);	
      cdr.read_type_e(data.getSupportRegion());	
      data.setSolutionQuality(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("current_toolbox_state", data.getCurrentToolboxState());
      ser.write_type_2("joint_name_hash", data.getJointNameHash());
      ser.write_type_e("desired_joint_angles", data.getDesiredJointAngles());
      ser.write_type_a("desired_root_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredRootPosition());

      ser.write_type_a("desired_root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootOrientation());

      ser.write_type_e("desired_joint_velocities", data.getDesiredJointVelocities());
      ser.write_type_a("desired_root_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearVelocity());

      ser.write_type_a("desired_root_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularVelocity());

      ser.write_type_e("desired_joint_accelerations", data.getDesiredJointAccelerations());
      ser.write_type_a("desired_root_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearAcceleration());

      ser.write_type_a("desired_root_angular_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularAcceleration());

      ser.write_type_e("support_region", data.getSupportRegion());
      ser.write_type_6("solution_quality", data.getSolutionQuality());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCurrentToolboxState(ser.read_type_9("current_toolbox_state"));
      data.setJointNameHash(ser.read_type_2("joint_name_hash"));
      ser.read_type_e("desired_joint_angles", data.getDesiredJointAngles());
      ser.read_type_a("desired_root_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredRootPosition());

      ser.read_type_a("desired_root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootOrientation());

      ser.read_type_e("desired_joint_velocities", data.getDesiredJointVelocities());
      ser.read_type_a("desired_root_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearVelocity());

      ser.read_type_a("desired_root_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularVelocity());

      ser.read_type_e("desired_joint_accelerations", data.getDesiredJointAccelerations());
      ser.read_type_a("desired_root_linear_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearAcceleration());

      ser.read_type_a("desired_root_angular_acceleration", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularAcceleration());

      ser.read_type_e("support_region", data.getSupportRegion());
      data.setSolutionQuality(ser.read_type_6("solution_quality"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus src, toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus src, toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxOutputStatusPubSubType newInstance()
   {
      return new KinematicsToolboxOutputStatusPubSubType();
   }
}
