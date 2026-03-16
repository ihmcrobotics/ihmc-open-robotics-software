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
   		return "f765e87a3802793e3079c4a5a0528b8a8b0fab25786001408e445ff4d52ce3e7";
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 32; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


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
      for(int i0 = 0; i0 < data.getSupportRegion().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getSupportRegion().get(i0), current_alignment);}

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredTorsoPosition(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredTorsoOrientation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredTorsoLinearVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredTorsoAngularVelocity(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRigidBodyNames().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getRigidBodyNames().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRigidBodyPositions().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getRigidBodyPositions().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRigidBodyOrientations().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getRigidBodyOrientations().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRigidBodyLinearVelocities().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRigidBodyLinearVelocities().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getRigidBodyAngularVelocities().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getRigidBodyAngularVelocities().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_9(data.getCurrentToolboxState());

      cdr.write_type_2(data.getJointNameHash());

      if(data.getDesiredJointAngles().size() <= 100)
      cdr.write_type_e(data.getDesiredJointAngles());else
          throw new RuntimeException("desired_joint_angles field exceeds the maximum length: %d > %d".formatted(data.getDesiredJointAngles().size(), 100));

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredRootPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredRootOrientation(), cdr);
      if(data.getDesiredJointVelocities().size() <= 100)
      cdr.write_type_e(data.getDesiredJointVelocities());else
          throw new RuntimeException("desired_joint_velocities field exceeds the maximum length: %d > %d".formatted(data.getDesiredJointVelocities().size(), 100));

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootLinearVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootAngularVelocity(), cdr);
      if(data.getSupportRegion().size() <= 32)
      cdr.write_type_e(data.getSupportRegion());else
          throw new RuntimeException("support_region field exceeds the maximum length: %d > %d".formatted(data.getSupportRegion().size(), 32));

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredTorsoPosition(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredTorsoOrientation(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredTorsoLinearVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredTorsoAngularVelocity(), cdr);
      if(data.getRigidBodyNames().size() <= 100)
      cdr.write_type_e(data.getRigidBodyNames());else
          throw new RuntimeException("rigid_body_names field exceeds the maximum length: %d > %d".formatted(data.getRigidBodyNames().size(), 100));

      if(data.getRigidBodyPositions().size() <= 100)
      cdr.write_type_e(data.getRigidBodyPositions());else
          throw new RuntimeException("rigid_body_positions field exceeds the maximum length: %d > %d".formatted(data.getRigidBodyPositions().size(), 100));

      if(data.getRigidBodyOrientations().size() <= 100)
      cdr.write_type_e(data.getRigidBodyOrientations());else
          throw new RuntimeException("rigid_body_orientations field exceeds the maximum length: %d > %d".formatted(data.getRigidBodyOrientations().size(), 100));

      if(data.getRigidBodyLinearVelocities().size() <= 100)
      cdr.write_type_e(data.getRigidBodyLinearVelocities());else
          throw new RuntimeException("rigid_body_linear_velocities field exceeds the maximum length: %d > %d".formatted(data.getRigidBodyLinearVelocities().size(), 100));

      if(data.getRigidBodyAngularVelocities().size() <= 100)
      cdr.write_type_e(data.getRigidBodyAngularVelocities());else
          throw new RuntimeException("rigid_body_angular_velocities field exceeds the maximum length: %d > %d".formatted(data.getRigidBodyAngularVelocities().size(), 100));

      cdr.write_type_5(data.getComOffset());

      cdr.write_type_7(data.getLeftFootInContact());

      cdr.write_type_7(data.getRightFootInContact());

      cdr.write_type_6(data.getSolutionQuality());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setCurrentToolboxState(cdr.read_type_9());
      	
      data.setJointNameHash(cdr.read_type_2());
      	
      cdr.read_type_e(data.getDesiredJointAngles());	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredRootPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredRootOrientation(), cdr);	
      cdr.read_type_e(data.getDesiredJointVelocities());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootLinearVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootAngularVelocity(), cdr);	
      cdr.read_type_e(data.getSupportRegion());	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredTorsoPosition(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredTorsoOrientation(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredTorsoLinearVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredTorsoAngularVelocity(), cdr);	
      cdr.read_type_e(data.getRigidBodyNames());	
      cdr.read_type_e(data.getRigidBodyPositions());	
      cdr.read_type_e(data.getRigidBodyOrientations());	
      cdr.read_type_e(data.getRigidBodyLinearVelocities());	
      cdr.read_type_e(data.getRigidBodyAngularVelocities());	
      data.setComOffset(cdr.read_type_5());
      	
      data.setLeftFootInContact(cdr.read_type_7());
      	
      data.setRightFootInContact(cdr.read_type_7());
      	
      data.setSolutionQuality(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_9("current_toolbox_state", data.getCurrentToolboxState());
      ser.write_type_2("joint_name_hash", data.getJointNameHash());
      ser.write_type_e("desired_joint_angles", data.getDesiredJointAngles());
      ser.write_type_a("desired_root_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredRootPosition());

      ser.write_type_a("desired_root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootOrientation());

      ser.write_type_e("desired_joint_velocities", data.getDesiredJointVelocities());
      ser.write_type_a("desired_root_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearVelocity());

      ser.write_type_a("desired_root_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularVelocity());

      ser.write_type_e("support_region", data.getSupportRegion());
      ser.write_type_a("desired_torso_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredTorsoPosition());

      ser.write_type_a("desired_torso_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredTorsoOrientation());

      ser.write_type_a("desired_torso_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredTorsoLinearVelocity());

      ser.write_type_a("desired_torso_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredTorsoAngularVelocity());

      ser.write_type_e("rigid_body_names", data.getRigidBodyNames());
      ser.write_type_e("rigid_body_positions", data.getRigidBodyPositions());
      ser.write_type_e("rigid_body_orientations", data.getRigidBodyOrientations());
      ser.write_type_e("rigid_body_linear_velocities", data.getRigidBodyLinearVelocities());
      ser.write_type_e("rigid_body_angular_velocities", data.getRigidBodyAngularVelocities());
      ser.write_type_5("com_offset", data.getComOffset());
      ser.write_type_7("left_foot_in_contact", data.getLeftFootInContact());
      ser.write_type_7("right_foot_in_contact", data.getRightFootInContact());
      ser.write_type_6("solution_quality", data.getSolutionQuality());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setCurrentToolboxState(ser.read_type_9("current_toolbox_state"));
      data.setJointNameHash(ser.read_type_2("joint_name_hash"));
      ser.read_type_e("desired_joint_angles", data.getDesiredJointAngles());
      ser.read_type_a("desired_root_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredRootPosition());

      ser.read_type_a("desired_root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootOrientation());

      ser.read_type_e("desired_joint_velocities", data.getDesiredJointVelocities());
      ser.read_type_a("desired_root_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearVelocity());

      ser.read_type_a("desired_root_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularVelocity());

      ser.read_type_e("support_region", data.getSupportRegion());
      ser.read_type_a("desired_torso_position", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredTorsoPosition());

      ser.read_type_a("desired_torso_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredTorsoOrientation());

      ser.read_type_a("desired_torso_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredTorsoLinearVelocity());

      ser.read_type_a("desired_torso_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredTorsoAngularVelocity());

      ser.read_type_e("rigid_body_names", data.getRigidBodyNames());
      ser.read_type_e("rigid_body_positions", data.getRigidBodyPositions());
      ser.read_type_e("rigid_body_orientations", data.getRigidBodyOrientations());
      ser.read_type_e("rigid_body_linear_velocities", data.getRigidBodyLinearVelocities());
      ser.read_type_e("rigid_body_angular_velocities", data.getRigidBodyAngularVelocities());
      data.setComOffset(ser.read_type_5("com_offset"));
      data.setLeftFootInContact(ser.read_type_7("left_foot_in_contact"));
      data.setRightFootInContact(ser.read_type_7("right_foot_in_contact"));
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
