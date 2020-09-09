package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxOutputStatus" defined in "KinematicsToolboxOutputStatus_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxOutputStatus_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxOutputStatus_.idl instead.
*
*/
public class KinematicsToolboxOutputStatusPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.KinematicsToolboxOutputStatus>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::KinematicsToolboxOutputStatus_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.KinematicsToolboxOutputStatus data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDesiredJointAngles().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootTranslation(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredRootOrientation(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDesiredJointVelocities().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootLinearVelocity(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredRootAngularVelocity(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getCurrentToolboxState());

      cdr.write_type_2(data.getJointNameHash());

      if(data.getDesiredJointAngles().size() <= 100)
      cdr.write_type_e(data.getDesiredJointAngles());else
          throw new RuntimeException("desired_joint_angles field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootTranslation(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredRootOrientation(), cdr);
      if(data.getDesiredJointVelocities().size() <= 100)
      cdr.write_type_e(data.getDesiredJointVelocities());else
          throw new RuntimeException("desired_joint_velocities field exceeds the maximum length");

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootLinearVelocity(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredRootAngularVelocity(), cdr);
      cdr.write_type_6(data.getSolutionQuality());

   }

   public static void read(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCurrentToolboxState(cdr.read_type_9());
      	
      data.setJointNameHash(cdr.read_type_2());
      	
      cdr.read_type_e(data.getDesiredJointAngles());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootTranslation(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredRootOrientation(), cdr);	
      cdr.read_type_e(data.getDesiredJointVelocities());	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootLinearVelocity(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredRootAngularVelocity(), cdr);	
      data.setSolutionQuality(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("current_toolbox_state", data.getCurrentToolboxState());
      ser.write_type_2("joint_name_hash", data.getJointNameHash());
      ser.write_type_e("desired_joint_angles", data.getDesiredJointAngles());
      ser.write_type_a("desired_root_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootTranslation());

      ser.write_type_a("desired_root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootOrientation());

      ser.write_type_e("desired_joint_velocities", data.getDesiredJointVelocities());
      ser.write_type_a("desired_root_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearVelocity());

      ser.write_type_a("desired_root_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularVelocity());

      ser.write_type_6("solution_quality", data.getSolutionQuality());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.KinematicsToolboxOutputStatus data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCurrentToolboxState(ser.read_type_9("current_toolbox_state"));
      data.setJointNameHash(ser.read_type_2("joint_name_hash"));
      ser.read_type_e("desired_joint_angles", data.getDesiredJointAngles());
      ser.read_type_a("desired_root_translation", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootTranslation());

      ser.read_type_a("desired_root_orientation", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredRootOrientation());

      ser.read_type_e("desired_joint_velocities", data.getDesiredJointVelocities());
      ser.read_type_a("desired_root_linear_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootLinearVelocity());

      ser.read_type_a("desired_root_angular_velocity", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredRootAngularVelocity());

      data.setSolutionQuality(ser.read_type_6("solution_quality"));
   }

   public static void staticCopy(controller_msgs.msg.dds.KinematicsToolboxOutputStatus src, controller_msgs.msg.dds.KinematicsToolboxOutputStatus dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.KinematicsToolboxOutputStatus createData()
   {
      return new controller_msgs.msg.dds.KinematicsToolboxOutputStatus();
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
   
   public void serialize(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.KinematicsToolboxOutputStatus data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.KinematicsToolboxOutputStatus src, controller_msgs.msg.dds.KinematicsToolboxOutputStatus dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxOutputStatusPubSubType newInstance()
   {
      return new KinematicsToolboxOutputStatusPubSubType();
   }
}
