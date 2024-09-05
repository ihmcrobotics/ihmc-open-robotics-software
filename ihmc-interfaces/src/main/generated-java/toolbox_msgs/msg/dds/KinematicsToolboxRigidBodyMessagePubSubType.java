package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "KinematicsToolboxRigidBodyMessage" defined in "KinematicsToolboxRigidBodyMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from KinematicsToolboxRigidBodyMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit KinematicsToolboxRigidBodyMessage_.idl instead.
*
*/
public class KinematicsToolboxRigidBodyMessagePubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::KinematicsToolboxRigidBodyMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "64dc3d3d898527dffb860bbb2dc872fe435da2c49801acaacdcf5198f8c5ee49";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getDesiredPositionInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getDesiredOrientationInWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredLinearVelocityInWorld(), current_alignment);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getDesiredAngularVelocityInWorld(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.PointPubSubType.getCdrSerializedSize(data.getControlFramePositionInEndEffector(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.QuaternionPubSubType.getCdrSerializedSize(data.getControlFrameOrientationInEndEffector(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getAngularSelectionMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.getCdrSerializedSize(data.getLinearSelectionMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getAngularWeightMatrix(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.getCdrSerializedSize(data.getLinearWeightMatrix(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getEndEffectorHashCode());

      geometry_msgs.msg.dds.PointPubSubType.write(data.getDesiredPositionInWorld(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getDesiredOrientationInWorld(), cdr);
      cdr.write_type_7(data.getHasDesiredLinearVelocity());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredLinearVelocityInWorld(), cdr);
      cdr.write_type_7(data.getHasDesiredAngularVelocity());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getDesiredAngularVelocityInWorld(), cdr);
      geometry_msgs.msg.dds.PointPubSubType.write(data.getControlFramePositionInEndEffector(), cdr);
      geometry_msgs.msg.dds.QuaternionPubSubType.write(data.getControlFrameOrientationInEndEffector(), cdr);
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getAngularSelectionMatrix(), cdr);
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.write(data.getLinearSelectionMatrix(), cdr);
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getAngularWeightMatrix(), cdr);
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.write(data.getLinearWeightMatrix(), cdr);
      cdr.write_type_6(data.getLinearRateLimitation());

      cdr.write_type_6(data.getAngularRateLimitation());

   }

   public static void read(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setEndEffectorHashCode(cdr.read_type_2());
      	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getDesiredPositionInWorld(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getDesiredOrientationInWorld(), cdr);	
      data.setHasDesiredLinearVelocity(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredLinearVelocityInWorld(), cdr);	
      data.setHasDesiredAngularVelocity(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getDesiredAngularVelocityInWorld(), cdr);	
      geometry_msgs.msg.dds.PointPubSubType.read(data.getControlFramePositionInEndEffector(), cdr);	
      geometry_msgs.msg.dds.QuaternionPubSubType.read(data.getControlFrameOrientationInEndEffector(), cdr);	
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getAngularSelectionMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType.read(data.getLinearSelectionMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getAngularWeightMatrix(), cdr);	
      ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType.read(data.getLinearWeightMatrix(), cdr);	
      data.setLinearRateLimitation(cdr.read_type_6());
      	
      data.setAngularRateLimitation(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("end_effector_hash_code", data.getEndEffectorHashCode());
      ser.write_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());

      ser.write_type_a("desired_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredOrientationInWorld());

      ser.write_type_7("has_desired_linear_velocity", data.getHasDesiredLinearVelocity());
      ser.write_type_a("desired_linear_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredLinearVelocityInWorld());

      ser.write_type_7("has_desired_angular_velocity", data.getHasDesiredAngularVelocity());
      ser.write_type_a("desired_angular_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredAngularVelocityInWorld());

      ser.write_type_a("control_frame_position_in_end_effector", new geometry_msgs.msg.dds.PointPubSubType(), data.getControlFramePositionInEndEffector());

      ser.write_type_a("control_frame_orientation_in_end_effector", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getControlFrameOrientationInEndEffector());

      ser.write_type_a("angular_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());

      ser.write_type_a("linear_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());

      ser.write_type_a("angular_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getAngularWeightMatrix());

      ser.write_type_a("linear_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getLinearWeightMatrix());

      ser.write_type_6("linear_rate_limitation", data.getLinearRateLimitation());
      ser.write_type_6("angular_rate_limitation", data.getAngularRateLimitation());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setEndEffectorHashCode(ser.read_type_2("end_effector_hash_code"));
      ser.read_type_a("desired_position_in_world", new geometry_msgs.msg.dds.PointPubSubType(), data.getDesiredPositionInWorld());

      ser.read_type_a("desired_orientation_in_world", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getDesiredOrientationInWorld());

      data.setHasDesiredLinearVelocity(ser.read_type_7("has_desired_linear_velocity"));
      ser.read_type_a("desired_linear_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredLinearVelocityInWorld());

      data.setHasDesiredAngularVelocity(ser.read_type_7("has_desired_angular_velocity"));
      ser.read_type_a("desired_angular_velocity_in_world", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getDesiredAngularVelocityInWorld());

      ser.read_type_a("control_frame_position_in_end_effector", new geometry_msgs.msg.dds.PointPubSubType(), data.getControlFramePositionInEndEffector());

      ser.read_type_a("control_frame_orientation_in_end_effector", new geometry_msgs.msg.dds.QuaternionPubSubType(), data.getControlFrameOrientationInEndEffector());

      ser.read_type_a("angular_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getAngularSelectionMatrix());

      ser.read_type_a("linear_selection_matrix", new ihmc_common_msgs.msg.dds.SelectionMatrix3DMessagePubSubType(), data.getLinearSelectionMatrix());

      ser.read_type_a("angular_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getAngularWeightMatrix());

      ser.read_type_a("linear_weight_matrix", new ihmc_common_msgs.msg.dds.WeightMatrix3DMessagePubSubType(), data.getLinearWeightMatrix());

      data.setLinearRateLimitation(ser.read_type_6("linear_rate_limitation"));
      data.setAngularRateLimitation(ser.read_type_6("angular_rate_limitation"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage src, toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage createData()
   {
      return new toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage();
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
   
   public void serialize(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage src, toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public KinematicsToolboxRigidBodyMessagePubSubType newInstance()
   {
      return new KinematicsToolboxRigidBodyMessagePubSubType();
   }
}
