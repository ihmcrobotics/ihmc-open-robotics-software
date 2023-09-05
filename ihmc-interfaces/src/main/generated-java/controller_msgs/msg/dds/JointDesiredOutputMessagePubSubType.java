package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "JointDesiredOutputMessage" defined in "JointDesiredOutputMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from JointDesiredOutputMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit JointDesiredOutputMessage_.idl instead.
*
*/
public class JointDesiredOutputMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.JointDesiredOutputMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::JointDesiredOutputMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "045fb22c5bfdc0c6acb6d80ffb074321114eabbeb4f5ecf7512c939ca2dd4437";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.JointDesiredOutputMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.JointDesiredOutputMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointDesiredOutputMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointDesiredOutputMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getJointName().length() + 1;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.JointDesiredOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getJointName().length() <= 255)
      cdr.write_type_d(data.getJointName());else
          throw new RuntimeException("joint_name field exceeds the maximum length");

      cdr.write_type_9(data.getControlMode());

      cdr.write_type_7(data.getHasDesiredTorque());

      cdr.write_type_7(data.getHasDesiredPosition());

      cdr.write_type_7(data.getHasDesiredVelocity());

      cdr.write_type_7(data.getHasDesiredAcceleration());

      cdr.write_type_7(data.getHasStiffness());

      cdr.write_type_7(data.getHasDamping());

      cdr.write_type_7(data.getHasMasterGain());

      cdr.write_type_7(data.getHasVelocityScaling());

      cdr.write_type_7(data.getHasPositionIntegrationBreakFrequency());

      cdr.write_type_7(data.getHasVelocityIntegrationBreakFrequency());

      cdr.write_type_7(data.getHasPositionIntegrationMaxError());

      cdr.write_type_7(data.getHasVelocityIntegrationMaxError());

      cdr.write_type_7(data.getHasPositionFeedbackMaxError());

      cdr.write_type_7(data.getHasVelocityFeedbackMaxError());

      cdr.write_type_6(data.getDesiredTorque());

      cdr.write_type_6(data.getDesiredPosition());

      cdr.write_type_6(data.getDesiredVelocity());

      cdr.write_type_6(data.getDesiredAcceleration());

      cdr.write_type_6(data.getStiffness());

      cdr.write_type_6(data.getDamping());

      cdr.write_type_6(data.getMasterGain());

      cdr.write_type_6(data.getVelocityScaling());

      cdr.write_type_6(data.getPositionIntegrationBreakFrequency());

      cdr.write_type_6(data.getVelocityIntegrationBreakFrequency());

      cdr.write_type_6(data.getPositionIntegrationMaxError());

      cdr.write_type_6(data.getVelocityIntegrationMaxError());

      cdr.write_type_6(data.getPositionFeedbackMaxError());

      cdr.write_type_6(data.getVelocityFeedbackMaxError());

   }

   public static void read(controller_msgs.msg.dds.JointDesiredOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_d(data.getJointName());	
      data.setControlMode(cdr.read_type_9());
      	
      data.setHasDesiredTorque(cdr.read_type_7());
      	
      data.setHasDesiredPosition(cdr.read_type_7());
      	
      data.setHasDesiredVelocity(cdr.read_type_7());
      	
      data.setHasDesiredAcceleration(cdr.read_type_7());
      	
      data.setHasStiffness(cdr.read_type_7());
      	
      data.setHasDamping(cdr.read_type_7());
      	
      data.setHasMasterGain(cdr.read_type_7());
      	
      data.setHasVelocityScaling(cdr.read_type_7());
      	
      data.setHasPositionIntegrationBreakFrequency(cdr.read_type_7());
      	
      data.setHasVelocityIntegrationBreakFrequency(cdr.read_type_7());
      	
      data.setHasPositionIntegrationMaxError(cdr.read_type_7());
      	
      data.setHasVelocityIntegrationMaxError(cdr.read_type_7());
      	
      data.setHasPositionFeedbackMaxError(cdr.read_type_7());
      	
      data.setHasVelocityFeedbackMaxError(cdr.read_type_7());
      	
      data.setDesiredTorque(cdr.read_type_6());
      	
      data.setDesiredPosition(cdr.read_type_6());
      	
      data.setDesiredVelocity(cdr.read_type_6());
      	
      data.setDesiredAcceleration(cdr.read_type_6());
      	
      data.setStiffness(cdr.read_type_6());
      	
      data.setDamping(cdr.read_type_6());
      	
      data.setMasterGain(cdr.read_type_6());
      	
      data.setVelocityScaling(cdr.read_type_6());
      	
      data.setPositionIntegrationBreakFrequency(cdr.read_type_6());
      	
      data.setVelocityIntegrationBreakFrequency(cdr.read_type_6());
      	
      data.setPositionIntegrationMaxError(cdr.read_type_6());
      	
      data.setVelocityIntegrationMaxError(cdr.read_type_6());
      	
      data.setPositionFeedbackMaxError(cdr.read_type_6());
      	
      data.setVelocityFeedbackMaxError(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.JointDesiredOutputMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_d("joint_name", data.getJointName());
      ser.write_type_9("control_mode", data.getControlMode());
      ser.write_type_7("has_desired_torque", data.getHasDesiredTorque());
      ser.write_type_7("has_desired_position", data.getHasDesiredPosition());
      ser.write_type_7("has_desired_velocity", data.getHasDesiredVelocity());
      ser.write_type_7("has_desired_acceleration", data.getHasDesiredAcceleration());
      ser.write_type_7("has_stiffness", data.getHasStiffness());
      ser.write_type_7("has_damping", data.getHasDamping());
      ser.write_type_7("has_master_gain", data.getHasMasterGain());
      ser.write_type_7("has_velocity_scaling", data.getHasVelocityScaling());
      ser.write_type_7("has_position_integration_break_frequency", data.getHasPositionIntegrationBreakFrequency());
      ser.write_type_7("has_velocity_integration_break_frequency", data.getHasVelocityIntegrationBreakFrequency());
      ser.write_type_7("has_position_integration_max_error", data.getHasPositionIntegrationMaxError());
      ser.write_type_7("has_velocity_integration_max_error", data.getHasVelocityIntegrationMaxError());
      ser.write_type_7("has_position_feedback_max_error", data.getHasPositionFeedbackMaxError());
      ser.write_type_7("has_velocity_feedback_max_error", data.getHasVelocityFeedbackMaxError());
      ser.write_type_6("desired_torque", data.getDesiredTorque());
      ser.write_type_6("desired_position", data.getDesiredPosition());
      ser.write_type_6("desired_velocity", data.getDesiredVelocity());
      ser.write_type_6("desired_acceleration", data.getDesiredAcceleration());
      ser.write_type_6("stiffness", data.getStiffness());
      ser.write_type_6("damping", data.getDamping());
      ser.write_type_6("master_gain", data.getMasterGain());
      ser.write_type_6("velocity_scaling", data.getVelocityScaling());
      ser.write_type_6("position_integration_break_frequency", data.getPositionIntegrationBreakFrequency());
      ser.write_type_6("velocity_integration_break_frequency", data.getVelocityIntegrationBreakFrequency());
      ser.write_type_6("position_integration_max_error", data.getPositionIntegrationMaxError());
      ser.write_type_6("velocity_integration_max_error", data.getVelocityIntegrationMaxError());
      ser.write_type_6("position_feedback_max_error", data.getPositionFeedbackMaxError());
      ser.write_type_6("velocity_feedback_max_error", data.getVelocityFeedbackMaxError());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.JointDesiredOutputMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_d("joint_name", data.getJointName());
      data.setControlMode(ser.read_type_9("control_mode"));
      data.setHasDesiredTorque(ser.read_type_7("has_desired_torque"));
      data.setHasDesiredPosition(ser.read_type_7("has_desired_position"));
      data.setHasDesiredVelocity(ser.read_type_7("has_desired_velocity"));
      data.setHasDesiredAcceleration(ser.read_type_7("has_desired_acceleration"));
      data.setHasStiffness(ser.read_type_7("has_stiffness"));
      data.setHasDamping(ser.read_type_7("has_damping"));
      data.setHasMasterGain(ser.read_type_7("has_master_gain"));
      data.setHasVelocityScaling(ser.read_type_7("has_velocity_scaling"));
      data.setHasPositionIntegrationBreakFrequency(ser.read_type_7("has_position_integration_break_frequency"));
      data.setHasVelocityIntegrationBreakFrequency(ser.read_type_7("has_velocity_integration_break_frequency"));
      data.setHasPositionIntegrationMaxError(ser.read_type_7("has_position_integration_max_error"));
      data.setHasVelocityIntegrationMaxError(ser.read_type_7("has_velocity_integration_max_error"));
      data.setHasPositionFeedbackMaxError(ser.read_type_7("has_position_feedback_max_error"));
      data.setHasVelocityFeedbackMaxError(ser.read_type_7("has_velocity_feedback_max_error"));
      data.setDesiredTorque(ser.read_type_6("desired_torque"));
      data.setDesiredPosition(ser.read_type_6("desired_position"));
      data.setDesiredVelocity(ser.read_type_6("desired_velocity"));
      data.setDesiredAcceleration(ser.read_type_6("desired_acceleration"));
      data.setStiffness(ser.read_type_6("stiffness"));
      data.setDamping(ser.read_type_6("damping"));
      data.setMasterGain(ser.read_type_6("master_gain"));
      data.setVelocityScaling(ser.read_type_6("velocity_scaling"));
      data.setPositionIntegrationBreakFrequency(ser.read_type_6("position_integration_break_frequency"));
      data.setVelocityIntegrationBreakFrequency(ser.read_type_6("velocity_integration_break_frequency"));
      data.setPositionIntegrationMaxError(ser.read_type_6("position_integration_max_error"));
      data.setVelocityIntegrationMaxError(ser.read_type_6("velocity_integration_max_error"));
      data.setPositionFeedbackMaxError(ser.read_type_6("position_feedback_max_error"));
      data.setVelocityFeedbackMaxError(ser.read_type_6("velocity_feedback_max_error"));
   }

   public static void staticCopy(controller_msgs.msg.dds.JointDesiredOutputMessage src, controller_msgs.msg.dds.JointDesiredOutputMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.JointDesiredOutputMessage createData()
   {
      return new controller_msgs.msg.dds.JointDesiredOutputMessage();
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
   
   public void serialize(controller_msgs.msg.dds.JointDesiredOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.JointDesiredOutputMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.JointDesiredOutputMessage src, controller_msgs.msg.dds.JointDesiredOutputMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JointDesiredOutputMessagePubSubType newInstance()
   {
      return new JointDesiredOutputMessagePubSubType();
   }
}
