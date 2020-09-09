package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "JointspaceTrajectoryStatusMessage" defined in "JointspaceTrajectoryStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from JointspaceTrajectoryStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit JointspaceTrajectoryStatusMessage_.idl instead.
*
*/
public class JointspaceTrajectoryStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::JointspaceTrajectoryStatusMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getJointNames().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getJointNames().get(i0).length() + 1;
      }
      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getDesiredJointPositions().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getActualJointPositions().size() * 8) + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getJointNames().size() <= 100)
      cdr.write_type_e(data.getJointNames());else
          throw new RuntimeException("joint_names field exceeds the maximum length");

      cdr.write_type_9(data.getTrajectoryExecutionStatus());

      cdr.write_type_6(data.getTimestamp());

      if(data.getDesiredJointPositions().size() <= 100)
      cdr.write_type_e(data.getDesiredJointPositions());else
          throw new RuntimeException("desired_joint_positions field exceeds the maximum length");

      if(data.getActualJointPositions().size() <= 100)
      cdr.write_type_e(data.getActualJointPositions());else
          throw new RuntimeException("actual_joint_positions field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getJointNames());	
      data.setTrajectoryExecutionStatus(cdr.read_type_9());
      	
      data.setTimestamp(cdr.read_type_6());
      	
      cdr.read_type_e(data.getDesiredJointPositions());	
      cdr.read_type_e(data.getActualJointPositions());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("joint_names", data.getJointNames());
      ser.write_type_9("trajectory_execution_status", data.getTrajectoryExecutionStatus());
      ser.write_type_6("timestamp", data.getTimestamp());
      ser.write_type_e("desired_joint_positions", data.getDesiredJointPositions());
      ser.write_type_e("actual_joint_positions", data.getActualJointPositions());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("joint_names", data.getJointNames());
      data.setTrajectoryExecutionStatus(ser.read_type_9("trajectory_execution_status"));
      data.setTimestamp(ser.read_type_6("timestamp"));
      ser.read_type_e("desired_joint_positions", data.getDesiredJointPositions());
      ser.read_type_e("actual_joint_positions", data.getActualJointPositions());
   }

   public static void staticCopy(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage src, controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage createData()
   {
      return new controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage src, controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public JointspaceTrajectoryStatusMessagePubSubType newInstance()
   {
      return new JointspaceTrajectoryStatusMessagePubSubType();
   }
}
