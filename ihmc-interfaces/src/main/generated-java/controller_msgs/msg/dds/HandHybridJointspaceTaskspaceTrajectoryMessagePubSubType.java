package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandHybridJointspaceTaskspaceTrajectoryMessage" defined in "HandHybridJointspaceTaskspaceTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandHybridJointspaceTaskspaceTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandHybridJointspaceTaskspaceTrajectoryMessage_.idl instead.
*
*/
public class HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandHybridJointspaceTaskspaceTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "17c3dfe04ca108a42c11a5b1e0985add8488ba4c359e4b3a5324c0b724c2588d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.WrenchTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getTaskspaceTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.WrenchTrajectoryMessagePubSubType.getCdrSerializedSize(data.getFeedforwardTaskspaceTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessagePubSubType.getCdrSerializedSize(data.getTaskspacePidGainsTrajectoryMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getForceExecution());

      cdr.write_type_9(data.getRobotSide());

      ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType.write(data.getTaskspaceTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.WrenchTrajectoryMessagePubSubType.write(data.getFeedforwardTaskspaceTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessagePubSubType.write(data.getTaskspacePidGainsTrajectoryMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setForceExecution(cdr.read_type_7());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType.read(data.getTaskspaceTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.WrenchTrajectoryMessagePubSubType.read(data.getFeedforwardTaskspaceTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessagePubSubType.read(data.getTaskspacePidGainsTrajectoryMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("force_execution", data.getForceExecution());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("taskspace_trajectory_message", new ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType(), data.getTaskspaceTrajectoryMessage());

      ser.write_type_a("jointspace_trajectory_message", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectoryMessage());

      ser.write_type_a("feedforward_taskspace_trajectory_message", new controller_msgs.msg.dds.WrenchTrajectoryMessagePubSubType(), data.getFeedforwardTaskspaceTrajectoryMessage());

      ser.write_type_a("taskspace_pid_gains_trajectory_message", new controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessagePubSubType(), data.getTaskspacePidGainsTrajectoryMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setForceExecution(ser.read_type_7("force_execution"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("taskspace_trajectory_message", new ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType(), data.getTaskspaceTrajectoryMessage());

      ser.read_type_a("jointspace_trajectory_message", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectoryMessage());

      ser.read_type_a("feedforward_taskspace_trajectory_message", new controller_msgs.msg.dds.WrenchTrajectoryMessagePubSubType(), data.getFeedforwardTaskspaceTrajectoryMessage());

      ser.read_type_a("taskspace_pid_gains_trajectory_message", new controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessagePubSubType(), data.getTaskspacePidGainsTrajectoryMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage src, controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage src, controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType newInstance()
   {
      return new HandHybridJointspaceTaskspaceTrajectoryMessagePubSubType();
   }
}
