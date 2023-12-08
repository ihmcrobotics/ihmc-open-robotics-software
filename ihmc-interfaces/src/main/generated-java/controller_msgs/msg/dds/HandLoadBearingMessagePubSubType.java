package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandLoadBearingMessage" defined in "HandLoadBearingMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandLoadBearingMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandLoadBearingMessage_.idl instead.
*
*/
public class HandLoadBearingMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandLoadBearingMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandLoadBearingMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "494b1d8e3720cd4fbe48a2fe7186bd5a949df952a5ff220ca1a3ad77feaa0320";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandLoadBearingMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += controller_msgs.msg.dds.LoadBearingMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandLoadBearingMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandLoadBearingMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectory(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getOrientationTrajectory(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += controller_msgs.msg.dds.LoadBearingMessagePubSubType.getCdrSerializedSize(data.getLoadBearingMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getRobotSide());

      cdr.write_type_7(data.getUseJointspaceCommand());

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectory(), cdr);
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.write(data.getOrientationTrajectory(), cdr);
      cdr.write_type_6(data.getExecutionDelayTime());

      controller_msgs.msg.dds.LoadBearingMessagePubSubType.write(data.getLoadBearingMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      data.setUseJointspaceCommand(cdr.read_type_7());
      	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectory(), cdr);	
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.read(data.getOrientationTrajectory(), cdr);	
      data.setExecutionDelayTime(cdr.read_type_6());
      	
      controller_msgs.msg.dds.LoadBearingMessagePubSubType.read(data.getLoadBearingMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_7("use_jointspace_command", data.getUseJointspaceCommand());
      ser.write_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

      ser.write_type_a("orientation_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getOrientationTrajectory());

      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
      ser.write_type_a("load_bearing_message", new controller_msgs.msg.dds.LoadBearingMessagePubSubType(), data.getLoadBearingMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandLoadBearingMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      data.setUseJointspaceCommand(ser.read_type_7("use_jointspace_command"));
      ser.read_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

      ser.read_type_a("orientation_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getOrientationTrajectory());

      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
      ser.read_type_a("load_bearing_message", new controller_msgs.msg.dds.LoadBearingMessagePubSubType(), data.getLoadBearingMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.HandLoadBearingMessage src, controller_msgs.msg.dds.HandLoadBearingMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandLoadBearingMessage createData()
   {
      return new controller_msgs.msg.dds.HandLoadBearingMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandLoadBearingMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandLoadBearingMessage src, controller_msgs.msg.dds.HandLoadBearingMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandLoadBearingMessagePubSubType newInstance()
   {
      return new HandLoadBearingMessagePubSubType();
   }
}
