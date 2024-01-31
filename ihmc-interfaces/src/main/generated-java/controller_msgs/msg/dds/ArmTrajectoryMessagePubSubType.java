package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ArmTrajectoryMessage" defined in "ArmTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ArmTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ArmTrajectoryMessage_.idl instead.
*
*/
public class ArmTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ArmTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ArmTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "d996f0d93f33dc161df4bc7c9bfc99f2a7d2b3edc134be66aea6c3c26784d9a0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ArmTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ArmTrajectoryMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ArmTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ArmTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ArmTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getForceExecution());

      cdr.write_type_9(data.getRobotSide());

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ArmTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setForceExecution(cdr.read_type_7());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ArmTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("force_execution", data.getForceExecution());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ArmTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setForceExecution(ser.read_type_7("force_execution"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("jointspace_trajectory", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.ArmTrajectoryMessage src, controller_msgs.msg.dds.ArmTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ArmTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.ArmTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ArmTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ArmTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ArmTrajectoryMessage src, controller_msgs.msg.dds.ArmTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ArmTrajectoryMessagePubSubType newInstance()
   {
      return new ArmTrajectoryMessagePubSubType();
   }
}
