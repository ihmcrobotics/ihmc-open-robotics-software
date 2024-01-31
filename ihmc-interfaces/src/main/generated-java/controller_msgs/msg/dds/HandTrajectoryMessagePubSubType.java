package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HandTrajectoryMessage" defined in "HandTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HandTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HandTrajectoryMessage_.idl instead.
*
*/
public class HandTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HandTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HandTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fbf7f1cee0f2224c948116d25730baa239af7aa9c8d518ca2984fc0dee2a20e1";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HandTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HandTrajectoryMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HandTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getSe3Trajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HandTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getForceExecution());

      cdr.write_type_9(data.getRobotSide());

      ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType.write(data.getSe3Trajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.HandTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setForceExecution(cdr.read_type_7());
      	
      data.setRobotSide(cdr.read_type_9());
      	
      ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType.read(data.getSe3Trajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HandTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("force_execution", data.getForceExecution());
      ser.write_type_9("robot_side", data.getRobotSide());
      ser.write_type_a("se3_trajectory", new ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType(), data.getSe3Trajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HandTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setForceExecution(ser.read_type_7("force_execution"));
      data.setRobotSide(ser.read_type_9("robot_side"));
      ser.read_type_a("se3_trajectory", new ihmc_common_msgs.msg.dds.SE3TrajectoryMessagePubSubType(), data.getSe3Trajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.HandTrajectoryMessage src, controller_msgs.msg.dds.HandTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HandTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.HandTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HandTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HandTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HandTrajectoryMessage src, controller_msgs.msg.dds.HandTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HandTrajectoryMessagePubSubType newInstance()
   {
      return new HandTrajectoryMessagePubSubType();
   }
}
