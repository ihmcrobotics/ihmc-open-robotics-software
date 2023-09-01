package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PelvisOrientationTrajectoryMessage" defined in "PelvisOrientationTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PelvisOrientationTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PelvisOrientationTrajectoryMessage_.idl instead.
*
*/
public class PelvisOrientationTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::PelvisOrientationTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bbf28ab951db0bd903feb11b8d9646533375b69e8b6dea0b253e2ae1825bf75d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getSo3Trajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getForceExecution());

      cdr.write_type_7(data.getEnableUserPelvisControlDuringWalking());

      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.write(data.getSo3Trajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setForceExecution(cdr.read_type_7());
      	
      data.setEnableUserPelvisControlDuringWalking(cdr.read_type_7());
      	
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.read(data.getSo3Trajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("force_execution", data.getForceExecution());
      ser.write_type_7("enable_user_pelvis_control_during_walking", data.getEnableUserPelvisControlDuringWalking());
      ser.write_type_a("so3_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setForceExecution(ser.read_type_7("force_execution"));
      data.setEnableUserPelvisControlDuringWalking(ser.read_type_7("enable_user_pelvis_control_during_walking"));
      ser.read_type_a("so3_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage src, controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage src, controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PelvisOrientationTrajectoryMessagePubSubType newInstance()
   {
      return new PelvisOrientationTrajectoryMessagePubSubType();
   }
}
