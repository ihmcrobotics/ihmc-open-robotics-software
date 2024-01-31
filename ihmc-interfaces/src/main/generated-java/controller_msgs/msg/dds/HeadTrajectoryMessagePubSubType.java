package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeadTrajectoryMessage" defined in "HeadTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeadTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeadTrajectoryMessage_.idl instead.
*
*/
public class HeadTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HeadTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HeadTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "498520b93d9647b2e9acb46dfb40843e09967c5e684ecd775156d630eaa5a6fa";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HeadTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HeadTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HeadTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HeadTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getSo3Trajectory(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HeadTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.write(data.getSo3Trajectory(), cdr);
   }

   public static void read(controller_msgs.msg.dds.HeadTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.read(data.getSo3Trajectory(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HeadTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("so3_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HeadTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("so3_trajectory", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getSo3Trajectory());

   }

   public static void staticCopy(controller_msgs.msg.dds.HeadTrajectoryMessage src, controller_msgs.msg.dds.HeadTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HeadTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.HeadTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HeadTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HeadTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HeadTrajectoryMessage src, controller_msgs.msg.dds.HeadTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeadTrajectoryMessagePubSubType newInstance()
   {
      return new HeadTrajectoryMessagePubSubType();
   }
}
