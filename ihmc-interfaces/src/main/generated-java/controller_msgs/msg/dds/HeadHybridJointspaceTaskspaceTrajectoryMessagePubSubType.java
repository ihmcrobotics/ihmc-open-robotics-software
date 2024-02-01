package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeadHybridJointspaceTaskspaceTrajectoryMessage" defined in "HeadHybridJointspaceTaskspaceTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeadHybridJointspaceTaskspaceTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeadHybridJointspaceTaskspaceTrajectoryMessage_.idl instead.
*
*/
public class HeadHybridJointspaceTaskspaceTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::HeadHybridJointspaceTaskspaceTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "73fa2864bc59e70dc8240e13135b07b661400a4dc6587ccb30e2589d1ae30bb3";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getTaskspaceTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectoryMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.write(data.getTaskspaceTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectoryMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.read(data.getTaskspaceTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectoryMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("taskspace_trajectory_message", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getTaskspaceTrajectoryMessage());

      ser.write_type_a("jointspace_trajectory_message", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectoryMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("taskspace_trajectory_message", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getTaskspaceTrajectoryMessage());

      ser.read_type_a("jointspace_trajectory_message", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectoryMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage src, controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage src, controller_msgs.msg.dds.HeadHybridJointspaceTaskspaceTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeadHybridJointspaceTaskspaceTrajectoryMessagePubSubType newInstance()
   {
      return new HeadHybridJointspaceTaskspaceTrajectoryMessagePubSubType();
   }
}
