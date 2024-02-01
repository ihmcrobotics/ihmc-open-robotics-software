package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ChestHybridJointspaceTaskspaceTrajectoryMessage" defined in "ChestHybridJointspaceTaskspaceTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ChestHybridJointspaceTaskspaceTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ChestHybridJointspaceTaskspaceTrajectoryMessage_.idl instead.
*
*/
public class ChestHybridJointspaceTaskspaceTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ChestHybridJointspaceTaskspaceTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5509eae047c36be6dfa62a027a85359226176969bea531a9932fa4e6a4f36508";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.getCdrSerializedSize(data.getTaskspaceTrajectoryMessage(), current_alignment);

      current_alignment += controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointspaceTrajectoryMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.write(data.getTaskspaceTrajectoryMessage(), cdr);
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.write(data.getJointspaceTrajectoryMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType.read(data.getTaskspaceTrajectoryMessage(), cdr);	
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.read(data.getJointspaceTrajectoryMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_a("taskspace_trajectory_message", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getTaskspaceTrajectoryMessage());

      ser.write_type_a("jointspace_trajectory_message", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectoryMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_a("taskspace_trajectory_message", new ihmc_common_msgs.msg.dds.SO3TrajectoryMessagePubSubType(), data.getTaskspaceTrajectoryMessage());

      ser.read_type_a("jointspace_trajectory_message", new controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType(), data.getJointspaceTrajectoryMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage src, controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage src, controller_msgs.msg.dds.ChestHybridJointspaceTaskspaceTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ChestHybridJointspaceTaskspaceTrajectoryMessagePubSubType newInstance()
   {
      return new ChestHybridJointspaceTaskspaceTrajectoryMessagePubSubType();
   }
}
