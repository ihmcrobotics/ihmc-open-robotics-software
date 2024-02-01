package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "WholeBodyImpedanceTrajectoryMessage" defined in "WholeBodyImpedanceTrajectoryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from WholeBodyImpedanceTrajectoryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit WholeBodyImpedanceTrajectoryMessage_.idl instead.
*
*/
public class WholeBodyImpedanceTrajectoryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::WholeBodyImpedanceTrajectoryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "ccdefb882d522754c1c0ac47585d5b57c7d0306e1d0b4f19ec5e784acb8c01d0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (100 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += controller_msgs.msg.dds.MultiContactTimedContactSequenceMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getJointHashCodes().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getJointTrajectoryMessages().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.OneDoFJointTrajectoryMessagePubSubType.getCdrSerializedSize(data.getJointTrajectoryMessages().get(i0), current_alignment);}

      current_alignment += ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.getCdrSerializedSize(data.getQueueingProperties(), current_alignment);

      current_alignment += controller_msgs.msg.dds.MultiContactTimedContactSequenceMessagePubSubType.getCdrSerializedSize(data.getContactSequenceMessage(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getJointHashCodes().size() <= 100)
      cdr.write_type_e(data.getJointHashCodes());else
          throw new RuntimeException("joint_hash_codes field exceeds the maximum length");

      if(data.getJointTrajectoryMessages().size() <= 100)
      cdr.write_type_e(data.getJointTrajectoryMessages());else
          throw new RuntimeException("joint_trajectory_messages field exceeds the maximum length");

      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.write(data.getQueueingProperties(), cdr);
      controller_msgs.msg.dds.MultiContactTimedContactSequenceMessagePubSubType.write(data.getContactSequenceMessage(), cdr);
   }

   public static void read(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getJointHashCodes());	
      cdr.read_type_e(data.getJointTrajectoryMessages());	
      ihmc_common_msgs.msg.dds.QueueableMessagePubSubType.read(data.getQueueingProperties(), cdr);	
      controller_msgs.msg.dds.MultiContactTimedContactSequenceMessagePubSubType.read(data.getContactSequenceMessage(), cdr);	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("joint_hash_codes", data.getJointHashCodes());
      ser.write_type_e("joint_trajectory_messages", data.getJointTrajectoryMessages());
      ser.write_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

      ser.write_type_a("contact_sequence_message", new controller_msgs.msg.dds.MultiContactTimedContactSequenceMessagePubSubType(), data.getContactSequenceMessage());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("joint_hash_codes", data.getJointHashCodes());
      ser.read_type_e("joint_trajectory_messages", data.getJointTrajectoryMessages());
      ser.read_type_a("queueing_properties", new ihmc_common_msgs.msg.dds.QueueableMessagePubSubType(), data.getQueueingProperties());

      ser.read_type_a("contact_sequence_message", new controller_msgs.msg.dds.MultiContactTimedContactSequenceMessagePubSubType(), data.getContactSequenceMessage());

   }

   public static void staticCopy(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage src, controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage createData()
   {
      return new controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage();
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
   
   public void serialize(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage src, controller_msgs.msg.dds.WholeBodyImpedanceTrajectoryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public WholeBodyImpedanceTrajectoryMessagePubSubType newInstance()
   {
      return new WholeBodyImpedanceTrajectoryMessagePubSubType();
   }
}
