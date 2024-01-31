package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MultiContactTrajectorySequenceMessage" defined in "MultiContactTrajectorySequenceMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MultiContactTrajectorySequenceMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MultiContactTrajectorySequenceMessage_.idl instead.
*
*/
public class MultiContactTrajectorySequenceMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::MultiContactTrajectorySequenceMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "7364c45f5d577264b066169bf4789a54c6eba9988dd136cc8a7790c78a93f5a7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 50; ++i0)
      {
          current_alignment += controller_msgs.msg.dds.MultiContactTrajectoryMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getTrajectorySequence().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.MultiContactTrajectoryMessagePubSubType.getCdrSerializedSize(data.getTrajectorySequence().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getTrajectorySequence().size() <= 50)
      cdr.write_type_e(data.getTrajectorySequence());else
          throw new RuntimeException("trajectory_sequence field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getTrajectorySequence());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("trajectory_sequence", data.getTrajectorySequence());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("trajectory_sequence", data.getTrajectorySequence());
   }

   public static void staticCopy(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage src, controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage createData()
   {
      return new controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage();
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
   
   public void serialize(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage src, controller_msgs.msg.dds.MultiContactTrajectorySequenceMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MultiContactTrajectorySequenceMessagePubSubType newInstance()
   {
      return new MultiContactTrajectorySequenceMessagePubSubType();
   }
}
