package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepQueueStatusMessage" defined in "FootstepQueueStatusMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepQueueStatusMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepQueueStatusMessage_.idl instead.
*
*/
public class FootstepQueueStatusMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepQueueStatusMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepQueueStatusMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c07d554804f1ab8b682cf98fd5f0597dd072e71c38fb980af55864cdcfb19545";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepQueueStatusMessage data) throws java.io.IOException
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
          current_alignment += controller_msgs.msg.dds.QueuedFootstepStatusMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepQueueStatusMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getFootstepDataList().size(); ++i0)
      {
          current_alignment += controller_msgs.msg.dds.QueuedFootstepStatusMessagePubSubType.getCdrSerializedSize(data.getFootstepDataList().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getFootstepDataList().size() <= 50)
      cdr.write_type_e(data.getFootstepDataList());else
          throw new RuntimeException("footstep_data_list field exceeds the maximum length");

   }

   public static void read(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getFootstepDataList());	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("footstep_data_list", data.getFootstepDataList());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepQueueStatusMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("footstep_data_list", data.getFootstepDataList());
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepQueueStatusMessage src, controller_msgs.msg.dds.FootstepQueueStatusMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepQueueStatusMessage createData()
   {
      return new controller_msgs.msg.dds.FootstepQueueStatusMessage();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepQueueStatusMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepQueueStatusMessage src, controller_msgs.msg.dds.FootstepQueueStatusMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepQueueStatusMessagePubSubType newInstance()
   {
      return new FootstepQueueStatusMessagePubSubType();
   }
}
