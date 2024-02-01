package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QueueableMessage" defined in "QueueableMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QueueableMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QueueableMessage_.idl instead.
*
*/
public class QueueableMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.QueueableMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::QueueableMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "532884e5aaa2436daa1cc485344f57765ae426d736760fd218be445b0b85978c";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.QueueableMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.QueueableMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.QueueableMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.QueueableMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getExecutionMode());

      cdr.write_type_11(data.getMessageId());

      cdr.write_type_11(data.getPreviousMessageId());

      cdr.write_type_6(data.getExecutionDelayTime());

      cdr.write_type_6(data.getStreamIntegrationDuration());

      cdr.write_type_11(data.getTimestamp());

   }

   public static void read(ihmc_common_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setExecutionMode(cdr.read_type_9());
      	
      data.setMessageId(cdr.read_type_11());
      	
      data.setPreviousMessageId(cdr.read_type_11());
      	
      data.setExecutionDelayTime(cdr.read_type_6());
      	
      data.setStreamIntegrationDuration(cdr.read_type_6());
      	
      data.setTimestamp(cdr.read_type_11());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("execution_mode", data.getExecutionMode());
      ser.write_type_11("message_id", data.getMessageId());
      ser.write_type_11("previous_message_id", data.getPreviousMessageId());
      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
      ser.write_type_6("stream_integration_duration", data.getStreamIntegrationDuration());
      ser.write_type_11("timestamp", data.getTimestamp());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.QueueableMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setExecutionMode(ser.read_type_9("execution_mode"));
      data.setMessageId(ser.read_type_11("message_id"));
      data.setPreviousMessageId(ser.read_type_11("previous_message_id"));
      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
      data.setStreamIntegrationDuration(ser.read_type_6("stream_integration_duration"));
      data.setTimestamp(ser.read_type_11("timestamp"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.QueueableMessage src, ihmc_common_msgs.msg.dds.QueueableMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.QueueableMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.QueueableMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.QueueableMessage src, ihmc_common_msgs.msg.dds.QueueableMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QueueableMessagePubSubType newInstance()
   {
      return new QueueableMessagePubSubType();
   }
}
