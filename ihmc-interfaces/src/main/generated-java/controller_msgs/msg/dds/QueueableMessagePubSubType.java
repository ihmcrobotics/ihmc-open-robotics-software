package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "QueueableMessage" defined in "QueueableMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from QueueableMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit QueueableMessage_.idl instead.
*
*/
public class QueueableMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.QueueableMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::QueueableMessage_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.QueueableMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.QueueableMessage data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QueueableMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.QueueableMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_9(data.getExecutionMode());

      cdr.write_type_11(data.getMessageId());

      cdr.write_type_11(data.getPreviousMessageId());

      cdr.write_type_6(data.getExecutionDelayTime());

      cdr.write_type_6(data.getStreamIntegrationDuration());

   }

   public static void read(controller_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setExecutionMode(cdr.read_type_9());
      	
      data.setMessageId(cdr.read_type_11());
      	
      data.setPreviousMessageId(cdr.read_type_11());
      	
      data.setExecutionDelayTime(cdr.read_type_6());
      	
      data.setStreamIntegrationDuration(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_9("execution_mode", data.getExecutionMode());
      ser.write_type_11("message_id", data.getMessageId());
      ser.write_type_11("previous_message_id", data.getPreviousMessageId());
      ser.write_type_6("execution_delay_time", data.getExecutionDelayTime());
      ser.write_type_6("stream_integration_duration", data.getStreamIntegrationDuration());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.QueueableMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setExecutionMode(ser.read_type_9("execution_mode"));
      data.setMessageId(ser.read_type_11("message_id"));
      data.setPreviousMessageId(ser.read_type_11("previous_message_id"));
      data.setExecutionDelayTime(ser.read_type_6("execution_delay_time"));
      data.setStreamIntegrationDuration(ser.read_type_6("stream_integration_duration"));
   }

   public static void staticCopy(controller_msgs.msg.dds.QueueableMessage src, controller_msgs.msg.dds.QueueableMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.QueueableMessage createData()
   {
      return new controller_msgs.msg.dds.QueueableMessage();
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
   
   public void serialize(controller_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.QueueableMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.QueueableMessage src, controller_msgs.msg.dds.QueueableMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public QueueableMessagePubSubType newInstance()
   {
      return new QueueableMessagePubSubType();
   }
}
