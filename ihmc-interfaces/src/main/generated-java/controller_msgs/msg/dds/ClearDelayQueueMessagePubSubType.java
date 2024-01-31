package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ClearDelayQueueMessage" defined in "ClearDelayQueueMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ClearDelayQueueMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ClearDelayQueueMessage_.idl instead.
*
*/
public class ClearDelayQueueMessagePubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ClearDelayQueueMessage>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ClearDelayQueueMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "82ffe2fbd077f17e9ba202e8be9abc6d7a9b85a58a9dd9016a9621bee9b4d6b6";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ClearDelayQueueMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ClearDelayQueueMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ClearDelayQueueMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ClearDelayQueueMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ClearDelayQueueMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_2(data.getClassSimpleNameBasedHashCode());

      cdr.write_type_7(data.getClearAllDelayBuffers());

   }

   public static void read(controller_msgs.msg.dds.ClearDelayQueueMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setClassSimpleNameBasedHashCode(cdr.read_type_2());
      	
      data.setClearAllDelayBuffers(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ClearDelayQueueMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_2("class_simple_name_based_hash_code", data.getClassSimpleNameBasedHashCode());
      ser.write_type_7("clear_all_delay_buffers", data.getClearAllDelayBuffers());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ClearDelayQueueMessage data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setClassSimpleNameBasedHashCode(ser.read_type_2("class_simple_name_based_hash_code"));
      data.setClearAllDelayBuffers(ser.read_type_7("clear_all_delay_buffers"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ClearDelayQueueMessage src, controller_msgs.msg.dds.ClearDelayQueueMessage dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ClearDelayQueueMessage createData()
   {
      return new controller_msgs.msg.dds.ClearDelayQueueMessage();
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
   
   public void serialize(controller_msgs.msg.dds.ClearDelayQueueMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ClearDelayQueueMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ClearDelayQueueMessage src, controller_msgs.msg.dds.ClearDelayQueueMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ClearDelayQueueMessagePubSubType newInstance()
   {
      return new ClearDelayQueueMessagePubSubType();
   }
}
