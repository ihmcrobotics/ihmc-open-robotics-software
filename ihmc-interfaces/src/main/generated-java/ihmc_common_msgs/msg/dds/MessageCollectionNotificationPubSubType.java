package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MessageCollectionNotification" defined in "MessageCollectionNotification_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MessageCollectionNotification_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MessageCollectionNotification_.idl instead.
*
*/
public class MessageCollectionNotificationPubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.MessageCollectionNotification>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::MessageCollectionNotification_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fd4ad86429a46561f9a8b2b484e5d955c22e42bbf67f10aaaf56e6e39878a3c4";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.MessageCollectionNotification data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.MessageCollectionNotification data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.MessageCollectionNotification data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.MessageCollectionNotification data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.MessageCollectionNotification data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_4(data.getMessageCollectionSequenceId());

   }

   public static void read(ihmc_common_msgs.msg.dds.MessageCollectionNotification data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setMessageCollectionSequenceId(cdr.read_type_4());
      	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.MessageCollectionNotification data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_4("message_collection_sequence_id", data.getMessageCollectionSequenceId());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.MessageCollectionNotification data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setMessageCollectionSequenceId(ser.read_type_4("message_collection_sequence_id"));
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.MessageCollectionNotification src, ihmc_common_msgs.msg.dds.MessageCollectionNotification dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.MessageCollectionNotification createData()
   {
      return new ihmc_common_msgs.msg.dds.MessageCollectionNotification();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.MessageCollectionNotification data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.MessageCollectionNotification data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.MessageCollectionNotification src, ihmc_common_msgs.msg.dds.MessageCollectionNotification dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MessageCollectionNotificationPubSubType newInstance()
   {
      return new MessageCollectionNotificationPubSubType();
   }
}
