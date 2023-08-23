package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MessageCollection" defined in "MessageCollection_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MessageCollection_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MessageCollection_.idl instead.
*
*/
public class MessageCollectionPubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.MessageCollection>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::MessageCollection_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "539ba3bd7996534a788a9a164e3b2bc1cfa847c4fae7ffcef5184551f8c71dec";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.MessageCollection data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.MessageCollection data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.MessageCollection data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.MessageCollection data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSequences().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.MessageCollection data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      if(data.getSequences().size() <= 100)
      cdr.write_type_e(data.getSequences());else
          throw new RuntimeException("sequences field exceeds the maximum length");

   }

   public static void read(ihmc_common_msgs.msg.dds.MessageCollection data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      cdr.read_type_e(data.getSequences());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.MessageCollection data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_e("sequences", data.getSequences());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.MessageCollection data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      ser.read_type_e("sequences", data.getSequences());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.MessageCollection src, ihmc_common_msgs.msg.dds.MessageCollection dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.MessageCollection createData()
   {
      return new ihmc_common_msgs.msg.dds.MessageCollection();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.MessageCollection data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.MessageCollection data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.MessageCollection src, ihmc_common_msgs.msg.dds.MessageCollection dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MessageCollectionPubSubType newInstance()
   {
      return new MessageCollectionPubSubType();
   }
}
