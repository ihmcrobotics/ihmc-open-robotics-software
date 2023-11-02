package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ConfirmableRequestMessage" defined in "ConfirmableRequestMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ConfirmableRequestMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ConfirmableRequestMessage_.idl instead.
*
*/
public class ConfirmableRequestMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.ConfirmableRequestMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::ConfirmableRequestMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5161e392560e9abe322d7ae1871b835afa721e9b6cbb22ec3dced0d553e3ce44";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data) throws java.io.IOException
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.getCdrSerializedSize(data.getRequestUuid(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_3(data.getValue());

      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.write(data.getRequestUuid(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setValue(cdr.read_type_3());
      	
      ihmc_common_msgs.msg.dds.UUIDMessagePubSubType.read(data.getRequestUuid(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_3("value", data.getValue());
      ser.write_type_a("request_uuid", new ihmc_common_msgs.msg.dds.UUIDMessagePubSubType(), data.getRequestUuid());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data)
   {
      data.setValue(ser.read_type_3("value"));
      ser.read_type_a("request_uuid", new ihmc_common_msgs.msg.dds.UUIDMessagePubSubType(), data.getRequestUuid());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage src, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.ConfirmableRequestMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.ConfirmableRequestMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.ConfirmableRequestMessage src, ihmc_common_msgs.msg.dds.ConfirmableRequestMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ConfirmableRequestMessagePubSubType newInstance()
   {
      return new ConfirmableRequestMessagePubSubType();
   }
}
