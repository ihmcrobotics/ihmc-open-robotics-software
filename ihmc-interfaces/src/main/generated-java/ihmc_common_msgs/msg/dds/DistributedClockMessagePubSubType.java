package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DistributedClockMessage" defined in "DistributedClockMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DistributedClockMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DistributedClockMessage_.idl instead.
*
*/
public class DistributedClockMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.DistributedClockMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::DistributedClockMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "9b29df5decb710752845fe1e9451103d00a8c5892b8dadd4a3a7998bea11974a";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.DistributedClockMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.DistributedClockMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.DistributedClockMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.DistributedClockMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getRequesterId().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getReplierId().length() + 1;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getRequestSendTime(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getReplySendTime(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.DistributedClockMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getRequesterId().length() <= 255)
      cdr.write_type_d(data.getRequesterId());else
          throw new RuntimeException("requester_id field exceeds the maximum length");

      if(data.getReplierId().length() <= 255)
      cdr.write_type_d(data.getReplierId());else
          throw new RuntimeException("replier_id field exceeds the maximum length");

      cdr.write_type_12(data.getRequestNumber());

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getRequestSendTime(), cdr);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getReplySendTime(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.DistributedClockMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getRequesterId());	
      cdr.read_type_d(data.getReplierId());	
      data.setRequestNumber(cdr.read_type_12());
      	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getRequestSendTime(), cdr);	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getReplySendTime(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.DistributedClockMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("requester_id", data.getRequesterId());
      ser.write_type_d("replier_id", data.getReplierId());
      ser.write_type_12("request_number", data.getRequestNumber());
      ser.write_type_a("request_send_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getRequestSendTime());

      ser.write_type_a("reply_send_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getReplySendTime());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.DistributedClockMessage data)
   {
      ser.read_type_d("requester_id", data.getRequesterId());
      ser.read_type_d("replier_id", data.getReplierId());
      data.setRequestNumber(ser.read_type_12("request_number"));
      ser.read_type_a("request_send_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getRequestSendTime());

      ser.read_type_a("reply_send_time", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getReplySendTime());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.DistributedClockMessage src, ihmc_common_msgs.msg.dds.DistributedClockMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.DistributedClockMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.DistributedClockMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.DistributedClockMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.DistributedClockMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.DistributedClockMessage src, ihmc_common_msgs.msg.dds.DistributedClockMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DistributedClockMessagePubSubType newInstance()
   {
      return new DistributedClockMessagePubSubType();
   }
}
