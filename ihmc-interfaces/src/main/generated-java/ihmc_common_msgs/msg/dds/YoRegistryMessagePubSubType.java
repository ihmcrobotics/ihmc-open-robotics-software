package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YoRegistryMessage" defined in "YoRegistryMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YoRegistryMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YoRegistryMessage_.idl instead.
*
*/
public class YoRegistryMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.YoRegistryMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::YoRegistryMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "77138f9d6630e4e510a7e0196413176eb618c8e052ed18c2c3db7a77141f497b";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.YoRegistryMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.YoRegistryMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (10000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.YoRegistryMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.YoRegistryMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.YoRegistryMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getData().size() <= 10000)
      cdr.write_type_e(data.getData());else
          throw new RuntimeException("data field exceeds the maximum length: %d > %d".formatted(data.getData().size(), 10000));

   }

   public static void read(ihmc_common_msgs.msg.dds.YoRegistryMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getData());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.YoRegistryMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("data", data.getData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.YoRegistryMessage data)
   {
      ser.read_type_e("data", data.getData());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.YoRegistryMessage src, ihmc_common_msgs.msg.dds.YoRegistryMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.YoRegistryMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.YoRegistryMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.YoRegistryMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.YoRegistryMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.YoRegistryMessage src, ihmc_common_msgs.msg.dds.YoRegistryMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YoRegistryMessagePubSubType newInstance()
   {
      return new YoRegistryMessagePubSubType();
   }
}
