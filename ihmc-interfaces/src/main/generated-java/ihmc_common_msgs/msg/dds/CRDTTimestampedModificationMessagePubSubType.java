package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "CRDTTimestampedModificationMessage" defined in "CRDTTimestampedModificationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from CRDTTimestampedModificationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit CRDTTimestampedModificationMessage_.idl instead.
*
*/
public class CRDTTimestampedModificationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::CRDTTimestampedModificationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "2440a68c78797577df657364e054562c9a6cba58a7e127916133055492cbd4cf";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.GuidMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.GuidMessagePubSubType.getCdrSerializedSize(data.getLatestModifierId(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getLatestModificationTimeInModifierFrame(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.GuidMessagePubSubType.write(data.getLatestModifierId(), cdr);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getLatestModificationTimeInModifierFrame(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.GuidMessagePubSubType.read(data.getLatestModifierId(), cdr);	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getLatestModificationTimeInModifierFrame(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_modifier_id", new ihmc_common_msgs.msg.dds.GuidMessagePubSubType(), data.getLatestModifierId());

      ser.write_type_a("latest_modification_time_in_modifier_frame", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLatestModificationTimeInModifierFrame());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data)
   {
      ser.read_type_a("latest_modifier_id", new ihmc_common_msgs.msg.dds.GuidMessagePubSubType(), data.getLatestModifierId());

      ser.read_type_a("latest_modification_time_in_modifier_frame", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLatestModificationTimeInModifierFrame());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage src, ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage src, ihmc_common_msgs.msg.dds.CRDTTimestampedModificationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public CRDTTimestampedModificationMessagePubSubType newInstance()
   {
      return new CRDTTimestampedModificationMessagePubSubType();
   }
}
