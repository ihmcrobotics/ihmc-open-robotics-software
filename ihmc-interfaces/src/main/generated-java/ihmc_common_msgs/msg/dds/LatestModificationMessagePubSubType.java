package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "LatestModificationMessage" defined in "LatestModificationMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from LatestModificationMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit LatestModificationMessage_.idl instead.
*
*/
public class LatestModificationMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.LatestModificationMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::LatestModificationMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "0db59189fea8bd2f9c5832e08d1c1fb14054dc774d8f9543d9610a2f554219cf";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.LatestModificationMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.LatestModificationMessage data) throws java.io.IOException
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

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.LatestModificationMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.LatestModificationMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.GuidMessagePubSubType.getCdrSerializedSize(data.getLatestModifierId(), current_alignment);

      current_alignment += ihmc_common_msgs.msg.dds.InstantMessagePubSubType.getCdrSerializedSize(data.getLatestModificationTimeInModifierFrame(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.LatestModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.GuidMessagePubSubType.write(data.getLatestModifierId(), cdr);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.write(data.getLatestModificationTimeInModifierFrame(), cdr);
   }

   public static void read(ihmc_common_msgs.msg.dds.LatestModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.GuidMessagePubSubType.read(data.getLatestModifierId(), cdr);	
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.read(data.getLatestModificationTimeInModifierFrame(), cdr);	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.LatestModificationMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_modifier_id", new ihmc_common_msgs.msg.dds.GuidMessagePubSubType(), data.getLatestModifierId());

      ser.write_type_a("latest_modification_time_in_modifier_frame", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLatestModificationTimeInModifierFrame());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.LatestModificationMessage data)
   {
      ser.read_type_a("latest_modifier_id", new ihmc_common_msgs.msg.dds.GuidMessagePubSubType(), data.getLatestModifierId());

      ser.read_type_a("latest_modification_time_in_modifier_frame", new ihmc_common_msgs.msg.dds.InstantMessagePubSubType(), data.getLatestModificationTimeInModifierFrame());

   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.LatestModificationMessage src, ihmc_common_msgs.msg.dds.LatestModificationMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.LatestModificationMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.LatestModificationMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.LatestModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.LatestModificationMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.LatestModificationMessage src, ihmc_common_msgs.msg.dds.LatestModificationMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public LatestModificationMessagePubSubType newInstance()
   {
      return new LatestModificationMessagePubSubType();
   }
}
