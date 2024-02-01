package ihmc_common_msgs.msg.dds;

/**
* 
* Topic data type of the struct "PoseListMessage" defined in "PoseListMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from PoseListMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit PoseListMessage_.idl instead.
*
*/
public class PoseListMessagePubSubType implements us.ihmc.pubsub.TopicDataType<ihmc_common_msgs.msg.dds.PoseListMessage>
{
   public static final java.lang.String name = "ihmc_common_msgs::msg::dds_::PoseListMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "bd64a0f475769c2c748f6cea9bb04ae5cf1a6022f1a031f4f9cc1f36ab2f06e0";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(ihmc_common_msgs.msg.dds.PoseListMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, ihmc_common_msgs.msg.dds.PoseListMessage data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 100; ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.PoseListMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(ihmc_common_msgs.msg.dds.PoseListMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getPoses().size(); ++i0)
      {
          current_alignment += geometry_msgs.msg.dds.PosePubSubType.getCdrSerializedSize(data.getPoses().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(ihmc_common_msgs.msg.dds.PoseListMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getPoses().size() <= 100)
      cdr.write_type_e(data.getPoses());else
          throw new RuntimeException("poses field exceeds the maximum length");

   }

   public static void read(ihmc_common_msgs.msg.dds.PoseListMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getPoses());	

   }

   @Override
   public final void serialize(ihmc_common_msgs.msg.dds.PoseListMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("poses", data.getPoses());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, ihmc_common_msgs.msg.dds.PoseListMessage data)
   {
      ser.read_type_e("poses", data.getPoses());
   }

   public static void staticCopy(ihmc_common_msgs.msg.dds.PoseListMessage src, ihmc_common_msgs.msg.dds.PoseListMessage dest)
   {
      dest.set(src);
   }

   @Override
   public ihmc_common_msgs.msg.dds.PoseListMessage createData()
   {
      return new ihmc_common_msgs.msg.dds.PoseListMessage();
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
   
   public void serialize(ihmc_common_msgs.msg.dds.PoseListMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(ihmc_common_msgs.msg.dds.PoseListMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(ihmc_common_msgs.msg.dds.PoseListMessage src, ihmc_common_msgs.msg.dds.PoseListMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public PoseListMessagePubSubType newInstance()
   {
      return new PoseListMessagePubSubType();
   }
}
