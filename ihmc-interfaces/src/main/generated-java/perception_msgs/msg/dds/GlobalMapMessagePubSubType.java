package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GlobalMapMessage" defined in "GlobalMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GlobalMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GlobalMapMessage_.idl instead.
*
*/
public class GlobalMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.GlobalMapMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::GlobalMapMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "c9b617108b1e683322464951d9d4c6e02d906eb1f353d68cecb0135057a93ae2";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.GlobalMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.GlobalMapMessage data) throws java.io.IOException
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
          current_alignment += perception_msgs.msg.dds.GlobalMapTileMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getGlobalMap().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.GlobalMapTileMessagePubSubType.getCdrSerializedSize(data.getGlobalMap().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.GlobalMapMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getGlobalMap().size() <= 100)
      cdr.write_type_e(data.getGlobalMap());else
          throw new RuntimeException("global_map field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.GlobalMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getGlobalMap());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.GlobalMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("global_map", data.getGlobalMap());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.GlobalMapMessage data)
   {
      ser.read_type_e("global_map", data.getGlobalMap());
   }

   public static void staticCopy(perception_msgs.msg.dds.GlobalMapMessage src, perception_msgs.msg.dds.GlobalMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.GlobalMapMessage createData()
   {
      return new perception_msgs.msg.dds.GlobalMapMessage();
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
   
   public void serialize(perception_msgs.msg.dds.GlobalMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.GlobalMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.GlobalMapMessage src, perception_msgs.msg.dds.GlobalMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GlobalMapMessagePubSubType newInstance()
   {
      return new GlobalMapMessagePubSubType();
   }
}
