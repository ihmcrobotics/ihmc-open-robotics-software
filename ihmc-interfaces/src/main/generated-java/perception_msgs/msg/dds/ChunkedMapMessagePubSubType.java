package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ChunkedMapMessage" defined in "ChunkedMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ChunkedMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ChunkedMapMessage_.idl instead.
*
*/
public class ChunkedMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.ChunkedMapMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ChunkedMapMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fd8508fd846e56c1ae1e56254200e1ac5b2797976a80c16d4d50620d76fdb4d7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.ChunkedMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.ChunkedMapMessage data) throws java.io.IOException
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
          current_alignment += perception_msgs.msg.dds.ChunkMessagePubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ChunkedMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ChunkedMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getChunks().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.ChunkMessagePubSubType.getCdrSerializedSize(data.getChunks().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ChunkedMapMessage data, us.ihmc.idl.CDR cdr)
   {
      if(data.getChunks().size() <= 100)
      cdr.write_type_e(data.getChunks());else
          throw new RuntimeException("chunks field exceeds the maximum length: %d > %d".formatted(data.getChunks().size(), 100));

   }

   public static void read(perception_msgs.msg.dds.ChunkedMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getChunks());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ChunkedMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("chunks", data.getChunks());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ChunkedMapMessage data)
   {
      ser.read_type_e("chunks", data.getChunks());
   }

   public static void staticCopy(perception_msgs.msg.dds.ChunkedMapMessage src, perception_msgs.msg.dds.ChunkedMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.ChunkedMapMessage createData()
   {
      return new perception_msgs.msg.dds.ChunkedMapMessage();
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
   
   public void serialize(perception_msgs.msg.dds.ChunkedMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.ChunkedMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.ChunkedMapMessage src, perception_msgs.msg.dds.ChunkedMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ChunkedMapMessagePubSubType newInstance()
   {
      return new ChunkedMapMessagePubSubType();
   }
}
