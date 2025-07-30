package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ChunkMessage" defined in "ChunkMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ChunkMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ChunkMessage_.idl instead.
*
*/
public class ChunkMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.ChunkMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::ChunkMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "aa62a795e7bcc1bfea7779c3d8844f8e91aab03483baa94ba3f85ee670147398";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.ChunkMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ChunkMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ChunkMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getCdrSerializedSize(data.getChunk(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getCenterX());

      cdr.write_type_6(data.getCenterY());

      cdr.write_type_2(data.getHashCodeOfChunk());

      perception_msgs.msg.dds.HeightMapMessagePubSubType.write(data.getChunk(), cdr);
   }

   public static void read(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setCenterX(cdr.read_type_6());
      	
      data.setCenterY(cdr.read_type_6());
      	
      data.setHashCodeOfChunk(cdr.read_type_2());
      	
      perception_msgs.msg.dds.HeightMapMessagePubSubType.read(data.getChunk(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("center_x", data.getCenterX());
      ser.write_type_6("center_y", data.getCenterY());
      ser.write_type_2("hash_code_of_chunk", data.getHashCodeOfChunk());
      ser.write_type_a("chunk", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getChunk());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ChunkMessage data)
   {
      data.setCenterX(ser.read_type_6("center_x"));
      data.setCenterY(ser.read_type_6("center_y"));
      data.setHashCodeOfChunk(ser.read_type_2("hash_code_of_chunk"));
      ser.read_type_a("chunk", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getChunk());

   }

   public static void staticCopy(perception_msgs.msg.dds.ChunkMessage src, perception_msgs.msg.dds.ChunkMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.ChunkMessage createData()
   {
      return new perception_msgs.msg.dds.ChunkMessage();
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
   
   public void serialize(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.ChunkMessage src, perception_msgs.msg.dds.ChunkMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ChunkMessagePubSubType newInstance()
   {
      return new ChunkMessagePubSubType();
   }
}
