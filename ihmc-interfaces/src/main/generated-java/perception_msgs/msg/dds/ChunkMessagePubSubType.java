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
   		return "23e6508d40724b22876e1d9338bda36450eab672784c593401655f7130170ed6";
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (20000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ChunkMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.ChunkMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getHeights().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getHashCodeOfChunk());

      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_5(data.getOriginX());

      cdr.write_type_5(data.getOriginY());

      cdr.write_type_5(data.getWidthInMeters());

      cdr.write_type_5(data.getCellSizeInMeters());

      cdr.write_type_2(data.getCellsPerAxis());

      if(data.getHeights().size() <= 20000)
      cdr.write_type_e(data.getHeights());else
          throw new RuntimeException("heights field exceeds the maximum length: %d > %d".formatted(data.getHeights().size(), 20000));

   }

   public static void read(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setHashCodeOfChunk(cdr.read_type_2());
      	
      data.setSequenceId(cdr.read_type_4());
      	
      data.setOriginX(cdr.read_type_5());
      	
      data.setOriginY(cdr.read_type_5());
      	
      data.setWidthInMeters(cdr.read_type_5());
      	
      data.setCellSizeInMeters(cdr.read_type_5());
      	
      data.setCellsPerAxis(cdr.read_type_2());
      	
      cdr.read_type_e(data.getHeights());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.ChunkMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("hash_code_of_chunk", data.getHashCodeOfChunk());
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_5("origin_x", data.getOriginX());
      ser.write_type_5("origin_y", data.getOriginY());
      ser.write_type_5("width_in_meters", data.getWidthInMeters());
      ser.write_type_5("cell_size_in_meters", data.getCellSizeInMeters());
      ser.write_type_2("cells_per_axis", data.getCellsPerAxis());
      ser.write_type_e("heights", data.getHeights());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.ChunkMessage data)
   {
      data.setHashCodeOfChunk(ser.read_type_2("hash_code_of_chunk"));
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setOriginX(ser.read_type_5("origin_x"));
      data.setOriginY(ser.read_type_5("origin_y"));
      data.setWidthInMeters(ser.read_type_5("width_in_meters"));
      data.setCellSizeInMeters(ser.read_type_5("cell_size_in_meters"));
      data.setCellsPerAxis(ser.read_type_2("cells_per_axis"));
      ser.read_type_e("heights", data.getHeights());
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
