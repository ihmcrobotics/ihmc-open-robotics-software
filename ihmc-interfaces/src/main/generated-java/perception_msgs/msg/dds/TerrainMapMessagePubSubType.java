package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "TerrainMapMessage" defined in "TerrainMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from TerrainMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit TerrainMapMessage_.idl instead.
*
*/
public class TerrainMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.TerrainMapMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::TerrainMapMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "92a21ed088aa2fbd828e2b9afe4f8ae27c71ec75d0941b4ed7b79e430e0adcc7";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.TerrainMapMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (255000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (255000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (255000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (255000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (255000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.TerrainMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.TerrainMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += perception_msgs.msg.dds.HeightMapMessagePubSubType.getCdrSerializedSize(data.getHeightMap(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getTraversabilityScore().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getTraversabilityClass().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedNormalXData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedNormalYData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedNormalZData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      perception_msgs.msg.dds.HeightMapMessagePubSubType.write(data.getHeightMap(), cdr);
      if(data.getTraversabilityScore().size() <= 255000)
      cdr.write_type_e(data.getTraversabilityScore());else
          throw new RuntimeException("traversability_score field exceeds the maximum length: %d > %d".formatted(data.getTraversabilityScore().size(), 255000));

      if(data.getTraversabilityClass().size() <= 255000)
      cdr.write_type_e(data.getTraversabilityClass());else
          throw new RuntimeException("traversability_class field exceeds the maximum length: %d > %d".formatted(data.getTraversabilityClass().size(), 255000));

      if(data.getSnappedNormalXData().size() <= 255000)
      cdr.write_type_e(data.getSnappedNormalXData());else
          throw new RuntimeException("snapped_normal_x_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedNormalXData().size(), 255000));

      if(data.getSnappedNormalYData().size() <= 255000)
      cdr.write_type_e(data.getSnappedNormalYData());else
          throw new RuntimeException("snapped_normal_y_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedNormalYData().size(), 255000));

      if(data.getSnappedNormalZData().size() <= 255000)
      cdr.write_type_e(data.getSnappedNormalZData());else
          throw new RuntimeException("snapped_normal_z_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedNormalZData().size(), 255000));

   }

   public static void read(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      perception_msgs.msg.dds.HeightMapMessagePubSubType.read(data.getHeightMap(), cdr);	
      cdr.read_type_e(data.getTraversabilityScore());	
      cdr.read_type_e(data.getTraversabilityClass());	
      cdr.read_type_e(data.getSnappedNormalXData());	
      cdr.read_type_e(data.getSnappedNormalYData());	
      cdr.read_type_e(data.getSnappedNormalZData());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_a("height_map", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMap());

      ser.write_type_e("traversability_score", data.getTraversabilityScore());
      ser.write_type_e("traversability_class", data.getTraversabilityClass());
      ser.write_type_e("snapped_normal_x_data", data.getSnappedNormalXData());
      ser.write_type_e("snapped_normal_y_data", data.getSnappedNormalYData());
      ser.write_type_e("snapped_normal_z_data", data.getSnappedNormalZData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.TerrainMapMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      ser.read_type_a("height_map", new perception_msgs.msg.dds.HeightMapMessagePubSubType(), data.getHeightMap());

      ser.read_type_e("traversability_score", data.getTraversabilityScore());
      ser.read_type_e("traversability_class", data.getTraversabilityClass());
      ser.read_type_e("snapped_normal_x_data", data.getSnappedNormalXData());
      ser.read_type_e("snapped_normal_y_data", data.getSnappedNormalYData());
      ser.read_type_e("snapped_normal_z_data", data.getSnappedNormalZData());
   }

   public static void staticCopy(perception_msgs.msg.dds.TerrainMapMessage src, perception_msgs.msg.dds.TerrainMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.TerrainMapMessage createData()
   {
      return new perception_msgs.msg.dds.TerrainMapMessage();
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
   
   public void serialize(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.TerrainMapMessage src, perception_msgs.msg.dds.TerrainMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public TerrainMapMessagePubSubType newInstance()
   {
      return new TerrainMapMessagePubSubType();
   }
}
