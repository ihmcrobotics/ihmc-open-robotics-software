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
   		return "b33ee5e3531510aefbaca77e4e2ddb5c993f1d03a3f7a4851d2a1531a208c7fa";
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

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (500000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (500000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (250000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (250000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (250000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (250000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (250000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


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


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getHeightMapData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedHeightData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedNormalXData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedNormalYData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedNormalZData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSnappedAreaData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getSteppabilityData().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_3(data.getLocalGridSize());

      cdr.write_type_9(data.getCellsPerMeter());

      cdr.write_type_6(data.getMapCenterX());

      cdr.write_type_6(data.getMapCenterY());

      cdr.write_type_6(data.getHeightScaleFactor());

      cdr.write_type_6(data.getHeightScaleOffset());

      cdr.write_type_7(data.getHasHeightMapData());

      cdr.write_type_7(data.getHasSnappedHeightData());

      cdr.write_type_7(data.getHasSnappedNormalData());

      cdr.write_type_7(data.getHasSnappedAreaData());

      cdr.write_type_7(data.getHasSteppabilityData());

      if(data.getHeightMapData().size() <= 500000)
      cdr.write_type_e(data.getHeightMapData());else
          throw new RuntimeException("height_map_data field exceeds the maximum length: %d > %d".formatted(data.getHeightMapData().size(), 500000));

      if(data.getSnappedHeightData().size() <= 500000)
      cdr.write_type_e(data.getSnappedHeightData());else
          throw new RuntimeException("snapped_height_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedHeightData().size(), 500000));

      if(data.getSnappedNormalXData().size() <= 250000)
      cdr.write_type_e(data.getSnappedNormalXData());else
          throw new RuntimeException("snapped_normal_x_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedNormalXData().size(), 250000));

      if(data.getSnappedNormalYData().size() <= 250000)
      cdr.write_type_e(data.getSnappedNormalYData());else
          throw new RuntimeException("snapped_normal_y_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedNormalYData().size(), 250000));

      if(data.getSnappedNormalZData().size() <= 250000)
      cdr.write_type_e(data.getSnappedNormalZData());else
          throw new RuntimeException("snapped_normal_z_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedNormalZData().size(), 250000));

      if(data.getSnappedAreaData().size() <= 250000)
      cdr.write_type_e(data.getSnappedAreaData());else
          throw new RuntimeException("snapped_area_data field exceeds the maximum length: %d > %d".formatted(data.getSnappedAreaData().size(), 250000));

      if(data.getSteppabilityData().size() <= 250000)
      cdr.write_type_e(data.getSteppabilityData());else
          throw new RuntimeException("steppability_data field exceeds the maximum length: %d > %d".formatted(data.getSteppabilityData().size(), 250000));

   }

   public static void read(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setLocalGridSize(cdr.read_type_3());
      	
      data.setCellsPerMeter(cdr.read_type_9());
      	
      data.setMapCenterX(cdr.read_type_6());
      	
      data.setMapCenterY(cdr.read_type_6());
      	
      data.setHeightScaleFactor(cdr.read_type_6());
      	
      data.setHeightScaleOffset(cdr.read_type_6());
      	
      data.setHasHeightMapData(cdr.read_type_7());
      	
      data.setHasSnappedHeightData(cdr.read_type_7());
      	
      data.setHasSnappedNormalData(cdr.read_type_7());
      	
      data.setHasSnappedAreaData(cdr.read_type_7());
      	
      data.setHasSteppabilityData(cdr.read_type_7());
      	
      cdr.read_type_e(data.getHeightMapData());	
      cdr.read_type_e(data.getSnappedHeightData());	
      cdr.read_type_e(data.getSnappedNormalXData());	
      cdr.read_type_e(data.getSnappedNormalYData());	
      cdr.read_type_e(data.getSnappedNormalZData());	
      cdr.read_type_e(data.getSnappedAreaData());	
      cdr.read_type_e(data.getSteppabilityData());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.TerrainMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_3("local_grid_size", data.getLocalGridSize());
      ser.write_type_9("cells_per_meter", data.getCellsPerMeter());
      ser.write_type_6("map_center_x", data.getMapCenterX());
      ser.write_type_6("map_center_y", data.getMapCenterY());
      ser.write_type_6("height_scale_factor", data.getHeightScaleFactor());
      ser.write_type_6("height_scale_offset", data.getHeightScaleOffset());
      ser.write_type_7("has_height_map_data", data.getHasHeightMapData());
      ser.write_type_7("has_snapped_height_data", data.getHasSnappedHeightData());
      ser.write_type_7("has_snapped_normal_data", data.getHasSnappedNormalData());
      ser.write_type_7("has_snapped_area_data", data.getHasSnappedAreaData());
      ser.write_type_7("has_steppability_data", data.getHasSteppabilityData());
      ser.write_type_e("height_map_data", data.getHeightMapData());
      ser.write_type_e("snapped_height_data", data.getSnappedHeightData());
      ser.write_type_e("snapped_normal_x_data", data.getSnappedNormalXData());
      ser.write_type_e("snapped_normal_y_data", data.getSnappedNormalYData());
      ser.write_type_e("snapped_normal_z_data", data.getSnappedNormalZData());
      ser.write_type_e("snapped_area_data", data.getSnappedAreaData());
      ser.write_type_e("steppability_data", data.getSteppabilityData());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.TerrainMapMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setLocalGridSize(ser.read_type_3("local_grid_size"));
      data.setCellsPerMeter(ser.read_type_9("cells_per_meter"));
      data.setMapCenterX(ser.read_type_6("map_center_x"));
      data.setMapCenterY(ser.read_type_6("map_center_y"));
      data.setHeightScaleFactor(ser.read_type_6("height_scale_factor"));
      data.setHeightScaleOffset(ser.read_type_6("height_scale_offset"));
      data.setHasHeightMapData(ser.read_type_7("has_height_map_data"));
      data.setHasSnappedHeightData(ser.read_type_7("has_snapped_height_data"));
      data.setHasSnappedNormalData(ser.read_type_7("has_snapped_normal_data"));
      data.setHasSnappedAreaData(ser.read_type_7("has_snapped_area_data"));
      data.setHasSteppabilityData(ser.read_type_7("has_steppability_data"));
      ser.read_type_e("height_map_data", data.getHeightMapData());
      ser.read_type_e("snapped_height_data", data.getSnappedHeightData());
      ser.read_type_e("snapped_normal_x_data", data.getSnappedNormalXData());
      ser.read_type_e("snapped_normal_y_data", data.getSnappedNormalYData());
      ser.read_type_e("snapped_normal_z_data", data.getSnappedNormalZData());
      ser.read_type_e("snapped_area_data", data.getSnappedAreaData());
      ser.read_type_e("steppability_data", data.getSteppabilityData());
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
