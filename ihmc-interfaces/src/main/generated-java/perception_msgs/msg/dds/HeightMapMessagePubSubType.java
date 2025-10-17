package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightMapMessage" defined in "HeightMapMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightMapMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightMapMessage_.idl instead.
*
*/
public class HeightMapMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.HeightMapMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::HeightMapMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "055fbac97c9992f7d1b678998153a4828d70e8d8d4e0d538159d4a4397237b13";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.HeightMapMessage data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (255000 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getHeights().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_12(data.getSequenceId());

      cdr.write_type_6(data.getGridCenterX());

      cdr.write_type_6(data.getGridCenterY());

      cdr.write_type_6(data.getWidthInMeters());

      cdr.write_type_6(data.getCellSizeInMeters());

      cdr.write_type_2(data.getCellsPerAxis());

      if(data.getHeights().size() <= 255000)
      cdr.write_type_e(data.getHeights());else
          throw new RuntimeException("heights field exceeds the maximum length: %d > %d".formatted(data.getHeights().size(), 255000));

   }

   public static void read(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_12());
      	
      data.setGridCenterX(cdr.read_type_6());
      	
      data.setGridCenterY(cdr.read_type_6());
      	
      data.setWidthInMeters(cdr.read_type_6());
      	
      data.setCellSizeInMeters(cdr.read_type_6());
      	
      data.setCellsPerAxis(cdr.read_type_2());
      	
      cdr.read_type_e(data.getHeights());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_12("sequence_id", data.getSequenceId());
      ser.write_type_6("grid_center_x", data.getGridCenterX());
      ser.write_type_6("grid_center_y", data.getGridCenterY());
      ser.write_type_6("width_in_meters", data.getWidthInMeters());
      ser.write_type_6("cell_size_in_meters", data.getCellSizeInMeters());
      ser.write_type_2("cells_per_axis", data.getCellsPerAxis());
      ser.write_type_e("heights", data.getHeights());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.HeightMapMessage data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setGridCenterX(ser.read_type_6("grid_center_x"));
      data.setGridCenterY(ser.read_type_6("grid_center_y"));
      data.setWidthInMeters(ser.read_type_6("width_in_meters"));
      data.setCellSizeInMeters(ser.read_type_6("cell_size_in_meters"));
      data.setCellsPerAxis(ser.read_type_2("cells_per_axis"));
      ser.read_type_e("heights", data.getHeights());
   }

   public static void staticCopy(perception_msgs.msg.dds.HeightMapMessage src, perception_msgs.msg.dds.HeightMapMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.HeightMapMessage createData()
   {
      return new perception_msgs.msg.dds.HeightMapMessage();
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
   
   public void serialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.HeightMapMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.HeightMapMessage src, perception_msgs.msg.dds.HeightMapMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightMapMessagePubSubType newInstance()
   {
      return new HeightMapMessagePubSubType();
   }
}
