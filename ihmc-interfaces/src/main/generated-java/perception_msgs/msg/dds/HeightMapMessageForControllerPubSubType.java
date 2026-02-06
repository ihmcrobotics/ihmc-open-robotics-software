package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "HeightMapMessageForController" defined in "HeightMapMessageForController_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from HeightMapMessageForController_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit HeightMapMessageForController_.idl instead.
*
*/
public class HeightMapMessageForControllerPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.HeightMapMessageForController>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::HeightMapMessageForController_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b2531291e40f39187c68a9a80e67a9ecae2ffefc7955cc9fd9ed62a79b091073";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.HeightMapMessageForController data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.HeightMapMessageForController data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (255000 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapMessageForController data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.HeightMapMessageForController data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getHeights().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.HeightMapMessageForController data, us.ihmc.idl.CDR cdr)
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

   public static void read(perception_msgs.msg.dds.HeightMapMessageForController data, us.ihmc.idl.CDR cdr)
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
   public final void serialize(perception_msgs.msg.dds.HeightMapMessageForController data, us.ihmc.idl.InterchangeSerializer ser)
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
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.HeightMapMessageForController data)
   {
      data.setSequenceId(ser.read_type_12("sequence_id"));
      data.setGridCenterX(ser.read_type_6("grid_center_x"));
      data.setGridCenterY(ser.read_type_6("grid_center_y"));
      data.setWidthInMeters(ser.read_type_6("width_in_meters"));
      data.setCellSizeInMeters(ser.read_type_6("cell_size_in_meters"));
      data.setCellsPerAxis(ser.read_type_2("cells_per_axis"));
      ser.read_type_e("heights", data.getHeights());
   }

   public static void staticCopy(perception_msgs.msg.dds.HeightMapMessageForController src, perception_msgs.msg.dds.HeightMapMessageForController dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.HeightMapMessageForController createData()
   {
      return new perception_msgs.msg.dds.HeightMapMessageForController();
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
   
   public void serialize(perception_msgs.msg.dds.HeightMapMessageForController data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.HeightMapMessageForController data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.HeightMapMessageForController src, perception_msgs.msg.dds.HeightMapMessageForController dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public HeightMapMessageForControllerPubSubType newInstance()
   {
      return new HeightMapMessageForControllerPubSubType();
   }
}
