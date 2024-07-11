package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GlobalMapCellEntry" defined in "GlobalMapCellEntry_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GlobalMapCellEntry_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GlobalMapCellEntry_.idl instead.
*
*/
public class GlobalMapCellEntryPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.GlobalMapCellEntry>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::GlobalMapCellEntry_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3fb32caedd6b932601c21f35fd43795b6ef05b8e83f6ecfe932dca5f19b33f51";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.GlobalMapCellEntry data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.GlobalMapCellEntry data) throws java.io.IOException
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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapCellEntry data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapCellEntry data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.GlobalMapCellEntry data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_2(data.getKey());

      cdr.write_type_2(data.getXIndex());

      cdr.write_type_2(data.getYIndex());

      cdr.write_type_6(data.getCellHeight());

      cdr.write_type_6(data.getResolution());

   }

   public static void read(perception_msgs.msg.dds.GlobalMapCellEntry data, us.ihmc.idl.CDR cdr)
   {
      data.setKey(cdr.read_type_2());
      	
      data.setXIndex(cdr.read_type_2());
      	
      data.setYIndex(cdr.read_type_2());
      	
      data.setCellHeight(cdr.read_type_6());
      	
      data.setResolution(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.GlobalMapCellEntry data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_2("key", data.getKey());
      ser.write_type_2("x_index", data.getXIndex());
      ser.write_type_2("y_index", data.getYIndex());
      ser.write_type_6("cell_height", data.getCellHeight());
      ser.write_type_6("resolution", data.getResolution());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.GlobalMapCellEntry data)
   {
      data.setKey(ser.read_type_2("key"));
      data.setXIndex(ser.read_type_2("x_index"));
      data.setYIndex(ser.read_type_2("y_index"));
      data.setCellHeight(ser.read_type_6("cell_height"));
      data.setResolution(ser.read_type_6("resolution"));
   }

   public static void staticCopy(perception_msgs.msg.dds.GlobalMapCellEntry src, perception_msgs.msg.dds.GlobalMapCellEntry dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.GlobalMapCellEntry createData()
   {
      return new perception_msgs.msg.dds.GlobalMapCellEntry();
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
   
   public void serialize(perception_msgs.msg.dds.GlobalMapCellEntry data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.GlobalMapCellEntry data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.GlobalMapCellEntry src, perception_msgs.msg.dds.GlobalMapCellEntry dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GlobalMapCellEntryPubSubType newInstance()
   {
      return new GlobalMapCellEntryPubSubType();
   }
}
