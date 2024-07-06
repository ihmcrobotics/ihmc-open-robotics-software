package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "GlobalMapCellMap" defined in "GlobalMapCellMap_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from GlobalMapCellMap_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit GlobalMapCellMap_.idl instead.
*
*/
public class GlobalMapCellMapPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.GlobalMapCellMap>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::GlobalMapCellMap_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "3e2066fdf03fbcf8511c2c10f2fd954220eec7d2de23d5f34a090a69929c7671";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.GlobalMapCellMap data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.GlobalMapCellMap data) throws java.io.IOException
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
          current_alignment += perception_msgs.msg.dds.GlobalMapCellEntryPubSubType.getMaxCdrSerializedSize(current_alignment);}
      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapCellMap data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.GlobalMapCellMap data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getGlobalMapCells().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.GlobalMapCellEntryPubSubType.getCdrSerializedSize(data.getGlobalMapCells().get(i0), current_alignment);}

      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.GlobalMapCellMap data, us.ihmc.idl.CDR cdr)
   {
      if(data.getGlobalMapCells().size() <= 100)
      cdr.write_type_e(data.getGlobalMapCells());else
          throw new RuntimeException("global_map_cells field exceeds the maximum length");

   }

   public static void read(perception_msgs.msg.dds.GlobalMapCellMap data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_e(data.getGlobalMapCells());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.GlobalMapCellMap data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_e("global_map_cells", data.getGlobalMapCells());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.GlobalMapCellMap data)
   {
      ser.read_type_e("global_map_cells", data.getGlobalMapCells());
   }

   public static void staticCopy(perception_msgs.msg.dds.GlobalMapCellMap src, perception_msgs.msg.dds.GlobalMapCellMap dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.GlobalMapCellMap createData()
   {
      return new perception_msgs.msg.dds.GlobalMapCellMap();
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
   
   public void serialize(perception_msgs.msg.dds.GlobalMapCellMap data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.GlobalMapCellMap data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.GlobalMapCellMap src, perception_msgs.msg.dds.GlobalMapCellMap dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public GlobalMapCellMapPubSubType newInstance()
   {
      return new GlobalMapCellMapPubSubType();
   }
}
