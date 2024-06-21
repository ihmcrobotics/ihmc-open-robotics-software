package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "DoorPanelMessage" defined in "DoorPanelMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from DoorPanelMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit DoorPanelMessage_.idl instead.
*
*/
public class DoorPanelMessagePubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.DoorPanelMessage>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::DoorPanelMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "aea9fbf35a91c6ab5d2217f853b0b76a5a5407e62caaf5e5fbb56f4a45ae3eb5";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.DoorPanelMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.DoorPanelMessage data) throws java.io.IOException
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

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorPanelMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.DoorPanelMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += perception_msgs.msg.dds.PlanarRegionMessagePubSubType.getCdrSerializedSize(data.getPlanarRegion(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.DoorPanelMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.write(data.getPlanarRegion(), cdr);
      cdr.write_type_12(data.getPlanarRegionLastUpdateTimeMillis());

   }

   public static void read(perception_msgs.msg.dds.DoorPanelMessage data, us.ihmc.idl.CDR cdr)
   {
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.read(data.getPlanarRegion(), cdr);	
      data.setPlanarRegionLastUpdateTimeMillis(cdr.read_type_12());
      	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.DoorPanelMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getPlanarRegion());

      ser.write_type_12("planar_region_last_update_time_millis", data.getPlanarRegionLastUpdateTimeMillis());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.DoorPanelMessage data)
   {
      ser.read_type_a("planar_region", new perception_msgs.msg.dds.PlanarRegionMessagePubSubType(), data.getPlanarRegion());

      data.setPlanarRegionLastUpdateTimeMillis(ser.read_type_12("planar_region_last_update_time_millis"));
   }

   public static void staticCopy(perception_msgs.msg.dds.DoorPanelMessage src, perception_msgs.msg.dds.DoorPanelMessage dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.DoorPanelMessage createData()
   {
      return new perception_msgs.msg.dds.DoorPanelMessage();
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
   
   public void serialize(perception_msgs.msg.dds.DoorPanelMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.DoorPanelMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.DoorPanelMessage src, perception_msgs.msg.dds.DoorPanelMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public DoorPanelMessagePubSubType newInstance()
   {
      return new DoorPanelMessagePubSubType();
   }
}
