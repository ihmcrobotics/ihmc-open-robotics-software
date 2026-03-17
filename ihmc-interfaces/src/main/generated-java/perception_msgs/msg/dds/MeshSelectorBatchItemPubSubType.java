package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "MeshSelectorBatchItem" defined in "MeshSelectorBatchItem_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from MeshSelectorBatchItem_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit MeshSelectorBatchItem_.idl instead.
*
*/
public class MeshSelectorBatchItemPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.MeshSelectorBatchItem>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::MeshSelectorBatchItem_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "5c5f857fef5a775b7698d88216aaa3735f3e75e042117b3330d34fd1dfcf440d";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.MeshSelectorBatchItem data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.MeshSelectorBatchItem data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getMaxCdrSerializedSize(current_alignment);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.MeshSelectorBatchItem data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.MeshSelectorBatchItem data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getCategory().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += sensor_msgs.msg.dds.ImagePubSubType.getCdrSerializedSize(data.getSegmentedRgb(), current_alignment);


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.MeshSelectorBatchItem data, us.ihmc.idl.CDR cdr)
   {
      if(data.getCategory().length() <= 255)
      cdr.write_type_d(data.getCategory());else
          throw new RuntimeException("category field exceeds the maximum length: %d > %d".formatted(data.getCategory().length(), 255));

      cdr.write_type_2(data.getTrackId());

      sensor_msgs.msg.dds.ImagePubSubType.write(data.getSegmentedRgb(), cdr);
   }

   public static void read(perception_msgs.msg.dds.MeshSelectorBatchItem data, us.ihmc.idl.CDR cdr)
   {
      cdr.read_type_d(data.getCategory());	
      data.setTrackId(cdr.read_type_2());
      	
      sensor_msgs.msg.dds.ImagePubSubType.read(data.getSegmentedRgb(), cdr);	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.MeshSelectorBatchItem data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_d("category", data.getCategory());
      ser.write_type_2("track_id", data.getTrackId());
      ser.write_type_a("segmented_rgb", new sensor_msgs.msg.dds.ImagePubSubType(), data.getSegmentedRgb());

   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.MeshSelectorBatchItem data)
   {
      ser.read_type_d("category", data.getCategory());
      data.setTrackId(ser.read_type_2("track_id"));
      ser.read_type_a("segmented_rgb", new sensor_msgs.msg.dds.ImagePubSubType(), data.getSegmentedRgb());

   }

   public static void staticCopy(perception_msgs.msg.dds.MeshSelectorBatchItem src, perception_msgs.msg.dds.MeshSelectorBatchItem dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.MeshSelectorBatchItem createData()
   {
      return new perception_msgs.msg.dds.MeshSelectorBatchItem();
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
   
   public void serialize(perception_msgs.msg.dds.MeshSelectorBatchItem data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.MeshSelectorBatchItem data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.MeshSelectorBatchItem src, perception_msgs.msg.dds.MeshSelectorBatchItem dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public MeshSelectorBatchItemPubSubType newInstance()
   {
      return new MeshSelectorBatchItemPubSubType();
   }
}
