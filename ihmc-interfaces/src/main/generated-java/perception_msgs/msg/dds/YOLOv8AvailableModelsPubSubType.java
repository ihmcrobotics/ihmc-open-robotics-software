package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8AvailableModels" defined in "YOLOv8AvailableModels_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8AvailableModels_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8AvailableModels_.idl instead.
*
*/
public class YOLOv8AvailableModelsPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8AvailableModels>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8AvailableModels_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "b91d143b621093319bc35937a6d6374ca1283278d0c2754acf124d46e83a3f75";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8AvailableModels data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8AvailableModels data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 8; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8ModelInfoPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8AvailableModels data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8AvailableModels data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAvailableYoloModels().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8ModelInfoPubSubType.getCdrSerializedSize(data.getAvailableYoloModels().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8AvailableModels data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_7(data.getRequest());

      if(data.getAvailableYoloModels().size() <= 8)
      cdr.write_type_e(data.getAvailableYoloModels());else
          throw new RuntimeException("available_yolo_models field exceeds the maximum length: %d > %d".formatted(data.getAvailableYoloModels().size(), 8));

   }

   public static void read(perception_msgs.msg.dds.YOLOv8AvailableModels data, us.ihmc.idl.CDR cdr)
   {
      data.setRequest(cdr.read_type_7());
      	
      cdr.read_type_e(data.getAvailableYoloModels());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8AvailableModels data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_7("request", data.getRequest());
      ser.write_type_e("available_yolo_models", data.getAvailableYoloModels());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8AvailableModels data)
   {
      data.setRequest(ser.read_type_7("request"));
      ser.read_type_e("available_yolo_models", data.getAvailableYoloModels());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8AvailableModels src, perception_msgs.msg.dds.YOLOv8AvailableModels dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8AvailableModels createData()
   {
      return new perception_msgs.msg.dds.YOLOv8AvailableModels();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8AvailableModels data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8AvailableModels data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8AvailableModels src, perception_msgs.msg.dds.YOLOv8AvailableModels dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8AvailableModelsPubSubType newInstance()
   {
      return new YOLOv8AvailableModelsPubSubType();
   }
}
