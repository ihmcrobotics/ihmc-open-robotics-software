package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8ExecutorSettings" defined in "YOLOv8ExecutorSettings_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8ExecutorSettings_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8ExecutorSettings_.idl instead.
*
*/
public class YOLOv8ExecutorSettingsPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8ExecutorSettings>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8ExecutorSettings_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "03e40887118ba9ab1a4ee8dcdb324f2df578c2e08648077c1a8ecdd89306aa26";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8ExecutorSettings data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8ExecutorSettings data) throws java.io.IOException
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

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8ModelInfoPubSubType.getMaxCdrSerializedSize(current_alignment);}
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 16; ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8ModelSettingsPubSubType.getMaxCdrSerializedSize(current_alignment);}

      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ExecutorSettings data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ExecutorSettings data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestTimestampModifiable(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getAvailableYoloModels().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8ModelInfoPubSubType.getCdrSerializedSize(data.getAvailableYoloModels().get(i0), current_alignment);}

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getModelsToRun().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getModelsToRun().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getModelSettings().size(); ++i0)
      {
          current_alignment += perception_msgs.msg.dds.YOLOv8ModelSettingsPubSubType.getCdrSerializedSize(data.getModelSettings().get(i0), current_alignment);}


      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8ExecutorSettings data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestTimestampModifiable(), cdr);
      if(data.getAvailableYoloModels().size() <= 16)
      cdr.write_type_e(data.getAvailableYoloModels());else
          throw new RuntimeException("available_yolo_models field exceeds the maximum length: %d > %d".formatted(data.getAvailableYoloModels().size(), 16));

      if(data.getModelsToRun().size() <= 16)
      cdr.write_type_e(data.getModelsToRun());else
          throw new RuntimeException("models_to_run field exceeds the maximum length: %d > %d".formatted(data.getModelsToRun().size(), 16));

      if(data.getModelSettings().size() <= 16)
      cdr.write_type_e(data.getModelSettings());else
          throw new RuntimeException("model_settings field exceeds the maximum length: %d > %d".formatted(data.getModelSettings().size(), 16));

   }

   public static void read(perception_msgs.msg.dds.YOLOv8ExecutorSettings data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestTimestampModifiable(), cdr);	
      cdr.read_type_e(data.getAvailableYoloModels());	
      cdr.read_type_e(data.getModelsToRun());	
      cdr.read_type_e(data.getModelSettings());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8ExecutorSettings data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      ser.write_type_e("available_yolo_models", data.getAvailableYoloModels());
      ser.write_type_e("models_to_run", data.getModelsToRun());
      ser.write_type_e("model_settings", data.getModelSettings());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8ExecutorSettings data)
   {
      ser.read_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      ser.read_type_e("available_yolo_models", data.getAvailableYoloModels());
      ser.read_type_e("models_to_run", data.getModelsToRun());
      ser.read_type_e("model_settings", data.getModelSettings());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8ExecutorSettings src, perception_msgs.msg.dds.YOLOv8ExecutorSettings dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8ExecutorSettings createData()
   {
      return new perception_msgs.msg.dds.YOLOv8ExecutorSettings();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8ExecutorSettings data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8ExecutorSettings data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8ExecutorSettings src, perception_msgs.msg.dds.YOLOv8ExecutorSettings dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8ExecutorSettingsPubSubType newInstance()
   {
      return new YOLOv8ExecutorSettingsPubSubType();
   }
}
