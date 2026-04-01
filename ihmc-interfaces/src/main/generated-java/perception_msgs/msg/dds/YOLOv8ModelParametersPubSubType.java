package perception_msgs.msg.dds;

/**
* 
* Topic data type of the struct "YOLOv8ModelParameters" defined in "YOLOv8ModelParameters_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from YOLOv8ModelParameters_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit YOLOv8ModelParameters_.idl instead.
*
*/
public class YOLOv8ModelParametersPubSubType implements us.ihmc.pubsub.TopicDataType<perception_msgs.msg.dds.YOLOv8ModelParameters>
{
   public static final java.lang.String name = "perception_msgs::msg::dds_::YOLOv8ModelParameters_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "8b2e39292064cb9541915ce4b1c4506fa442921aec6c1ce007c92aae144b6554";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(perception_msgs.msg.dds.YOLOv8ModelParameters data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, perception_msgs.msg.dds.YOLOv8ModelParameters data) throws java.io.IOException
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

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (96 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (96 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (96 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (96 * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (96 * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ModelParameters data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(perception_msgs.msg.dds.YOLOv8ModelParameters data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.getCdrSerializedSize(data.getLatestTimestampModifiable(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getModelName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getIgnoredObjectClasses().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getConfidenceThresholds().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getMaskThresholds().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getErosionKernelRadii().size() * 2) + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getOutlierThresholds().size() * 4) + us.ihmc.idl.CDR.alignment(current_alignment, 4);



      return current_alignment - initial_alignment;
   }

   public static void write(perception_msgs.msg.dds.YOLOv8ModelParameters data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.write(data.getLatestTimestampModifiable(), cdr);
      if(data.getModelName().length() <= 255)
      cdr.write_type_d(data.getModelName());else
          throw new RuntimeException("model_name field exceeds the maximum length: %d > %d".formatted(data.getModelName().length(), 255));

      if(data.getIgnoredObjectClasses().size() <= 96)
      cdr.write_type_e(data.getIgnoredObjectClasses());else
          throw new RuntimeException("ignored_object_classes field exceeds the maximum length: %d > %d".formatted(data.getIgnoredObjectClasses().size(), 96));

      if(data.getConfidenceThresholds().size() <= 96)
      cdr.write_type_e(data.getConfidenceThresholds());else
          throw new RuntimeException("confidence_thresholds field exceeds the maximum length: %d > %d".formatted(data.getConfidenceThresholds().size(), 96));

      if(data.getMaskThresholds().size() <= 96)
      cdr.write_type_e(data.getMaskThresholds());else
          throw new RuntimeException("mask_thresholds field exceeds the maximum length: %d > %d".formatted(data.getMaskThresholds().size(), 96));

      cdr.write_type_5(data.getNonMaximumSuppressionThreshold());

      if(data.getErosionKernelRadii().size() <= 96)
      cdr.write_type_e(data.getErosionKernelRadii());else
          throw new RuntimeException("erosion_kernel_radii field exceeds the maximum length: %d > %d".formatted(data.getErosionKernelRadii().size(), 96));

      if(data.getOutlierThresholds().size() <= 96)
      cdr.write_type_e(data.getOutlierThresholds());else
          throw new RuntimeException("outlier_thresholds field exceeds the maximum length: %d > %d".formatted(data.getOutlierThresholds().size(), 96));

   }

   public static void read(perception_msgs.msg.dds.YOLOv8ModelParameters data, us.ihmc.idl.CDR cdr)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.read(data.getLatestTimestampModifiable(), cdr);	
      cdr.read_type_d(data.getModelName());	
      cdr.read_type_e(data.getIgnoredObjectClasses());	
      cdr.read_type_e(data.getConfidenceThresholds());	
      cdr.read_type_e(data.getMaskThresholds());	
      data.setNonMaximumSuppressionThreshold(cdr.read_type_5());
      	
      cdr.read_type_e(data.getErosionKernelRadii());	
      cdr.read_type_e(data.getOutlierThresholds());	

   }

   @Override
   public final void serialize(perception_msgs.msg.dds.YOLOv8ModelParameters data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      ser.write_type_d("model_name", data.getModelName());
      ser.write_type_e("ignored_object_classes", data.getIgnoredObjectClasses());
      ser.write_type_e("confidence_thresholds", data.getConfidenceThresholds());
      ser.write_type_e("mask_thresholds", data.getMaskThresholds());
      ser.write_type_5("non_maximum_suppression_threshold", data.getNonMaximumSuppressionThreshold());
      ser.write_type_e("erosion_kernel_radii", data.getErosionKernelRadii());
      ser.write_type_e("outlier_thresholds", data.getOutlierThresholds());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, perception_msgs.msg.dds.YOLOv8ModelParameters data)
   {
      ser.read_type_a("latest_timestamp_modifiable", new ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType(), data.getLatestTimestampModifiable());

      ser.read_type_d("model_name", data.getModelName());
      ser.read_type_e("ignored_object_classes", data.getIgnoredObjectClasses());
      ser.read_type_e("confidence_thresholds", data.getConfidenceThresholds());
      ser.read_type_e("mask_thresholds", data.getMaskThresholds());
      data.setNonMaximumSuppressionThreshold(ser.read_type_5("non_maximum_suppression_threshold"));
      ser.read_type_e("erosion_kernel_radii", data.getErosionKernelRadii());
      ser.read_type_e("outlier_thresholds", data.getOutlierThresholds());
   }

   public static void staticCopy(perception_msgs.msg.dds.YOLOv8ModelParameters src, perception_msgs.msg.dds.YOLOv8ModelParameters dest)
   {
      dest.set(src);
   }

   @Override
   public perception_msgs.msg.dds.YOLOv8ModelParameters createData()
   {
      return new perception_msgs.msg.dds.YOLOv8ModelParameters();
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
   
   public void serialize(perception_msgs.msg.dds.YOLOv8ModelParameters data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(perception_msgs.msg.dds.YOLOv8ModelParameters data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(perception_msgs.msg.dds.YOLOv8ModelParameters src, perception_msgs.msg.dds.YOLOv8ModelParameters dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public YOLOv8ModelParametersPubSubType newInstance()
   {
      return new YOLOv8ModelParametersPubSubType();
   }
}
