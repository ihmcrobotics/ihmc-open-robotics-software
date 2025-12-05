package behavior_msgs.msg.dds;

/**
* 
* Topic data type of the struct "SceneActionNodeDefinitionMessage" defined in "SceneActionNodeDefinitionMessage_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from SceneActionNodeDefinitionMessage_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit SceneActionNodeDefinitionMessage_.idl instead.
*
*/
public class SceneActionNodeDefinitionMessagePubSubType implements us.ihmc.pubsub.TopicDataType<behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage>
{
   public static final java.lang.String name = "behavior_msgs::msg::dds_::SceneActionNodeDefinitionMessage_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "106a465f2ca6c52121b3de77c5e225fc8b620486fef483ba9a1a6320ca9437aa";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data) throws java.io.IOException
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

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (256 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (10 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.getCdrSerializedSize(data.getDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getYoloModelName().length() + 1;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getEnabledYoloModels().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getEnabledYoloModels().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getIgnoredYoloClassIndices().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getEnabledFoundationPoseModels().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      if(data.getYoloModelName().length() <= 255)
      cdr.write_type_d(data.getYoloModelName());else
          throw new RuntimeException("yolo_model_name field exceeds the maximum length: %d > %d".formatted(data.getYoloModelName().length(), 255));

      cdr.write_type_5(data.getYoloConfidenceThreshold());

      cdr.write_type_5(data.getYoloMaskThreshold());

      cdr.write_type_3(data.getSegmentationMaskErosionRadius());

      cdr.write_type_5(data.getOutlierThreshold());

      cdr.write_type_9(data.getObjectType());

      cdr.write_type_7(data.getUseFoundationPose());

      if(data.getEnabledYoloModels().size() <= 10)
      cdr.write_type_e(data.getEnabledYoloModels());else
          throw new RuntimeException("enabled_yolo_models field exceeds the maximum length: %d > %d".formatted(data.getEnabledYoloModels().size(), 10));

      if(data.getIgnoredYoloClassIndices().size() <= 256)
      cdr.write_type_e(data.getIgnoredYoloClassIndices());else
          throw new RuntimeException("ignored_yolo_class_indices field exceeds the maximum length: %d > %d".formatted(data.getIgnoredYoloClassIndices().size(), 256));

      if(data.getEnabledFoundationPoseModels().size() <= 10)
      cdr.write_type_e(data.getEnabledFoundationPoseModels());else
          throw new RuntimeException("enabled_foundation_pose_models field exceeds the maximum length: %d > %d".formatted(data.getEnabledFoundationPoseModels().size(), 10));

   }

   public static void read(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      cdr.read_type_d(data.getYoloModelName());	
      data.setYoloConfidenceThreshold(cdr.read_type_5());
      	
      data.setYoloMaskThreshold(cdr.read_type_5());
      	
      data.setSegmentationMaskErosionRadius(cdr.read_type_3());
      	
      data.setOutlierThreshold(cdr.read_type_5());
      	
      data.setObjectType(cdr.read_type_9());
      	
      data.setUseFoundationPose(cdr.read_type_7());
      	
      cdr.read_type_e(data.getEnabledYoloModels());	
      cdr.read_type_e(data.getIgnoredYoloClassIndices());	
      cdr.read_type_e(data.getEnabledFoundationPoseModels());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_d("yolo_model_name", data.getYoloModelName());
      ser.write_type_5("yolo_confidence_threshold", data.getYoloConfidenceThreshold());
      ser.write_type_5("yolo_mask_threshold", data.getYoloMaskThreshold());
      ser.write_type_3("segmentation_mask_erosion_radius", data.getSegmentationMaskErosionRadius());
      ser.write_type_5("outlier_threshold", data.getOutlierThreshold());
      ser.write_type_9("object_type", data.getObjectType());
      ser.write_type_7("use_foundation_pose", data.getUseFoundationPose());
      ser.write_type_e("enabled_yolo_models", data.getEnabledYoloModels());
      ser.write_type_e("ignored_yolo_class_indices", data.getIgnoredYoloClassIndices());
      ser.write_type_e("enabled_foundation_pose_models", data.getEnabledFoundationPoseModels());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.read_type_d("yolo_model_name", data.getYoloModelName());
      data.setYoloConfidenceThreshold(ser.read_type_5("yolo_confidence_threshold"));
      data.setYoloMaskThreshold(ser.read_type_5("yolo_mask_threshold"));
      data.setSegmentationMaskErosionRadius(ser.read_type_3("segmentation_mask_erosion_radius"));
      data.setOutlierThreshold(ser.read_type_5("outlier_threshold"));
      data.setObjectType(ser.read_type_9("object_type"));
      data.setUseFoundationPose(ser.read_type_7("use_foundation_pose"));
      ser.read_type_e("enabled_yolo_models", data.getEnabledYoloModels());
      ser.read_type_e("ignored_yolo_class_indices", data.getIgnoredYoloClassIndices());
      ser.read_type_e("enabled_foundation_pose_models", data.getEnabledFoundationPoseModels());
   }

   public static void staticCopy(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage src, behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage dest)
   {
      dest.set(src);
   }

   @Override
   public behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage createData()
   {
      return new behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage();
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
   
   public void serialize(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage src, behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public SceneActionNodeDefinitionMessagePubSubType newInstance()
   {
      return new SceneActionNodeDefinitionMessagePubSubType();
   }
}
