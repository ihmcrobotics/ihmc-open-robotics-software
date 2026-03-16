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
   		return "b87821bd87d1d44d4b1de4680918d62c5de7a038fdcb4e3d007598d23fb9c9d5";
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);for(int i0 = 0; i0 < 10; ++i0)
      {
        current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + 255 + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);current_alignment += (256 * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.getCdrSerializedSize(data.getSceneObjectDefinition(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.getCdrSerializedSize(data.getNominalObjectPose(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 2 + us.ihmc.idl.CDR.alignment(current_alignment, 2);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      for(int i0 = 0; i0 < data.getEnabledYoloModels().size(); ++i0)
      {
          current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4) + data.getEnabledYoloModels().get(i0).length() + 1;
      }
      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getIgnoredYoloClassIndices().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);
      current_alignment += (data.getEnabledFoundationPoseModels().size() * 1) + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.write(data.getDefinition(), cdr);
      cdr.write_type_9(data.getSceneActionType());

      behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.write(data.getSceneObjectDefinition(), cdr);
      cdr.write_type_5(data.getTimeout());

      cdr.write_type_3(data.getMinimumHistorySize());

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.write(data.getNominalObjectPose(), cdr);
      cdr.write_type_5(data.getYoloConfidenceThreshold());

      cdr.write_type_5(data.getYoloMaskThreshold());

      cdr.write_type_3(data.getSegmentationMaskErosionRadius());

      cdr.write_type_5(data.getOutlierThreshold());

      if(data.getEnabledYoloModels().size() <= 10)
      cdr.write_type_e(data.getEnabledYoloModels());else
          throw new RuntimeException("enabled_yolo_models field exceeds the maximum length: %d > %d".formatted(data.getEnabledYoloModels().size(), 10));

      if(data.getIgnoredYoloClassIndices().size() <= 256)
      cdr.write_type_e(data.getIgnoredYoloClassIndices());else
          throw new RuntimeException("ignored_yolo_class_indices field exceeds the maximum length: %d > %d".formatted(data.getIgnoredYoloClassIndices().size(), 256));

      cdr.write_type_9(data.getFoundationPoseObjectType());

      if(data.getEnabledFoundationPoseModels().size() <= 10)
      cdr.write_type_e(data.getEnabledFoundationPoseModels());else
          throw new RuntimeException("enabled_foundation_pose_models field exceeds the maximum length: %d > %d".formatted(data.getEnabledFoundationPoseModels().size(), 10));

   }

   public static void read(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.CDR cdr)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.read(data.getDefinition(), cdr);	
      data.setSceneActionType(cdr.read_type_9());
      	
      behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.read(data.getSceneObjectDefinition(), cdr);	
      data.setTimeout(cdr.read_type_5());
      	
      data.setMinimumHistorySize(cdr.read_type_3());
      	
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.read(data.getNominalObjectPose(), cdr);	
      data.setYoloConfidenceThreshold(cdr.read_type_5());
      	
      data.setYoloMaskThreshold(cdr.read_type_5());
      	
      data.setSegmentationMaskErosionRadius(cdr.read_type_3());
      	
      data.setOutlierThreshold(cdr.read_type_5());
      	
      cdr.read_type_e(data.getEnabledYoloModels());	
      cdr.read_type_e(data.getIgnoredYoloClassIndices());	
      data.setFoundationPoseObjectType(cdr.read_type_9());
      	
      cdr.read_type_e(data.getEnabledFoundationPoseModels());	

   }

   @Override
   public final void serialize(behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      ser.write_type_9("scene_action_type", data.getSceneActionType());
      ser.write_type_a("scene_object_definition", new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType(), data.getSceneObjectDefinition());

      ser.write_type_5("timeout", data.getTimeout());
      ser.write_type_3("minimum_history_size", data.getMinimumHistorySize());
      ser.write_type_a("nominal_object_pose", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getNominalObjectPose());

      ser.write_type_5("yolo_confidence_threshold", data.getYoloConfidenceThreshold());
      ser.write_type_5("yolo_mask_threshold", data.getYoloMaskThreshold());
      ser.write_type_3("segmentation_mask_erosion_radius", data.getSegmentationMaskErosionRadius());
      ser.write_type_5("outlier_threshold", data.getOutlierThreshold());
      ser.write_type_e("enabled_yolo_models", data.getEnabledYoloModels());
      ser.write_type_e("ignored_yolo_class_indices", data.getIgnoredYoloClassIndices());
      ser.write_type_9("foundation_pose_object_type", data.getFoundationPoseObjectType());
      ser.write_type_e("enabled_foundation_pose_models", data.getEnabledFoundationPoseModels());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, behavior_msgs.msg.dds.SceneActionNodeDefinitionMessage data)
   {
      ser.read_type_a("definition", new behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType(), data.getDefinition());

      data.setSceneActionType(ser.read_type_9("scene_action_type"));
      ser.read_type_a("scene_object_definition", new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType(), data.getSceneObjectDefinition());

      data.setTimeout(ser.read_type_5("timeout"));
      data.setMinimumHistorySize(ser.read_type_3("minimum_history_size"));
      ser.read_type_a("nominal_object_pose", new controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType(), data.getNominalObjectPose());

      data.setYoloConfidenceThreshold(ser.read_type_5("yolo_confidence_threshold"));
      data.setYoloMaskThreshold(ser.read_type_5("yolo_mask_threshold"));
      data.setSegmentationMaskErosionRadius(ser.read_type_3("segmentation_mask_erosion_radius"));
      data.setOutlierThreshold(ser.read_type_5("outlier_threshold"));
      ser.read_type_e("enabled_yolo_models", data.getEnabledYoloModels());
      ser.read_type_e("ignored_yolo_class_indices", data.getIgnoredYoloClassIndices());
      data.setFoundationPoseObjectType(ser.read_type_9("foundation_pose_object_type"));
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
