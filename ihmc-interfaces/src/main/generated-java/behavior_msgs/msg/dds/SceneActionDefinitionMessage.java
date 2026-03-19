package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * TODO: The following fields are work in progress:
       */
public class SceneActionDefinitionMessage extends Packet<SceneActionDefinitionMessage> implements Settable<SceneActionDefinitionMessage>, EpsilonComparable<SceneActionDefinitionMessage>
{
   public static final byte SETUP_OBJECT = (byte) 0;
   public static final byte FREEZE_OBJECT = (byte) 1;
   public static final byte DELETE_OBJECT = (byte) 2;
   public static final byte CLEAR_SCENE = (byte) 3;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * The type of action as defined above
            */
   public byte scene_action_type_;
   /**
            * Definition of the object to setup
            */
   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage scene_object_definition_;
   /**
            * If this action can't do what it needs to do in this amount of time, give up.
            */
   public float timeout_;
   /**
            * Filter for persistent detection history size
            */
   public int minimum_history_size_;
   /**
            * The nominal object pose for behavior previewing
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage nominal_object_pose_;
   /**
            * YOLO confidence threshold
            */
   public float yolo_confidence_threshold_;
   /**
            * YOLO mask threshold
            */
   public float yolo_mask_threshold_;
   /**
            * Segmentation mask erosion radius
            */
   public int segmentation_mask_erosion_radius_;
   /**
            * Threshold for removing outlier points of segmented point cloud
            */
   public float outlier_threshold_;
   /**
            * YOLO models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  enabled_yolo_models_;
   /**
            * YOLO classes to ignore
            */
   public us.ihmc.idl.IDLSequence.Byte  ignored_yolo_class_indices_;
   /**
            * FoundationPose object type (enum ordinal)
            */
   public byte foundation_pose_object_type_;
   /**
            * FoundationPose models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.Byte  enabled_foundation_pose_models_;

   public SceneActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      scene_object_definition_ = new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage();
      nominal_object_pose_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      enabled_yolo_models_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (10, "type_d");
      ignored_yolo_class_indices_ = new us.ihmc.idl.IDLSequence.Byte (256, "type_9");

      enabled_foundation_pose_models_ = new us.ihmc.idl.IDLSequence.Byte (10, "type_9");

   }

   public SceneActionDefinitionMessage(SceneActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      scene_action_type_ = other.scene_action_type_;

      behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.staticCopy(other.scene_object_definition_, scene_object_definition_);
      timeout_ = other.timeout_;

      minimum_history_size_ = other.minimum_history_size_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.nominal_object_pose_, nominal_object_pose_);
      yolo_confidence_threshold_ = other.yolo_confidence_threshold_;

      yolo_mask_threshold_ = other.yolo_mask_threshold_;

      segmentation_mask_erosion_radius_ = other.segmentation_mask_erosion_radius_;

      outlier_threshold_ = other.outlier_threshold_;

      enabled_yolo_models_.set(other.enabled_yolo_models_);
      ignored_yolo_class_indices_.set(other.ignored_yolo_class_indices_);
      foundation_pose_object_type_ = other.foundation_pose_object_type_;

      enabled_foundation_pose_models_.set(other.enabled_foundation_pose_models_);
   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The type of action as defined above
            */
   public void setSceneActionType(byte scene_action_type)
   {
      scene_action_type_ = scene_action_type;
   }
   /**
            * The type of action as defined above
            */
   public byte getSceneActionType()
   {
      return scene_action_type_;
   }


   /**
            * Definition of the object to setup
            */
   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage getSceneObjectDefinition()
   {
      return scene_object_definition_;
   }

   /**
            * If this action can't do what it needs to do in this amount of time, give up.
            */
   public void setTimeout(float timeout)
   {
      timeout_ = timeout;
   }
   /**
            * If this action can't do what it needs to do in this amount of time, give up.
            */
   public float getTimeout()
   {
      return timeout_;
   }

   /**
            * Filter for persistent detection history size
            */
   public void setMinimumHistorySize(int minimum_history_size)
   {
      minimum_history_size_ = minimum_history_size;
   }
   /**
            * Filter for persistent detection history size
            */
   public int getMinimumHistorySize()
   {
      return minimum_history_size_;
   }


   /**
            * The nominal object pose for behavior previewing
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getNominalObjectPose()
   {
      return nominal_object_pose_;
   }

   /**
            * YOLO confidence threshold
            */
   public void setYoloConfidenceThreshold(float yolo_confidence_threshold)
   {
      yolo_confidence_threshold_ = yolo_confidence_threshold;
   }
   /**
            * YOLO confidence threshold
            */
   public float getYoloConfidenceThreshold()
   {
      return yolo_confidence_threshold_;
   }

   /**
            * YOLO mask threshold
            */
   public void setYoloMaskThreshold(float yolo_mask_threshold)
   {
      yolo_mask_threshold_ = yolo_mask_threshold;
   }
   /**
            * YOLO mask threshold
            */
   public float getYoloMaskThreshold()
   {
      return yolo_mask_threshold_;
   }

   /**
            * Segmentation mask erosion radius
            */
   public void setSegmentationMaskErosionRadius(int segmentation_mask_erosion_radius)
   {
      segmentation_mask_erosion_radius_ = segmentation_mask_erosion_radius;
   }
   /**
            * Segmentation mask erosion radius
            */
   public int getSegmentationMaskErosionRadius()
   {
      return segmentation_mask_erosion_radius_;
   }

   /**
            * Threshold for removing outlier points of segmented point cloud
            */
   public void setOutlierThreshold(float outlier_threshold)
   {
      outlier_threshold_ = outlier_threshold;
   }
   /**
            * Threshold for removing outlier points of segmented point cloud
            */
   public float getOutlierThreshold()
   {
      return outlier_threshold_;
   }


   /**
            * YOLO models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getEnabledYoloModels()
   {
      return enabled_yolo_models_;
   }


   /**
            * YOLO classes to ignore
            */
   public us.ihmc.idl.IDLSequence.Byte  getIgnoredYoloClassIndices()
   {
      return ignored_yolo_class_indices_;
   }

   /**
            * FoundationPose object type (enum ordinal)
            */
   public void setFoundationPoseObjectType(byte foundation_pose_object_type)
   {
      foundation_pose_object_type_ = foundation_pose_object_type;
   }
   /**
            * FoundationPose object type (enum ordinal)
            */
   public byte getFoundationPoseObjectType()
   {
      return foundation_pose_object_type_;
   }


   /**
            * FoundationPose models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.Byte  getEnabledFoundationPoseModels()
   {
      return enabled_foundation_pose_models_;
   }


   public static Supplier<SceneActionDefinitionMessagePubSubType> getPubSubType()
   {
      return SceneActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SceneActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SceneActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.scene_action_type_, other.scene_action_type_, epsilon)) return false;

      if (!this.scene_object_definition_.epsilonEquals(other.scene_object_definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.minimum_history_size_, other.minimum_history_size_, epsilon)) return false;

      if (!this.nominal_object_pose_.epsilonEquals(other.nominal_object_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yolo_confidence_threshold_, other.yolo_confidence_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yolo_mask_threshold_, other.yolo_mask_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.segmentation_mask_erosion_radius_, other.segmentation_mask_erosion_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.outlier_threshold_, other.outlier_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.enabled_yolo_models_, other.enabled_yolo_models_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.ignored_yolo_class_indices_, other.ignored_yolo_class_indices_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.foundation_pose_object_type_, other.foundation_pose_object_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.enabled_foundation_pose_models_, other.enabled_foundation_pose_models_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SceneActionDefinitionMessage)) return false;

      SceneActionDefinitionMessage otherMyClass = (SceneActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.scene_action_type_ != otherMyClass.scene_action_type_) return false;

      if (!this.scene_object_definition_.equals(otherMyClass.scene_object_definition_)) return false;
      if(this.timeout_ != otherMyClass.timeout_) return false;

      if(this.minimum_history_size_ != otherMyClass.minimum_history_size_) return false;

      if (!this.nominal_object_pose_.equals(otherMyClass.nominal_object_pose_)) return false;
      if(this.yolo_confidence_threshold_ != otherMyClass.yolo_confidence_threshold_) return false;

      if(this.yolo_mask_threshold_ != otherMyClass.yolo_mask_threshold_) return false;

      if(this.segmentation_mask_erosion_radius_ != otherMyClass.segmentation_mask_erosion_radius_) return false;

      if(this.outlier_threshold_ != otherMyClass.outlier_threshold_) return false;

      if (!this.enabled_yolo_models_.equals(otherMyClass.enabled_yolo_models_)) return false;
      if (!this.ignored_yolo_class_indices_.equals(otherMyClass.ignored_yolo_class_indices_)) return false;
      if(this.foundation_pose_object_type_ != otherMyClass.foundation_pose_object_type_) return false;

      if (!this.enabled_foundation_pose_models_.equals(otherMyClass.enabled_foundation_pose_models_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("scene_action_type=");
      builder.append(this.scene_action_type_);      builder.append(", ");
      builder.append("scene_object_definition=");
      builder.append(this.scene_object_definition_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");
      builder.append("minimum_history_size=");
      builder.append(this.minimum_history_size_);      builder.append(", ");
      builder.append("nominal_object_pose=");
      builder.append(this.nominal_object_pose_);      builder.append(", ");
      builder.append("yolo_confidence_threshold=");
      builder.append(this.yolo_confidence_threshold_);      builder.append(", ");
      builder.append("yolo_mask_threshold=");
      builder.append(this.yolo_mask_threshold_);      builder.append(", ");
      builder.append("segmentation_mask_erosion_radius=");
      builder.append(this.segmentation_mask_erosion_radius_);      builder.append(", ");
      builder.append("outlier_threshold=");
      builder.append(this.outlier_threshold_);      builder.append(", ");
      builder.append("enabled_yolo_models=");
      builder.append(this.enabled_yolo_models_);      builder.append(", ");
      builder.append("ignored_yolo_class_indices=");
      builder.append(this.ignored_yolo_class_indices_);      builder.append(", ");
      builder.append("foundation_pose_object_type=");
      builder.append(this.foundation_pose_object_type_);      builder.append(", ");
      builder.append("enabled_foundation_pose_models=");
      builder.append(this.enabled_foundation_pose_models_);
      builder.append("}");
      return builder.toString();
   }
}
