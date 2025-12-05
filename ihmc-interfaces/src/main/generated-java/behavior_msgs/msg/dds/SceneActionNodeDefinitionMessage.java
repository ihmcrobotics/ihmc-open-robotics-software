package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class SceneActionNodeDefinitionMessage extends Packet<SceneActionNodeDefinitionMessage> implements Settable<SceneActionNodeDefinitionMessage>, EpsilonComparable<SceneActionNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * YOLO model name to use
            */
   public java.lang.StringBuilder yolo_model_name_;
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
            * FoundationPose object type (enum ordinal)
            */
   public byte object_type_;
   /**
            * True to use foundation pose, else use a YOLO based detection
            */
   public boolean use_foundation_pose_;
   /**
            * YOLO models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  enabled_yolo_models_;
   /**
            * YOLO classes to ignore
            */
   public us.ihmc.idl.IDLSequence.Byte  ignored_yolo_class_indices_;
   /**
            * FoundationPose models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.Byte  enabled_foundation_pose_models_;

   public SceneActionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      yolo_model_name_ = new java.lang.StringBuilder(255);
      enabled_yolo_models_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (10, "type_d");
      ignored_yolo_class_indices_ = new us.ihmc.idl.IDLSequence.Byte (256, "type_9");

      enabled_foundation_pose_models_ = new us.ihmc.idl.IDLSequence.Byte (10, "type_9");

   }

   public SceneActionNodeDefinitionMessage(SceneActionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneActionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      yolo_model_name_.setLength(0);
      yolo_model_name_.append(other.yolo_model_name_);

      yolo_confidence_threshold_ = other.yolo_confidence_threshold_;

      yolo_mask_threshold_ = other.yolo_mask_threshold_;

      segmentation_mask_erosion_radius_ = other.segmentation_mask_erosion_radius_;

      outlier_threshold_ = other.outlier_threshold_;

      object_type_ = other.object_type_;

      use_foundation_pose_ = other.use_foundation_pose_;

      enabled_yolo_models_.set(other.enabled_yolo_models_);
      ignored_yolo_class_indices_.set(other.ignored_yolo_class_indices_);
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
            * YOLO model name to use
            */
   public void setYoloModelName(java.lang.String yolo_model_name)
   {
      yolo_model_name_.setLength(0);
      yolo_model_name_.append(yolo_model_name);
   }

   /**
            * YOLO model name to use
            */
   public java.lang.String getYoloModelNameAsString()
   {
      return getYoloModelName().toString();
   }
   /**
            * YOLO model name to use
            */
   public java.lang.StringBuilder getYoloModelName()
   {
      return yolo_model_name_;
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
            * FoundationPose object type (enum ordinal)
            */
   public void setObjectType(byte object_type)
   {
      object_type_ = object_type;
   }
   /**
            * FoundationPose object type (enum ordinal)
            */
   public byte getObjectType()
   {
      return object_type_;
   }

   /**
            * True to use foundation pose, else use a YOLO based detection
            */
   public void setUseFoundationPose(boolean use_foundation_pose)
   {
      use_foundation_pose_ = use_foundation_pose;
   }
   /**
            * True to use foundation pose, else use a YOLO based detection
            */
   public boolean getUseFoundationPose()
   {
      return use_foundation_pose_;
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
            * FoundationPose models that should be enabled at this point
            */
   public us.ihmc.idl.IDLSequence.Byte  getEnabledFoundationPoseModels()
   {
      return enabled_foundation_pose_models_;
   }


   public static Supplier<SceneActionNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return SceneActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SceneActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SceneActionNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.yolo_model_name_, other.yolo_model_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yolo_confidence_threshold_, other.yolo_confidence_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yolo_mask_threshold_, other.yolo_mask_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.segmentation_mask_erosion_radius_, other.segmentation_mask_erosion_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.outlier_threshold_, other.outlier_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_type_, other.object_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_foundation_pose_, other.use_foundation_pose_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.enabled_yolo_models_, other.enabled_yolo_models_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.ignored_yolo_class_indices_, other.ignored_yolo_class_indices_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.enabled_foundation_pose_models_, other.enabled_foundation_pose_models_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SceneActionNodeDefinitionMessage)) return false;

      SceneActionNodeDefinitionMessage otherMyClass = (SceneActionNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.yolo_model_name_, otherMyClass.yolo_model_name_)) return false;

      if(this.yolo_confidence_threshold_ != otherMyClass.yolo_confidence_threshold_) return false;

      if(this.yolo_mask_threshold_ != otherMyClass.yolo_mask_threshold_) return false;

      if(this.segmentation_mask_erosion_radius_ != otherMyClass.segmentation_mask_erosion_radius_) return false;

      if(this.outlier_threshold_ != otherMyClass.outlier_threshold_) return false;

      if(this.object_type_ != otherMyClass.object_type_) return false;

      if(this.use_foundation_pose_ != otherMyClass.use_foundation_pose_) return false;

      if (!this.enabled_yolo_models_.equals(otherMyClass.enabled_yolo_models_)) return false;
      if (!this.ignored_yolo_class_indices_.equals(otherMyClass.ignored_yolo_class_indices_)) return false;
      if (!this.enabled_foundation_pose_models_.equals(otherMyClass.enabled_foundation_pose_models_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneActionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("yolo_model_name=");
      builder.append(this.yolo_model_name_);      builder.append(", ");
      builder.append("yolo_confidence_threshold=");
      builder.append(this.yolo_confidence_threshold_);      builder.append(", ");
      builder.append("yolo_mask_threshold=");
      builder.append(this.yolo_mask_threshold_);      builder.append(", ");
      builder.append("segmentation_mask_erosion_radius=");
      builder.append(this.segmentation_mask_erosion_radius_);      builder.append(", ");
      builder.append("outlier_threshold=");
      builder.append(this.outlier_threshold_);      builder.append(", ");
      builder.append("object_type=");
      builder.append(this.object_type_);      builder.append(", ");
      builder.append("use_foundation_pose=");
      builder.append(this.use_foundation_pose_);      builder.append(", ");
      builder.append("enabled_yolo_models=");
      builder.append(this.enabled_yolo_models_);      builder.append(", ");
      builder.append("ignored_yolo_class_indices=");
      builder.append(this.ignored_yolo_class_indices_);      builder.append(", ");
      builder.append("enabled_foundation_pose_models=");
      builder.append(this.enabled_foundation_pose_models_);
      builder.append("}");
      return builder.toString();
   }
}
