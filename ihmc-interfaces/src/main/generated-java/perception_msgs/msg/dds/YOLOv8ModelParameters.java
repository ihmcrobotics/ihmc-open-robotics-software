package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Settings for an individual YOLOv8 Model
       * Below are parameters not used directly by the YOLOv8Model class,
       * but related to the way we use the YOLO output.
       */
public class YOLOv8ModelParameters extends Packet<YOLOv8ModelParameters> implements Settable<YOLOv8ModelParameters>, EpsilonComparable<YOLOv8ModelParameters>
{
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_timestamp_modifiable_;
   public java.lang.StringBuilder model_name_;
   /**
            * A boolean for each object class, whether its detections should be ignored
            */
   public us.ihmc.idl.IDLSequence.Boolean  ignored_object_classes_;
   /**
            * Confidence thresholds, corresponding to each detectable object class
            */
   public us.ihmc.idl.IDLSequence.Float  confidence_thresholds_;
   /**
            * Mask thresholds, corresponding to each detectable object class
            */
   public us.ihmc.idl.IDLSequence.Float  mask_thresholds_;
   /**
            * Non-maximum suppression threshold
            */
   public float non_maximum_suppression_threshold_;
   /**
            * Radii used for mask erosion, corresponding to each detectable object class
            */
   public us.ihmc.idl.IDLSequence.Integer  erosion_kernel_radii_;
   /**
            * Thresholds for removing outlier points of segmented point cloud
            */
   public us.ihmc.idl.IDLSequence.Float  outlier_thresholds_;

   public YOLOv8ModelParameters()
   {
      latest_timestamp_modifiable_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      model_name_ = new java.lang.StringBuilder(255);
      ignored_object_classes_ = new us.ihmc.idl.IDLSequence.Boolean (96, "type_7");

      confidence_thresholds_ = new us.ihmc.idl.IDLSequence.Float (96, "type_5");

      mask_thresholds_ = new us.ihmc.idl.IDLSequence.Float (96, "type_5");

      erosion_kernel_radii_ = new us.ihmc.idl.IDLSequence.Integer (96, "type_3");

      outlier_thresholds_ = new us.ihmc.idl.IDLSequence.Float (96, "type_5");

   }

   public YOLOv8ModelParameters(YOLOv8ModelParameters other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ModelParameters other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_timestamp_modifiable_, latest_timestamp_modifiable_);
      model_name_.setLength(0);
      model_name_.append(other.model_name_);

      ignored_object_classes_.set(other.ignored_object_classes_);
      confidence_thresholds_.set(other.confidence_thresholds_);
      mask_thresholds_.set(other.mask_thresholds_);
      non_maximum_suppression_threshold_ = other.non_maximum_suppression_threshold_;

      erosion_kernel_radii_.set(other.erosion_kernel_radii_);
      outlier_thresholds_.set(other.outlier_thresholds_);
   }


   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestTimestampModifiable()
   {
      return latest_timestamp_modifiable_;
   }

   public void setModelName(java.lang.String model_name)
   {
      model_name_.setLength(0);
      model_name_.append(model_name);
   }

   public java.lang.String getModelNameAsString()
   {
      return getModelName().toString();
   }
   public java.lang.StringBuilder getModelName()
   {
      return model_name_;
   }


   /**
            * A boolean for each object class, whether its detections should be ignored
            */
   public us.ihmc.idl.IDLSequence.Boolean  getIgnoredObjectClasses()
   {
      return ignored_object_classes_;
   }


   /**
            * Confidence thresholds, corresponding to each detectable object class
            */
   public us.ihmc.idl.IDLSequence.Float  getConfidenceThresholds()
   {
      return confidence_thresholds_;
   }


   /**
            * Mask thresholds, corresponding to each detectable object class
            */
   public us.ihmc.idl.IDLSequence.Float  getMaskThresholds()
   {
      return mask_thresholds_;
   }

   /**
            * Non-maximum suppression threshold
            */
   public void setNonMaximumSuppressionThreshold(float non_maximum_suppression_threshold)
   {
      non_maximum_suppression_threshold_ = non_maximum_suppression_threshold;
   }
   /**
            * Non-maximum suppression threshold
            */
   public float getNonMaximumSuppressionThreshold()
   {
      return non_maximum_suppression_threshold_;
   }


   /**
            * Radii used for mask erosion, corresponding to each detectable object class
            */
   public us.ihmc.idl.IDLSequence.Integer  getErosionKernelRadii()
   {
      return erosion_kernel_radii_;
   }


   /**
            * Thresholds for removing outlier points of segmented point cloud
            */
   public us.ihmc.idl.IDLSequence.Float  getOutlierThresholds()
   {
      return outlier_thresholds_;
   }


   public static Supplier<YOLOv8ModelParametersPubSubType> getPubSubType()
   {
      return YOLOv8ModelParametersPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ModelParametersPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ModelParameters other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_timestamp_modifiable_.epsilonEquals(other.latest_timestamp_modifiable_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.model_name_, other.model_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBooleanSequence(this.ignored_object_classes_, other.ignored_object_classes_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.confidence_thresholds_, other.confidence_thresholds_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.mask_thresholds_, other.mask_thresholds_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.non_maximum_suppression_threshold_, other.non_maximum_suppression_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.erosion_kernel_radii_, other.erosion_kernel_radii_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.outlier_thresholds_, other.outlier_thresholds_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ModelParameters)) return false;

      YOLOv8ModelParameters otherMyClass = (YOLOv8ModelParameters) other;

      if (!this.latest_timestamp_modifiable_.equals(otherMyClass.latest_timestamp_modifiable_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.model_name_, otherMyClass.model_name_)) return false;

      if (!this.ignored_object_classes_.equals(otherMyClass.ignored_object_classes_)) return false;
      if (!this.confidence_thresholds_.equals(otherMyClass.confidence_thresholds_)) return false;
      if (!this.mask_thresholds_.equals(otherMyClass.mask_thresholds_)) return false;
      if(this.non_maximum_suppression_threshold_ != otherMyClass.non_maximum_suppression_threshold_) return false;

      if (!this.erosion_kernel_radii_.equals(otherMyClass.erosion_kernel_radii_)) return false;
      if (!this.outlier_thresholds_.equals(otherMyClass.outlier_thresholds_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ModelParameters {");
      builder.append("latest_timestamp_modifiable=");
      builder.append(this.latest_timestamp_modifiable_);      builder.append(", ");
      builder.append("model_name=");
      builder.append(this.model_name_);      builder.append(", ");
      builder.append("ignored_object_classes=");
      builder.append(this.ignored_object_classes_);      builder.append(", ");
      builder.append("confidence_thresholds=");
      builder.append(this.confidence_thresholds_);      builder.append(", ");
      builder.append("mask_thresholds=");
      builder.append(this.mask_thresholds_);      builder.append(", ");
      builder.append("non_maximum_suppression_threshold=");
      builder.append(this.non_maximum_suppression_threshold_);      builder.append(", ");
      builder.append("erosion_kernel_radii=");
      builder.append(this.erosion_kernel_radii_);      builder.append(", ");
      builder.append("outlier_thresholds=");
      builder.append(this.outlier_thresholds_);
      builder.append("}");
      return builder.toString();
   }
}
