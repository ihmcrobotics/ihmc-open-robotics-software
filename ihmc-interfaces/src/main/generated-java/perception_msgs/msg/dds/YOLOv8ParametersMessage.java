package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * TODO: Having these parameter/settings messages kinda sucks.
       * Should try to get a better system.
       * Parameters for running YOLOv8
       */
public class YOLOv8ParametersMessage extends Packet<YOLOv8ParametersMessage> implements Settable<YOLOv8ParametersMessage>, EpsilonComparable<YOLOv8ParametersMessage>
{
   /**
            * Confidence required to consider a mask a valid detection
            */
   public float confidence_threshold_;
   public float non_maximum_suppression_threshold_;
   public float segmentation_threshold_;
   /**
            * How much to shrink the object mask. Higher value = more shrinkage
            */
   public int erosion_kernel_radius_;
   public float outlier_threshold_;
   /**
            * List of YOLOv8DetectionClass values that YOLO should detect
            */
   public us.ihmc.idl.IDLSequence.Byte  target_detection_classes_;

   public YOLOv8ParametersMessage()
   {
      target_detection_classes_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");

   }

   public YOLOv8ParametersMessage(YOLOv8ParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ParametersMessage other)
   {
      confidence_threshold_ = other.confidence_threshold_;

      non_maximum_suppression_threshold_ = other.non_maximum_suppression_threshold_;

      segmentation_threshold_ = other.segmentation_threshold_;

      erosion_kernel_radius_ = other.erosion_kernel_radius_;

      outlier_threshold_ = other.outlier_threshold_;

      target_detection_classes_.set(other.target_detection_classes_);
   }

   /**
            * Confidence required to consider a mask a valid detection
            */
   public void setConfidenceThreshold(float confidence_threshold)
   {
      confidence_threshold_ = confidence_threshold;
   }
   /**
            * Confidence required to consider a mask a valid detection
            */
   public float getConfidenceThreshold()
   {
      return confidence_threshold_;
   }

   public void setNonMaximumSuppressionThreshold(float non_maximum_suppression_threshold)
   {
      non_maximum_suppression_threshold_ = non_maximum_suppression_threshold;
   }
   public float getNonMaximumSuppressionThreshold()
   {
      return non_maximum_suppression_threshold_;
   }

   public void setSegmentationThreshold(float segmentation_threshold)
   {
      segmentation_threshold_ = segmentation_threshold;
   }
   public float getSegmentationThreshold()
   {
      return segmentation_threshold_;
   }

   /**
            * How much to shrink the object mask. Higher value = more shrinkage
            */
   public void setErosionKernelRadius(int erosion_kernel_radius)
   {
      erosion_kernel_radius_ = erosion_kernel_radius;
   }
   /**
            * How much to shrink the object mask. Higher value = more shrinkage
            */
   public int getErosionKernelRadius()
   {
      return erosion_kernel_radius_;
   }

   public void setOutlierThreshold(float outlier_threshold)
   {
      outlier_threshold_ = outlier_threshold;
   }
   public float getOutlierThreshold()
   {
      return outlier_threshold_;
   }


   /**
            * List of YOLOv8DetectionClass values that YOLO should detect
            */
   public us.ihmc.idl.IDLSequence.Byte  getTargetDetectionClasses()
   {
      return target_detection_classes_;
   }


   public static Supplier<YOLOv8ParametersMessagePubSubType> getPubSubType()
   {
      return YOLOv8ParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_threshold_, other.confidence_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.non_maximum_suppression_threshold_, other.non_maximum_suppression_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.segmentation_threshold_, other.segmentation_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.erosion_kernel_radius_, other.erosion_kernel_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.outlier_threshold_, other.outlier_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.target_detection_classes_, other.target_detection_classes_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ParametersMessage)) return false;

      YOLOv8ParametersMessage otherMyClass = (YOLOv8ParametersMessage) other;

      if(this.confidence_threshold_ != otherMyClass.confidence_threshold_) return false;

      if(this.non_maximum_suppression_threshold_ != otherMyClass.non_maximum_suppression_threshold_) return false;

      if(this.segmentation_threshold_ != otherMyClass.segmentation_threshold_) return false;

      if(this.erosion_kernel_radius_ != otherMyClass.erosion_kernel_radius_) return false;

      if(this.outlier_threshold_ != otherMyClass.outlier_threshold_) return false;

      if (!this.target_detection_classes_.equals(otherMyClass.target_detection_classes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ParametersMessage {");
      builder.append("confidence_threshold=");
      builder.append(this.confidence_threshold_);      builder.append(", ");
      builder.append("non_maximum_suppression_threshold=");
      builder.append(this.non_maximum_suppression_threshold_);      builder.append(", ");
      builder.append("segmentation_threshold=");
      builder.append(this.segmentation_threshold_);      builder.append(", ");
      builder.append("erosion_kernel_radius=");
      builder.append(this.erosion_kernel_radius_);      builder.append(", ");
      builder.append("outlier_threshold=");
      builder.append(this.outlier_threshold_);      builder.append(", ");
      builder.append("target_detection_classes=");
      builder.append(this.target_detection_classes_);
      builder.append("}");
      return builder.toString();
   }
}
