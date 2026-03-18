package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Information used for annotating a single YOLOv8 detection
       */
public class YOLOv8DetectionAnnotationInfo extends Packet<YOLOv8DetectionAnnotationInfo> implements Settable<YOLOv8DetectionAnnotationInfo>, EpsilonComparable<YOLOv8DetectionAnnotationInfo>
{
   /**
            * Time the detection was made
            */
   public ihmc_common_msgs.msg.dds.InstantMessage detection_instant_;
   /**
            * Class of the detected object
            */
   public java.lang.StringBuilder object_class_;
   /**
            * Bounding box of the detection
            */
   public vision_msgs.msg.dds.BoundingBox2D bounding_box_;
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.Point2DArray>  mask_polygons_;

   public YOLOv8DetectionAnnotationInfo()
   {
      detection_instant_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      object_class_ = new java.lang.StringBuilder(255);
      bounding_box_ = new vision_msgs.msg.dds.BoundingBox2D();
      mask_polygons_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.Point2DArray> (100, new perception_msgs.msg.dds.Point2DArrayPubSubType());

   }

   public YOLOv8DetectionAnnotationInfo(YOLOv8DetectionAnnotationInfo other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8DetectionAnnotationInfo other)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.detection_instant_, detection_instant_);
      object_class_.setLength(0);
      object_class_.append(other.object_class_);

      vision_msgs.msg.dds.BoundingBox2DPubSubType.staticCopy(other.bounding_box_, bounding_box_);
      mask_polygons_.set(other.mask_polygons_);
   }


   /**
            * Time the detection was made
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getDetectionInstant()
   {
      return detection_instant_;
   }

   /**
            * Class of the detected object
            */
   public void setObjectClass(java.lang.String object_class)
   {
      object_class_.setLength(0);
      object_class_.append(object_class);
   }

   /**
            * Class of the detected object
            */
   public java.lang.String getObjectClassAsString()
   {
      return getObjectClass().toString();
   }
   /**
            * Class of the detected object
            */
   public java.lang.StringBuilder getObjectClass()
   {
      return object_class_;
   }


   /**
            * Bounding box of the detection
            */
   public vision_msgs.msg.dds.BoundingBox2D getBoundingBox()
   {
      return bounding_box_;
   }


   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.Point2DArray>  getMaskPolygons()
   {
      return mask_polygons_;
   }


   public static Supplier<YOLOv8DetectionAnnotationInfoPubSubType> getPubSubType()
   {
      return YOLOv8DetectionAnnotationInfoPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8DetectionAnnotationInfoPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8DetectionAnnotationInfo other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detection_instant_.epsilonEquals(other.detection_instant_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_class_, other.object_class_, epsilon)) return false;

      if (!this.bounding_box_.epsilonEquals(other.bounding_box_, epsilon)) return false;
      if (this.mask_polygons_.size() != other.mask_polygons_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.mask_polygons_.size(); i++)
         {  if (!this.mask_polygons_.get(i).epsilonEquals(other.mask_polygons_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8DetectionAnnotationInfo)) return false;

      YOLOv8DetectionAnnotationInfo otherMyClass = (YOLOv8DetectionAnnotationInfo) other;

      if (!this.detection_instant_.equals(otherMyClass.detection_instant_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.object_class_, otherMyClass.object_class_)) return false;

      if (!this.bounding_box_.equals(otherMyClass.bounding_box_)) return false;
      if (!this.mask_polygons_.equals(otherMyClass.mask_polygons_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8DetectionAnnotationInfo {");
      builder.append("detection_instant=");
      builder.append(this.detection_instant_);      builder.append(", ");
      builder.append("object_class=");
      builder.append(this.object_class_);      builder.append(", ");
      builder.append("bounding_box=");
      builder.append(this.bounding_box_);      builder.append(", ");
      builder.append("mask_polygons=");
      builder.append(this.mask_polygons_);
      builder.append("}");
      return builder.toString();
   }
}
