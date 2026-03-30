package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Information used for annotating a single YOLOv8 detection
       */
public class YOLOv8AnnotationInfoMessage extends Packet<YOLOv8AnnotationInfoMessage> implements Settable<YOLOv8AnnotationInfoMessage>, EpsilonComparable<YOLOv8AnnotationInfoMessage>
{
   /**
            * Class of the detected object
            */
   public java.lang.StringBuilder object_class_;
   /**
            * Confidence of the detection
            */
   public float confidence_;
   /**
            * Bounding box of the detection
            */
   public vision_msgs.msg.dds.BoundingBox2D bounding_box_;
   /**
            * Polygons representing the mask of the detection
            */
   public perception_msgs.msg.dds.UInt16MultiArrayHack mask_polygons_;

   public YOLOv8AnnotationInfoMessage()
   {
      object_class_ = new java.lang.StringBuilder(255);
      bounding_box_ = new vision_msgs.msg.dds.BoundingBox2D();
      mask_polygons_ = new perception_msgs.msg.dds.UInt16MultiArrayHack();
   }

   public YOLOv8AnnotationInfoMessage(YOLOv8AnnotationInfoMessage other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8AnnotationInfoMessage other)
   {
      object_class_.setLength(0);
      object_class_.append(other.object_class_);

      confidence_ = other.confidence_;

      vision_msgs.msg.dds.BoundingBox2DPubSubType.staticCopy(other.bounding_box_, bounding_box_);
      perception_msgs.msg.dds.UInt16MultiArrayHackPubSubType.staticCopy(other.mask_polygons_, mask_polygons_);
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
            * Confidence of the detection
            */
   public void setConfidence(float confidence)
   {
      confidence_ = confidence;
   }
   /**
            * Confidence of the detection
            */
   public float getConfidence()
   {
      return confidence_;
   }


   /**
            * Bounding box of the detection
            */
   public vision_msgs.msg.dds.BoundingBox2D getBoundingBox()
   {
      return bounding_box_;
   }


   /**
            * Polygons representing the mask of the detection
            */
   public perception_msgs.msg.dds.UInt16MultiArrayHack getMaskPolygons()
   {
      return mask_polygons_;
   }


   public static Supplier<YOLOv8AnnotationInfoMessagePubSubType> getPubSubType()
   {
      return YOLOv8AnnotationInfoMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8AnnotationInfoMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8AnnotationInfoMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_class_, other.object_class_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_, other.confidence_, epsilon)) return false;

      if (!this.bounding_box_.epsilonEquals(other.bounding_box_, epsilon)) return false;
      if (!this.mask_polygons_.epsilonEquals(other.mask_polygons_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8AnnotationInfoMessage)) return false;

      YOLOv8AnnotationInfoMessage otherMyClass = (YOLOv8AnnotationInfoMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.object_class_, otherMyClass.object_class_)) return false;

      if(this.confidence_ != otherMyClass.confidence_) return false;

      if (!this.bounding_box_.equals(otherMyClass.bounding_box_)) return false;
      if (!this.mask_polygons_.equals(otherMyClass.mask_polygons_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8AnnotationInfoMessage {");
      builder.append("object_class=");
      builder.append(this.object_class_);      builder.append(", ");
      builder.append("confidence=");
      builder.append(this.confidence_);      builder.append(", ");
      builder.append("bounding_box=");
      builder.append(this.bounding_box_);      builder.append(", ");
      builder.append("mask_polygons=");
      builder.append(this.mask_polygons_);
      builder.append("}");
      return builder.toString();
   }
}
