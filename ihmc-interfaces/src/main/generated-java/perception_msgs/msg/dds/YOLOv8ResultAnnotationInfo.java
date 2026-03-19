package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Information used for annotating YOLOv8 results that may contain multiple detections.
       */
public class YOLOv8ResultAnnotationInfo extends Packet<YOLOv8ResultAnnotationInfo> implements Settable<YOLOv8ResultAnnotationInfo>, EpsilonComparable<YOLOv8ResultAnnotationInfo>
{
   /**
            * Time the detections were made
            */
   public ihmc_common_msgs.msg.dds.InstantMessage detection_instant_;
   /**
            * Detection annotation records
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8AnnotationRecordMessage>  annotation_records_;

   public YOLOv8ResultAnnotationInfo()
   {
      detection_instant_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      annotation_records_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8AnnotationRecordMessage> (100, new perception_msgs.msg.dds.YOLOv8AnnotationRecordMessagePubSubType());

   }

   public YOLOv8ResultAnnotationInfo(YOLOv8ResultAnnotationInfo other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ResultAnnotationInfo other)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.detection_instant_, detection_instant_);
      annotation_records_.set(other.annotation_records_);
   }


   /**
            * Time the detections were made
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getDetectionInstant()
   {
      return detection_instant_;
   }


   /**
            * Detection annotation records
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8AnnotationRecordMessage>  getAnnotationRecords()
   {
      return annotation_records_;
   }


   public static Supplier<YOLOv8ResultAnnotationInfoPubSubType> getPubSubType()
   {
      return YOLOv8ResultAnnotationInfoPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8ResultAnnotationInfoPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8ResultAnnotationInfo other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detection_instant_.epsilonEquals(other.detection_instant_, epsilon)) return false;
      if (this.annotation_records_.size() != other.annotation_records_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.annotation_records_.size(); i++)
         {  if (!this.annotation_records_.get(i).epsilonEquals(other.annotation_records_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8ResultAnnotationInfo)) return false;

      YOLOv8ResultAnnotationInfo otherMyClass = (YOLOv8ResultAnnotationInfo) other;

      if (!this.detection_instant_.equals(otherMyClass.detection_instant_)) return false;
      if (!this.annotation_records_.equals(otherMyClass.annotation_records_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ResultAnnotationInfo {");
      builder.append("detection_instant=");
      builder.append(this.detection_instant_);      builder.append(", ");
      builder.append("annotation_records=");
      builder.append(this.annotation_records_);
      builder.append("}");
      return builder.toString();
   }
}
