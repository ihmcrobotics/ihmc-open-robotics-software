package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class YOLOv8ResultAnnotationInfo extends Packet<YOLOv8ResultAnnotationInfo> implements Settable<YOLOv8ResultAnnotationInfo>, EpsilonComparable<YOLOv8ResultAnnotationInfo>
{
   /**
            * Information used for annotating YOLOv8 results that may contain multiple detections.
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo>  detection_annotation_infos_;

   public YOLOv8ResultAnnotationInfo()
   {
      detection_annotation_infos_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo> (100, new perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfoPubSubType());

   }

   public YOLOv8ResultAnnotationInfo(YOLOv8ResultAnnotationInfo other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8ResultAnnotationInfo other)
   {
      detection_annotation_infos_.set(other.detection_annotation_infos_);
   }


   /**
            * Information used for annotating YOLOv8 results that may contain multiple detections.
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8DetectionAnnotationInfo>  getDetectionAnnotationInfos()
   {
      return detection_annotation_infos_;
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

      if (this.detection_annotation_infos_.size() != other.detection_annotation_infos_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detection_annotation_infos_.size(); i++)
         {  if (!this.detection_annotation_infos_.get(i).epsilonEquals(other.detection_annotation_infos_.get(i), epsilon)) return false; }
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

      if (!this.detection_annotation_infos_.equals(otherMyClass.detection_annotation_infos_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8ResultAnnotationInfo {");
      builder.append("detection_annotation_infos=");
      builder.append(this.detection_annotation_infos_);
      builder.append("}");
      return builder.toString();
   }
}
