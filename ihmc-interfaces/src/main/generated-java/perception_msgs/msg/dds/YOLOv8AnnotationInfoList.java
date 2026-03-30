package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Information used for annotating YOLOv8 results that may contain multiple detections.
       */
public class YOLOv8AnnotationInfoList extends Packet<YOLOv8AnnotationInfoList> implements Settable<YOLOv8AnnotationInfoList>, EpsilonComparable<YOLOv8AnnotationInfoList>
{
   /**
            * Time the detections were made
            */
   public ihmc_common_msgs.msg.dds.InstantMessage detection_instant_;
   /**
            * Detection annotation records
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage>  annotation_infos_;

   public YOLOv8AnnotationInfoList()
   {
      detection_instant_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      annotation_infos_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage> (100, new perception_msgs.msg.dds.YOLOv8AnnotationInfoMessagePubSubType());

   }

   public YOLOv8AnnotationInfoList(YOLOv8AnnotationInfoList other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8AnnotationInfoList other)
   {
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.detection_instant_, detection_instant_);
      annotation_infos_.set(other.annotation_infos_);
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
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.YOLOv8AnnotationInfoMessage>  getAnnotationInfos()
   {
      return annotation_infos_;
   }


   public static Supplier<YOLOv8AnnotationInfoListPubSubType> getPubSubType()
   {
      return YOLOv8AnnotationInfoListPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8AnnotationInfoListPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8AnnotationInfoList other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detection_instant_.epsilonEquals(other.detection_instant_, epsilon)) return false;
      if (this.annotation_infos_.size() != other.annotation_infos_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.annotation_infos_.size(); i++)
         {  if (!this.annotation_infos_.get(i).epsilonEquals(other.annotation_infos_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8AnnotationInfoList)) return false;

      YOLOv8AnnotationInfoList otherMyClass = (YOLOv8AnnotationInfoList) other;

      if (!this.detection_instant_.equals(otherMyClass.detection_instant_)) return false;
      if (!this.annotation_infos_.equals(otherMyClass.annotation_infos_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8AnnotationInfoList {");
      builder.append("detection_instant=");
      builder.append(this.detection_instant_);      builder.append(", ");
      builder.append("annotation_infos=");
      builder.append(this.annotation_infos_);
      builder.append("}");
      return builder.toString();
   }
}
