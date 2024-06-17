package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class InstantDetectionMessage extends Packet<InstantDetectionMessage> implements Settable<InstantDetectionMessage>, EpsilonComparable<InstantDetectionMessage>
{
   /**
            * Message representing an InstantDetection
            */
   public java.lang.StringBuilder detected_object_class_;
   public java.lang.StringBuilder detected_object_name_;
   public double confidence_;
   public us.ihmc.euclid.geometry.Pose3D object_pose_;
   public ihmc_common_msgs.msg.dds.InstantMessage detection_time_;

   public InstantDetectionMessage()
   {
      detected_object_class_ = new java.lang.StringBuilder(255);
      detected_object_name_ = new java.lang.StringBuilder(255);
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      detection_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
   }

   public InstantDetectionMessage(InstantDetectionMessage other)
   {
      this();
      set(other);
   }

   public void set(InstantDetectionMessage other)
   {
      detected_object_class_.setLength(0);
      detected_object_class_.append(other.detected_object_class_);

      detected_object_name_.setLength(0);
      detected_object_name_.append(other.detected_object_name_);

      confidence_ = other.confidence_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.detection_time_, detection_time_);
   }

   /**
            * Message representing an InstantDetection
            */
   public void setDetectedObjectClass(java.lang.String detected_object_class)
   {
      detected_object_class_.setLength(0);
      detected_object_class_.append(detected_object_class);
   }

   /**
            * Message representing an InstantDetection
            */
   public java.lang.String getDetectedObjectClassAsString()
   {
      return getDetectedObjectClass().toString();
   }
   /**
            * Message representing an InstantDetection
            */
   public java.lang.StringBuilder getDetectedObjectClass()
   {
      return detected_object_class_;
   }

   public void setDetectedObjectName(java.lang.String detected_object_name)
   {
      detected_object_name_.setLength(0);
      detected_object_name_.append(detected_object_name);
   }

   public java.lang.String getDetectedObjectNameAsString()
   {
      return getDetectedObjectName().toString();
   }
   public java.lang.StringBuilder getDetectedObjectName()
   {
      return detected_object_name_;
   }

   public void setConfidence(double confidence)
   {
      confidence_ = confidence;
   }
   public double getConfidence()
   {
      return confidence_;
   }


   public us.ihmc.euclid.geometry.Pose3D getObjectPose()
   {
      return object_pose_;
   }


   public ihmc_common_msgs.msg.dds.InstantMessage getDetectionTime()
   {
      return detection_time_;
   }


   public static Supplier<InstantDetectionMessagePubSubType> getPubSubType()
   {
      return InstantDetectionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return InstantDetectionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(InstantDetectionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.detected_object_class_, other.detected_object_class_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.detected_object_name_, other.detected_object_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.confidence_, other.confidence_, epsilon)) return false;

      if (!this.object_pose_.epsilonEquals(other.object_pose_, epsilon)) return false;
      if (!this.detection_time_.epsilonEquals(other.detection_time_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof InstantDetectionMessage)) return false;

      InstantDetectionMessage otherMyClass = (InstantDetectionMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.detected_object_class_, otherMyClass.detected_object_class_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.detected_object_name_, otherMyClass.detected_object_name_)) return false;

      if(this.confidence_ != otherMyClass.confidence_) return false;

      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;
      if (!this.detection_time_.equals(otherMyClass.detection_time_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("InstantDetectionMessage {");
      builder.append("detected_object_class=");
      builder.append(this.detected_object_class_);      builder.append(", ");
      builder.append("detected_object_name=");
      builder.append(this.detected_object_name_);      builder.append(", ");
      builder.append("confidence=");
      builder.append(this.confidence_);      builder.append(", ");
      builder.append("object_pose=");
      builder.append(this.object_pose_);      builder.append(", ");
      builder.append("detection_time=");
      builder.append(this.detection_time_);
      builder.append("}");
      return builder.toString();
   }
}
