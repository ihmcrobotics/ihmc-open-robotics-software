package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message representing an InstantDetection of any type.
       */
public class InstantDetectionMessage extends Packet<InstantDetectionMessage> implements Settable<InstantDetectionMessage>, EpsilonComparable<InstantDetectionMessage>
{
   /**
            * Standard InstantDetection fields:
            */
   public java.lang.StringBuilder detected_object_class_;
   public java.lang.StringBuilder detected_object_name_;
   public double confidence_;
   public us.ihmc.euclid.geometry.Pose3D object_pose_;
   public ihmc_common_msgs.msg.dds.InstantMessage detection_time_;
   /**
            * YOLOv8InstantDetection fields:
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  yolo_object_point_cloud_;
   /**
            * CenterPoseInstantDetection fields:
            */
   public us.ihmc.euclid.tuple3D.Point3D[] center_pose_bounding_box_2d_vertices_;
   public us.ihmc.euclid.tuple3D.Point3D[] center_pose_bounding_box_vertices_;

   public InstantDetectionMessage()
   {
      detected_object_class_ = new java.lang.StringBuilder(255);
      detected_object_name_ = new java.lang.StringBuilder(255);
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      detection_time_ = new ihmc_common_msgs.msg.dds.InstantMessage();
      yolo_object_point_cloud_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32> (5000, new geometry_msgs.msg.dds.Point32PubSubType());
      center_pose_bounding_box_2d_vertices_ = new us.ihmc.euclid.tuple3D.Point3D[8];

      for(int i1 = 0; i1 < center_pose_bounding_box_2d_vertices_.length; ++i1)
      {
          center_pose_bounding_box_2d_vertices_[i1] = new us.ihmc.euclid.tuple3D.Point3D();
      }
      center_pose_bounding_box_vertices_ = new us.ihmc.euclid.tuple3D.Point3D[8];

      for(int i3 = 0; i3 < center_pose_bounding_box_vertices_.length; ++i3)
      {
          center_pose_bounding_box_vertices_[i3] = new us.ihmc.euclid.tuple3D.Point3D();
      }

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
      yolo_object_point_cloud_.set(other.yolo_object_point_cloud_);
      for(int i5 = 0; i5 < center_pose_bounding_box_2d_vertices_.length; ++i5)
      {
            geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.center_pose_bounding_box_2d_vertices_[i5], center_pose_bounding_box_2d_vertices_[i5]);}

      for(int i7 = 0; i7 < center_pose_bounding_box_vertices_.length; ++i7)
      {
            geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.center_pose_bounding_box_vertices_[i7], center_pose_bounding_box_vertices_[i7]);}

   }

   /**
            * Standard InstantDetection fields:
            */
   public void setDetectedObjectClass(java.lang.String detected_object_class)
   {
      detected_object_class_.setLength(0);
      detected_object_class_.append(detected_object_class);
   }

   /**
            * Standard InstantDetection fields:
            */
   public java.lang.String getDetectedObjectClassAsString()
   {
      return getDetectedObjectClass().toString();
   }
   /**
            * Standard InstantDetection fields:
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


   /**
            * YOLOv8InstantDetection fields:
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  getYoloObjectPointCloud()
   {
      return yolo_object_point_cloud_;
   }


   /**
            * CenterPoseInstantDetection fields:
            */
   public us.ihmc.euclid.tuple3D.Point3D[] getCenterPoseBoundingBox2dVertices()
   {
      return center_pose_bounding_box_2d_vertices_;
   }


   public us.ihmc.euclid.tuple3D.Point3D[] getCenterPoseBoundingBoxVertices()
   {
      return center_pose_bounding_box_vertices_;
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
      if (this.yolo_object_point_cloud_.size() != other.yolo_object_point_cloud_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.yolo_object_point_cloud_.size(); i++)
         {  if (!this.yolo_object_point_cloud_.get(i).epsilonEquals(other.yolo_object_point_cloud_.get(i), epsilon)) return false; }
      }

      for(int i9 = 0; i9 < center_pose_bounding_box_2d_vertices_.length; ++i9)
      {
              if (!this.center_pose_bounding_box_2d_vertices_[i9].epsilonEquals(other.center_pose_bounding_box_2d_vertices_[i9], epsilon)) return false;
      }

      for(int i11 = 0; i11 < center_pose_bounding_box_vertices_.length; ++i11)
      {
              if (!this.center_pose_bounding_box_vertices_[i11].epsilonEquals(other.center_pose_bounding_box_vertices_[i11], epsilon)) return false;
      }


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
      if (!this.yolo_object_point_cloud_.equals(otherMyClass.yolo_object_point_cloud_)) return false;
      for(int i13 = 0; i13 < center_pose_bounding_box_2d_vertices_.length; ++i13)
      {
                if (!this.center_pose_bounding_box_2d_vertices_[i13].equals(otherMyClass.center_pose_bounding_box_2d_vertices_[i13])) return false;
      }
      for(int i15 = 0; i15 < center_pose_bounding_box_vertices_.length; ++i15)
      {
                if (!this.center_pose_bounding_box_vertices_[i15].equals(otherMyClass.center_pose_bounding_box_vertices_[i15])) return false;
      }

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
      builder.append(this.detection_time_);      builder.append(", ");
      builder.append("yolo_object_point_cloud=");
      builder.append(this.yolo_object_point_cloud_);      builder.append(", ");
      builder.append("center_pose_bounding_box_2d_vertices=");
      builder.append(java.util.Arrays.toString(this.center_pose_bounding_box_2d_vertices_));      builder.append(", ");
      builder.append("center_pose_bounding_box_vertices=");
      builder.append(java.util.Arrays.toString(this.center_pose_bounding_box_vertices_));
      builder.append("}");
      return builder.toString();
   }
}
