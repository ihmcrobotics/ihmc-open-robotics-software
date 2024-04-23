package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Perception scene node for YOLOv8 detections
       */
public class YOLOv8NodeMessage extends Packet<YOLOv8NodeMessage> implements Settable<YOLOv8NodeMessage>, EpsilonComparable<YOLOv8NodeMessage>
{
   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   /**
            * YOLO parameters
            */
   public int mask_erosion_kernel_radius_;
   public double outlier_filter_threshold_;
   public float detection_acceptance_threshold_;
   /**
            * Detection information
            */
   public java.lang.StringBuilder detection_class_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  object_point_cloud_;
   public us.ihmc.euclid.tuple3D.Point3D32 object_centroid_;
   public us.ihmc.euclid.transform.QuaternionBasedTransform centroid_to_object_transform_;
   public us.ihmc.euclid.geometry.Pose3D object_pose_;
   public us.ihmc.euclid.geometry.Pose3D filtered_object_pose_;
   /**
            * The visual transform to parent
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform visual_transform_to_object_pose_;
   /**
            * Alpha filter for transform interpolation
            */
   public float alpha_filter_;

   public YOLOv8NodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      detection_class_ = new java.lang.StringBuilder(255);
      object_point_cloud_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32> (5000, new geometry_msgs.msg.dds.Point32PubSubType());
      object_centroid_ = new us.ihmc.euclid.tuple3D.Point3D32();
      centroid_to_object_transform_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      filtered_object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      visual_transform_to_object_pose_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();

   }

   public YOLOv8NodeMessage(YOLOv8NodeMessage other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8NodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      mask_erosion_kernel_radius_ = other.mask_erosion_kernel_radius_;

      outlier_filter_threshold_ = other.outlier_filter_threshold_;

      detection_acceptance_threshold_ = other.detection_acceptance_threshold_;

      detection_class_.setLength(0);
      detection_class_.append(other.detection_class_);

      object_point_cloud_.set(other.object_point_cloud_);
      geometry_msgs.msg.dds.Point32PubSubType.staticCopy(other.object_centroid_, object_centroid_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.centroid_to_object_transform_, centroid_to_object_transform_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.filtered_object_pose_, filtered_object_pose_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.visual_transform_to_object_pose_, visual_transform_to_object_pose_);
      alpha_filter_ = other.alpha_filter_;

   }


   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }

   /**
            * YOLO parameters
            */
   public void setMaskErosionKernelRadius(int mask_erosion_kernel_radius)
   {
      mask_erosion_kernel_radius_ = mask_erosion_kernel_radius;
   }
   /**
            * YOLO parameters
            */
   public int getMaskErosionKernelRadius()
   {
      return mask_erosion_kernel_radius_;
   }

   public void setOutlierFilterThreshold(double outlier_filter_threshold)
   {
      outlier_filter_threshold_ = outlier_filter_threshold;
   }
   public double getOutlierFilterThreshold()
   {
      return outlier_filter_threshold_;
   }

   public void setDetectionAcceptanceThreshold(float detection_acceptance_threshold)
   {
      detection_acceptance_threshold_ = detection_acceptance_threshold;
   }
   public float getDetectionAcceptanceThreshold()
   {
      return detection_acceptance_threshold_;
   }

   /**
            * Detection information
            */
   public void setDetectionClass(java.lang.String detection_class)
   {
      detection_class_.setLength(0);
      detection_class_.append(detection_class);
   }

   /**
            * Detection information
            */
   public java.lang.String getDetectionClassAsString()
   {
      return getDetectionClass().toString();
   }
   /**
            * Detection information
            */
   public java.lang.StringBuilder getDetectionClass()
   {
      return detection_class_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  getObjectPointCloud()
   {
      return object_point_cloud_;
   }


   public us.ihmc.euclid.tuple3D.Point3D32 getObjectCentroid()
   {
      return object_centroid_;
   }


   public us.ihmc.euclid.transform.QuaternionBasedTransform getCentroidToObjectTransform()
   {
      return centroid_to_object_transform_;
   }


   public us.ihmc.euclid.geometry.Pose3D getObjectPose()
   {
      return object_pose_;
   }


   public us.ihmc.euclid.geometry.Pose3D getFilteredObjectPose()
   {
      return filtered_object_pose_;
   }


   /**
            * The visual transform to parent
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform getVisualTransformToObjectPose()
   {
      return visual_transform_to_object_pose_;
   }

   /**
            * Alpha filter for transform interpolation
            */
   public void setAlphaFilter(float alpha_filter)
   {
      alpha_filter_ = alpha_filter;
   }
   /**
            * Alpha filter for transform interpolation
            */
   public float getAlphaFilter()
   {
      return alpha_filter_;
   }


   public static Supplier<YOLOv8NodeMessagePubSubType> getPubSubType()
   {
      return YOLOv8NodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return YOLOv8NodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(YOLOv8NodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mask_erosion_kernel_radius_, other.mask_erosion_kernel_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.outlier_filter_threshold_, other.outlier_filter_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.detection_acceptance_threshold_, other.detection_acceptance_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.detection_class_, other.detection_class_, epsilon)) return false;

      if (this.object_point_cloud_.size() != other.object_point_cloud_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.object_point_cloud_.size(); i++)
         {  if (!this.object_point_cloud_.get(i).epsilonEquals(other.object_point_cloud_.get(i), epsilon)) return false; }
      }

      if (!this.object_centroid_.epsilonEquals(other.object_centroid_, epsilon)) return false;
      if (!this.centroid_to_object_transform_.epsilonEquals(other.centroid_to_object_transform_, epsilon)) return false;
      if (!this.object_pose_.epsilonEquals(other.object_pose_, epsilon)) return false;
      if (!this.filtered_object_pose_.epsilonEquals(other.filtered_object_pose_, epsilon)) return false;
      if (!this.visual_transform_to_object_pose_.epsilonEquals(other.visual_transform_to_object_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.alpha_filter_, other.alpha_filter_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof YOLOv8NodeMessage)) return false;

      YOLOv8NodeMessage otherMyClass = (YOLOv8NodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if(this.mask_erosion_kernel_radius_ != otherMyClass.mask_erosion_kernel_radius_) return false;

      if(this.outlier_filter_threshold_ != otherMyClass.outlier_filter_threshold_) return false;

      if(this.detection_acceptance_threshold_ != otherMyClass.detection_acceptance_threshold_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.detection_class_, otherMyClass.detection_class_)) return false;

      if (!this.object_point_cloud_.equals(otherMyClass.object_point_cloud_)) return false;
      if (!this.object_centroid_.equals(otherMyClass.object_centroid_)) return false;
      if (!this.centroid_to_object_transform_.equals(otherMyClass.centroid_to_object_transform_)) return false;
      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;
      if (!this.filtered_object_pose_.equals(otherMyClass.filtered_object_pose_)) return false;
      if (!this.visual_transform_to_object_pose_.equals(otherMyClass.visual_transform_to_object_pose_)) return false;
      if(this.alpha_filter_ != otherMyClass.alpha_filter_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8NodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("mask_erosion_kernel_radius=");
      builder.append(this.mask_erosion_kernel_radius_);      builder.append(", ");
      builder.append("outlier_filter_threshold=");
      builder.append(this.outlier_filter_threshold_);      builder.append(", ");
      builder.append("detection_acceptance_threshold=");
      builder.append(this.detection_acceptance_threshold_);      builder.append(", ");
      builder.append("detection_class=");
      builder.append(this.detection_class_);      builder.append(", ");
      builder.append("object_point_cloud=");
      builder.append(this.object_point_cloud_);      builder.append(", ");
      builder.append("object_centroid=");
      builder.append(this.object_centroid_);      builder.append(", ");
      builder.append("centroid_to_object_transform=");
      builder.append(this.centroid_to_object_transform_);      builder.append(", ");
      builder.append("object_pose=");
      builder.append(this.object_pose_);      builder.append(", ");
      builder.append("filtered_object_pose=");
      builder.append(this.filtered_object_pose_);      builder.append(", ");
      builder.append("visual_transform_to_object_pose=");
      builder.append(this.visual_transform_to_object_pose_);      builder.append(", ");
      builder.append("alpha_filter=");
      builder.append(this.alpha_filter_);
      builder.append("}");
      return builder.toString();
   }
}
