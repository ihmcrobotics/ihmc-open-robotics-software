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
            * YOLOv8Node data
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform centroid_to_object_transform_;
   public us.ihmc.euclid.geometry.Pose3D object_pose_;
   public us.ihmc.euclid.geometry.Pose3D filtered_object_pose_;

   public YOLOv8NodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      centroid_to_object_transform_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      filtered_object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public YOLOv8NodeMessage(YOLOv8NodeMessage other)
   {
      this();
      set(other);
   }

   public void set(YOLOv8NodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.centroid_to_object_transform_, centroid_to_object_transform_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.filtered_object_pose_, filtered_object_pose_);
   }


   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }


   /**
            * YOLOv8Node data
            */
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
      if (!this.centroid_to_object_transform_.epsilonEquals(other.centroid_to_object_transform_, epsilon)) return false;
      if (!this.object_pose_.epsilonEquals(other.object_pose_, epsilon)) return false;
      if (!this.filtered_object_pose_.epsilonEquals(other.filtered_object_pose_, epsilon)) return false;

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
      if (!this.centroid_to_object_transform_.equals(otherMyClass.centroid_to_object_transform_)) return false;
      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;
      if (!this.filtered_object_pose_.equals(otherMyClass.filtered_object_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("YOLOv8NodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("centroid_to_object_transform=");
      builder.append(this.centroid_to_object_transform_);      builder.append(", ");
      builder.append("object_pose=");
      builder.append(this.object_pose_);      builder.append(", ");
      builder.append("filtered_object_pose=");
      builder.append(this.filtered_object_pose_);
      builder.append("}");
      return builder.toString();
   }
}
