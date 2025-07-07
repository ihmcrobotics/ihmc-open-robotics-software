package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * DetectableSceneNode for FoundationPose tracked objects
       */
public class FoundationPoseNodeMessage extends Packet<FoundationPoseNodeMessage> implements Settable<FoundationPoseNodeMessage>, EpsilonComparable<FoundationPoseNodeMessage>
{
   /**
            * DetectableSceneNode information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   /**
            * Object's pose estimated by FoundationPose
            */
   public us.ihmc.euclid.geometry.Pose3D object_pose_;

   public FoundationPoseNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public FoundationPoseNodeMessage(FoundationPoseNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(FoundationPoseNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
   }


   /**
            * DetectableSceneNode information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }


   /**
            * Object's pose estimated by FoundationPose
            */
   public us.ihmc.euclid.geometry.Pose3D getObjectPose()
   {
      return object_pose_;
   }


   public static Supplier<FoundationPoseNodeMessagePubSubType> getPubSubType()
   {
      return FoundationPoseNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FoundationPoseNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FoundationPoseNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!this.object_pose_.epsilonEquals(other.object_pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FoundationPoseNodeMessage)) return false;

      FoundationPoseNodeMessage otherMyClass = (FoundationPoseNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FoundationPoseNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("object_pose=");
      builder.append(this.object_pose_);
      builder.append("}");
      return builder.toString();
   }
}
