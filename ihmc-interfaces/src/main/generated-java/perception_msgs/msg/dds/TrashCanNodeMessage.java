package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TrashCanNodeMessage extends Packet<TrashCanNodeMessage> implements Settable<TrashCanNodeMessage>, EpsilonComparable<TrashCanNodeMessage>
{
   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   /**
            * The below transforms may be NaN if not determined
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform trash_can_to_world_transform_;
   public double trash_can_yaw_;

   public TrashCanNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      trash_can_to_world_transform_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
   }

   public TrashCanNodeMessage(TrashCanNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(TrashCanNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.trash_can_to_world_transform_, trash_can_to_world_transform_);
      trash_can_yaw_ = other.trash_can_yaw_;

   }


   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }


   /**
            * The below transforms may be NaN if not determined
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform getTrashCanToWorldTransform()
   {
      return trash_can_to_world_transform_;
   }

   public void setTrashCanYaw(double trash_can_yaw)
   {
      trash_can_yaw_ = trash_can_yaw;
   }
   public double getTrashCanYaw()
   {
      return trash_can_yaw_;
   }


   public static Supplier<TrashCanNodeMessagePubSubType> getPubSubType()
   {
      return TrashCanNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TrashCanNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TrashCanNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!this.trash_can_to_world_transform_.epsilonEquals(other.trash_can_to_world_transform_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trash_can_yaw_, other.trash_can_yaw_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TrashCanNodeMessage)) return false;

      TrashCanNodeMessage otherMyClass = (TrashCanNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if (!this.trash_can_to_world_transform_.equals(otherMyClass.trash_can_to_world_transform_)) return false;
      if(this.trash_can_yaw_ != otherMyClass.trash_can_yaw_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TrashCanNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("trash_can_to_world_transform=");
      builder.append(this.trash_can_to_world_transform_);      builder.append(", ");
      builder.append("trash_can_yaw=");
      builder.append(this.trash_can_yaw_);
      builder.append("}");
      return builder.toString();
   }
}
