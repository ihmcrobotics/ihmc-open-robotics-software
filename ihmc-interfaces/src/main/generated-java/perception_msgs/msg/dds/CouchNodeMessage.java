package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class CouchNodeMessage extends Packet<CouchNodeMessage> implements Settable<CouchNodeMessage>, EpsilonComparable<CouchNodeMessage>
{
   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   public us.ihmc.euclid.transform.QuaternionBasedTransform couch_centroid_to_world_transform_;
   public us.ihmc.euclid.transform.QuaternionBasedTransform pillow_to_world_transform_;

   public CouchNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      couch_centroid_to_world_transform_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
      pillow_to_world_transform_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
   }

   public CouchNodeMessage(CouchNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(CouchNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.couch_centroid_to_world_transform_, couch_centroid_to_world_transform_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.pillow_to_world_transform_, pillow_to_world_transform_);
   }


   /**
            * Detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }


   public us.ihmc.euclid.transform.QuaternionBasedTransform getCouchCentroidToWorldTransform()
   {
      return couch_centroid_to_world_transform_;
   }


   public us.ihmc.euclid.transform.QuaternionBasedTransform getPillowToWorldTransform()
   {
      return pillow_to_world_transform_;
   }


   public static Supplier<CouchNodeMessagePubSubType> getPubSubType()
   {
      return CouchNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CouchNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CouchNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!this.couch_centroid_to_world_transform_.epsilonEquals(other.couch_centroid_to_world_transform_, epsilon)) return false;
      if (!this.pillow_to_world_transform_.epsilonEquals(other.pillow_to_world_transform_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CouchNodeMessage)) return false;

      CouchNodeMessage otherMyClass = (CouchNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if (!this.couch_centroid_to_world_transform_.equals(otherMyClass.couch_centroid_to_world_transform_)) return false;
      if (!this.pillow_to_world_transform_.equals(otherMyClass.pillow_to_world_transform_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CouchNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("couch_centroid_to_world_transform=");
      builder.append(this.couch_centroid_to_world_transform_);      builder.append(", ");
      builder.append("pillow_to_world_transform=");
      builder.append(this.pillow_to_world_transform_);
      builder.append("}");
      return builder.toString();
   }
}
