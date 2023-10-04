package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A reshapable rigid body scene node
       * The topic name identifies the node.
       */
public class ReshapableRigidBodySceneNodeMessage extends Packet<ReshapableRigidBodySceneNodeMessage> implements Settable<ReshapableRigidBodySceneNodeMessage>, EpsilonComparable<ReshapableRigidBodySceneNodeMessage>
{
   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.SceneNodeMessage scene_node_;
   /**
            * Initial parent ID
            */
   public long initial_parent_id_;
   /**
            * Initial transform to initial parent
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage initial_transform_to_parent_;

   public ReshapableRigidBodySceneNodeMessage()
   {
      scene_node_ = new perception_msgs.msg.dds.SceneNodeMessage();
      initial_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public ReshapableRigidBodySceneNodeMessage(ReshapableRigidBodySceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(ReshapableRigidBodySceneNodeMessage other)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.staticCopy(other.scene_node_, scene_node_);
      initial_parent_id_ = other.initial_parent_id_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.initial_transform_to_parent_, initial_transform_to_parent_);
   }


   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.SceneNodeMessage getSceneNode()
   {
      return scene_node_;
   }

   /**
            * Initial parent ID
            */
   public void setInitialParentId(long initial_parent_id)
   {
      initial_parent_id_ = initial_parent_id;
   }
   /**
            * Initial parent ID
            */
   public long getInitialParentId()
   {
      return initial_parent_id_;
   }


   /**
            * Initial transform to initial parent
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getInitialTransformToParent()
   {
      return initial_transform_to_parent_;
   }


   public static Supplier<ReshapableRigidBodySceneNodeMessagePubSubType> getPubSubType()
   {
      return ReshapableRigidBodySceneNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ReshapableRigidBodySceneNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ReshapableRigidBodySceneNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.scene_node_.epsilonEquals(other.scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_parent_id_, other.initial_parent_id_, epsilon)) return false;

      if (!this.initial_transform_to_parent_.epsilonEquals(other.initial_transform_to_parent_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ReshapableRigidBodySceneNodeMessage)) return false;

      ReshapableRigidBodySceneNodeMessage otherMyClass = (ReshapableRigidBodySceneNodeMessage) other;

      if (!this.scene_node_.equals(otherMyClass.scene_node_)) return false;
      if(this.initial_parent_id_ != otherMyClass.initial_parent_id_) return false;

      if (!this.initial_transform_to_parent_.equals(otherMyClass.initial_transform_to_parent_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReshapableRigidBodySceneNodeMessage {");
      builder.append("scene_node=");
      builder.append(this.scene_node_);      builder.append(", ");
      builder.append("initial_parent_id=");
      builder.append(this.initial_parent_id_);      builder.append(", ");
      builder.append("initial_transform_to_parent=");
      builder.append(this.initial_transform_to_parent_);
      builder.append("}");
      return builder.toString();
   }
}
