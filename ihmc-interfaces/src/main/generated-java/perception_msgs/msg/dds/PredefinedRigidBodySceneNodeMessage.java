package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A predefined rigid body scene node
       * The topic name identifies the node.
       */
public class PredefinedRigidBodySceneNodeMessage extends Packet<PredefinedRigidBodySceneNodeMessage> implements Settable<PredefinedRigidBodySceneNodeMessage>, EpsilonComparable<PredefinedRigidBodySceneNodeMessage>
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
   /**
            * The path to the visual model file
            */
   public java.lang.StringBuilder visual_model_file_path_;
   /**
            * The visual transform to parent
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage visual_transform_to_parent_;
   public boolean left_door_;

   public PredefinedRigidBodySceneNodeMessage()
   {
      scene_node_ = new perception_msgs.msg.dds.SceneNodeMessage();
      initial_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      visual_model_file_path_ = new java.lang.StringBuilder(255);
      visual_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public PredefinedRigidBodySceneNodeMessage(PredefinedRigidBodySceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(PredefinedRigidBodySceneNodeMessage other)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.staticCopy(other.scene_node_, scene_node_);
      initial_parent_id_ = other.initial_parent_id_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.initial_transform_to_parent_, initial_transform_to_parent_);
      visual_model_file_path_.setLength(0);
      visual_model_file_path_.append(other.visual_model_file_path_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.visual_transform_to_parent_, visual_transform_to_parent_);
      left_door_ = other.left_door_;

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

   /**
            * The path to the visual model file
            */
   public void setVisualModelFilePath(java.lang.String visual_model_file_path)
   {
      visual_model_file_path_.setLength(0);
      visual_model_file_path_.append(visual_model_file_path);
   }

   /**
            * The path to the visual model file
            */
   public java.lang.String getVisualModelFilePathAsString()
   {
      return getVisualModelFilePath().toString();
   }
   /**
            * The path to the visual model file
            */
   public java.lang.StringBuilder getVisualModelFilePath()
   {
      return visual_model_file_path_;
   }


   /**
            * The visual transform to parent
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getVisualTransformToParent()
   {
      return visual_transform_to_parent_;
   }

   public void setLeftDoor(boolean left_door)
   {
      left_door_ = left_door;
   }
   public boolean getLeftDoor()
   {
      return left_door_;
   }


   public static Supplier<PredefinedRigidBodySceneNodeMessagePubSubType> getPubSubType()
   {
      return PredefinedRigidBodySceneNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PredefinedRigidBodySceneNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PredefinedRigidBodySceneNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.scene_node_.epsilonEquals(other.scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.initial_parent_id_, other.initial_parent_id_, epsilon)) return false;

      if (!this.initial_transform_to_parent_.epsilonEquals(other.initial_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.visual_model_file_path_, other.visual_model_file_path_, epsilon)) return false;

      if (!this.visual_transform_to_parent_.epsilonEquals(other.visual_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.left_door_, other.left_door_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PredefinedRigidBodySceneNodeMessage)) return false;

      PredefinedRigidBodySceneNodeMessage otherMyClass = (PredefinedRigidBodySceneNodeMessage) other;

      if (!this.scene_node_.equals(otherMyClass.scene_node_)) return false;
      if(this.initial_parent_id_ != otherMyClass.initial_parent_id_) return false;

      if (!this.initial_transform_to_parent_.equals(otherMyClass.initial_transform_to_parent_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.visual_model_file_path_, otherMyClass.visual_model_file_path_)) return false;

      if (!this.visual_transform_to_parent_.equals(otherMyClass.visual_transform_to_parent_)) return false;
      if(this.left_door_ != otherMyClass.left_door_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PredefinedRigidBodySceneNodeMessage {");
      builder.append("scene_node=");
      builder.append(this.scene_node_);      builder.append(", ");
      builder.append("initial_parent_id=");
      builder.append(this.initial_parent_id_);      builder.append(", ");
      builder.append("initial_transform_to_parent=");
      builder.append(this.initial_transform_to_parent_);      builder.append(", ");
      builder.append("visual_model_file_path=");
      builder.append(this.visual_model_file_path_);      builder.append(", ");
      builder.append("visual_transform_to_parent=");
      builder.append(this.visual_transform_to_parent_);      builder.append(", ");
      builder.append("left_door=");
      builder.append(this.left_door_);
      builder.append("}");
      return builder.toString();
   }
}
