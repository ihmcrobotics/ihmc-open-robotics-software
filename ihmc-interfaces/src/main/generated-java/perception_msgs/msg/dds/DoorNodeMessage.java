package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DoorNodeMessage extends Packet<DoorNodeMessage> implements Settable<DoorNodeMessage>, EpsilonComparable<DoorNodeMessage>
{
   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   /**
            * The transform of the frame of the door corner (X points in the direction the door opens)
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform door_corner_transform_to_world_;
   /**
            * Whether the pose of the door frame is locked
            */
   public boolean pose_locked_;
   /**
            * The door panel
            */
   public perception_msgs.msg.dds.DoorPanelMessage door_panel_;
   /**
            * The door opening mechanisms
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DoorOpeningMechanismMessage>  opening_mechanisms_;

   public DoorNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
      door_corner_transform_to_world_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
      door_panel_ = new perception_msgs.msg.dds.DoorPanelMessage();
      opening_mechanisms_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DoorOpeningMechanismMessage> (100, new perception_msgs.msg.dds.DoorOpeningMechanismMessagePubSubType());

   }

   public DoorNodeMessage(DoorNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.door_corner_transform_to_world_, door_corner_transform_to_world_);
      pose_locked_ = other.pose_locked_;

      perception_msgs.msg.dds.DoorPanelMessagePubSubType.staticCopy(other.door_panel_, door_panel_);
      opening_mechanisms_.set(other.opening_mechanisms_);
   }


   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }


   /**
            * The transform of the frame of the door corner (X points in the direction the door opens)
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform getDoorCornerTransformToWorld()
   {
      return door_corner_transform_to_world_;
   }

   /**
            * Whether the pose of the door frame is locked
            */
   public void setPoseLocked(boolean pose_locked)
   {
      pose_locked_ = pose_locked;
   }
   /**
            * Whether the pose of the door frame is locked
            */
   public boolean getPoseLocked()
   {
      return pose_locked_;
   }


   /**
            * The door panel
            */
   public perception_msgs.msg.dds.DoorPanelMessage getDoorPanel()
   {
      return door_panel_;
   }


   /**
            * The door opening mechanisms
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DoorOpeningMechanismMessage>  getOpeningMechanisms()
   {
      return opening_mechanisms_;
   }


   public static Supplier<DoorNodeMessagePubSubType> getPubSubType()
   {
      return DoorNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DoorNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DoorNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!this.door_corner_transform_to_world_.epsilonEquals(other.door_corner_transform_to_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.pose_locked_, other.pose_locked_, epsilon)) return false;

      if (!this.door_panel_.epsilonEquals(other.door_panel_, epsilon)) return false;
      if (this.opening_mechanisms_.size() != other.opening_mechanisms_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.opening_mechanisms_.size(); i++)
         {  if (!this.opening_mechanisms_.get(i).epsilonEquals(other.opening_mechanisms_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DoorNodeMessage)) return false;

      DoorNodeMessage otherMyClass = (DoorNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if (!this.door_corner_transform_to_world_.equals(otherMyClass.door_corner_transform_to_world_)) return false;
      if(this.pose_locked_ != otherMyClass.pose_locked_) return false;

      if (!this.door_panel_.equals(otherMyClass.door_panel_)) return false;
      if (!this.opening_mechanisms_.equals(otherMyClass.opening_mechanisms_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("door_corner_transform_to_world=");
      builder.append(this.door_corner_transform_to_world_);      builder.append(", ");
      builder.append("pose_locked=");
      builder.append(this.pose_locked_);      builder.append(", ");
      builder.append("door_panel=");
      builder.append(this.door_panel_);      builder.append(", ");
      builder.append("opening_mechanisms=");
      builder.append(this.opening_mechanisms_);
      builder.append("}");
      return builder.toString();
   }
}
