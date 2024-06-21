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
   public perception_msgs.msg.dds.SceneNodeMessage scene_node_;
   public us.ihmc.euclid.geometry.Pose3D door_frame_pose_;
   public perception_msgs.msg.dds.DoorPanelMessage door_panel_;
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DoorOpeningMechanismMessage>  opening_mechanisms_;

   public DoorNodeMessage()
   {
      scene_node_ = new perception_msgs.msg.dds.SceneNodeMessage();
      door_frame_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      door_panel_ = new perception_msgs.msg.dds.DoorPanelMessage();
      opening_mechanisms_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DoorOpeningMechanismMessage> (8, new perception_msgs.msg.dds.DoorOpeningMechanismMessagePubSubType());

   }

   public DoorNodeMessage(DoorNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorNodeMessage other)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.staticCopy(other.scene_node_, scene_node_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.door_frame_pose_, door_frame_pose_);
      perception_msgs.msg.dds.DoorPanelMessagePubSubType.staticCopy(other.door_panel_, door_panel_);
      opening_mechanisms_.set(other.opening_mechanisms_);
   }


   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.SceneNodeMessage getSceneNode()
   {
      return scene_node_;
   }


   public us.ihmc.euclid.geometry.Pose3D getDoorFramePose()
   {
      return door_frame_pose_;
   }


   public perception_msgs.msg.dds.DoorPanelMessage getDoorPanel()
   {
      return door_panel_;
   }


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

      if (!this.scene_node_.epsilonEquals(other.scene_node_, epsilon)) return false;
      if (!this.door_frame_pose_.epsilonEquals(other.door_frame_pose_, epsilon)) return false;
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

      if (!this.scene_node_.equals(otherMyClass.scene_node_)) return false;
      if (!this.door_frame_pose_.equals(otherMyClass.door_frame_pose_)) return false;
      if (!this.door_panel_.equals(otherMyClass.door_panel_)) return false;
      if (!this.opening_mechanisms_.equals(otherMyClass.opening_mechanisms_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorNodeMessage {");
      builder.append("scene_node=");
      builder.append(this.scene_node_);      builder.append(", ");
      builder.append("door_frame_pose=");
      builder.append(this.door_frame_pose_);      builder.append(", ");
      builder.append("door_panel=");
      builder.append(this.door_panel_);      builder.append(", ");
      builder.append("opening_mechanisms=");
      builder.append(this.opening_mechanisms_);
      builder.append("}");
      return builder.toString();
   }
}
