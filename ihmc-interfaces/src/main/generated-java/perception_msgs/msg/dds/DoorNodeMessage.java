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
   public byte door_hardware_type_;
   public us.ihmc.euclid.geometry.Pose3D object_pose_;
   /**
            * The visual transform to parent
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform visual_transform_to_object_pose_;
   public perception_msgs.msg.dds.PlanarRegionMessage door_planar_region_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  presumed_door_point_cloud_;

   public DoorNodeMessage()
   {
      scene_node_ = new perception_msgs.msg.dds.SceneNodeMessage();
      object_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      visual_transform_to_object_pose_ = new us.ihmc.euclid.transform.QuaternionBasedTransform();
      door_planar_region_ = new perception_msgs.msg.dds.PlanarRegionMessage();
      presumed_door_point_cloud_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32> (5000, new geometry_msgs.msg.dds.Point32PubSubType());

   }

   public DoorNodeMessage(DoorNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorNodeMessage other)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.staticCopy(other.scene_node_, scene_node_);
      door_hardware_type_ = other.door_hardware_type_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.object_pose_, object_pose_);
      geometry_msgs.msg.dds.TransformPubSubType.staticCopy(other.visual_transform_to_object_pose_, visual_transform_to_object_pose_);
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.staticCopy(other.door_planar_region_, door_planar_region_);
      presumed_door_point_cloud_.set(other.presumed_door_point_cloud_);
   }


   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.SceneNodeMessage getSceneNode()
   {
      return scene_node_;
   }

   public void setDoorHardwareType(byte door_hardware_type)
   {
      door_hardware_type_ = door_hardware_type;
   }
   public byte getDoorHardwareType()
   {
      return door_hardware_type_;
   }


   public us.ihmc.euclid.geometry.Pose3D getObjectPose()
   {
      return object_pose_;
   }


   /**
            * The visual transform to parent
            */
   public us.ihmc.euclid.transform.QuaternionBasedTransform getVisualTransformToObjectPose()
   {
      return visual_transform_to_object_pose_;
   }


   public perception_msgs.msg.dds.PlanarRegionMessage getDoorPlanarRegion()
   {
      return door_planar_region_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D32>  getPresumedDoorPointCloud()
   {
      return presumed_door_point_cloud_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.door_hardware_type_, other.door_hardware_type_, epsilon)) return false;

      if (!this.object_pose_.epsilonEquals(other.object_pose_, epsilon)) return false;
      if (!this.visual_transform_to_object_pose_.epsilonEquals(other.visual_transform_to_object_pose_, epsilon)) return false;
      if (!this.door_planar_region_.epsilonEquals(other.door_planar_region_, epsilon)) return false;
      if (this.presumed_door_point_cloud_.size() != other.presumed_door_point_cloud_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.presumed_door_point_cloud_.size(); i++)
         {  if (!this.presumed_door_point_cloud_.get(i).epsilonEquals(other.presumed_door_point_cloud_.get(i), epsilon)) return false; }
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
      if(this.door_hardware_type_ != otherMyClass.door_hardware_type_) return false;

      if (!this.object_pose_.equals(otherMyClass.object_pose_)) return false;
      if (!this.visual_transform_to_object_pose_.equals(otherMyClass.visual_transform_to_object_pose_)) return false;
      if (!this.door_planar_region_.equals(otherMyClass.door_planar_region_)) return false;
      if (!this.presumed_door_point_cloud_.equals(otherMyClass.presumed_door_point_cloud_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorNodeMessage {");
      builder.append("scene_node=");
      builder.append(this.scene_node_);      builder.append(", ");
      builder.append("door_hardware_type=");
      builder.append(this.door_hardware_type_);      builder.append(", ");
      builder.append("object_pose=");
      builder.append(this.object_pose_);      builder.append(", ");
      builder.append("visual_transform_to_object_pose=");
      builder.append(this.visual_transform_to_object_pose_);      builder.append(", ");
      builder.append("door_planar_region=");
      builder.append(this.door_planar_region_);      builder.append(", ");
      builder.append("presumed_door_point_cloud=");
      builder.append(this.presumed_door_point_cloud_);
      builder.append("}");
      return builder.toString();
   }
}
