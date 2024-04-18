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
   /**
            * The type of door opening mechanism detected
            */
   public byte opening_mechanism_type_;
   /**
            * The 3D point of the detected door hardware
            */
   public us.ihmc.euclid.tuple3D.Point3D opening_mechanism_point_;
   /**
            * The pose of the detected door hardware wrt the door planar region
            */
   public us.ihmc.euclid.geometry.Pose3D opening_mechanism_pose_;
   /**
            * The planar region we assume is the door panel
            */
   public perception_msgs.msg.dds.PlanarRegionMessage door_planar_region_;
   /**
            * The last time the door planar region was updated
            */
   public long door_planar_region_update_time_millis_;

   public DoorNodeMessage()
   {
      scene_node_ = new perception_msgs.msg.dds.SceneNodeMessage();
      opening_mechanism_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      opening_mechanism_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      door_planar_region_ = new perception_msgs.msg.dds.PlanarRegionMessage();
   }

   public DoorNodeMessage(DoorNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(DoorNodeMessage other)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.staticCopy(other.scene_node_, scene_node_);
      opening_mechanism_type_ = other.opening_mechanism_type_;

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.opening_mechanism_point_, opening_mechanism_point_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.opening_mechanism_pose_, opening_mechanism_pose_);
      perception_msgs.msg.dds.PlanarRegionMessagePubSubType.staticCopy(other.door_planar_region_, door_planar_region_);
      door_planar_region_update_time_millis_ = other.door_planar_region_update_time_millis_;

   }


   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.SceneNodeMessage getSceneNode()
   {
      return scene_node_;
   }

   /**
            * The type of door opening mechanism detected
            */
   public void setOpeningMechanismType(byte opening_mechanism_type)
   {
      opening_mechanism_type_ = opening_mechanism_type;
   }
   /**
            * The type of door opening mechanism detected
            */
   public byte getOpeningMechanismType()
   {
      return opening_mechanism_type_;
   }


   /**
            * The 3D point of the detected door hardware
            */
   public us.ihmc.euclid.tuple3D.Point3D getOpeningMechanismPoint()
   {
      return opening_mechanism_point_;
   }


   /**
            * The pose of the detected door hardware wrt the door planar region
            */
   public us.ihmc.euclid.geometry.Pose3D getOpeningMechanismPose()
   {
      return opening_mechanism_pose_;
   }


   /**
            * The planar region we assume is the door panel
            */
   public perception_msgs.msg.dds.PlanarRegionMessage getDoorPlanarRegion()
   {
      return door_planar_region_;
   }

   /**
            * The last time the door planar region was updated
            */
   public void setDoorPlanarRegionUpdateTimeMillis(long door_planar_region_update_time_millis)
   {
      door_planar_region_update_time_millis_ = door_planar_region_update_time_millis;
   }
   /**
            * The last time the door planar region was updated
            */
   public long getDoorPlanarRegionUpdateTimeMillis()
   {
      return door_planar_region_update_time_millis_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.opening_mechanism_type_, other.opening_mechanism_type_, epsilon)) return false;

      if (!this.opening_mechanism_point_.epsilonEquals(other.opening_mechanism_point_, epsilon)) return false;
      if (!this.opening_mechanism_pose_.epsilonEquals(other.opening_mechanism_pose_, epsilon)) return false;
      if (!this.door_planar_region_.epsilonEquals(other.door_planar_region_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.door_planar_region_update_time_millis_, other.door_planar_region_update_time_millis_, epsilon)) return false;


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
      if(this.opening_mechanism_type_ != otherMyClass.opening_mechanism_type_) return false;

      if (!this.opening_mechanism_point_.equals(otherMyClass.opening_mechanism_point_)) return false;
      if (!this.opening_mechanism_pose_.equals(otherMyClass.opening_mechanism_pose_)) return false;
      if (!this.door_planar_region_.equals(otherMyClass.door_planar_region_)) return false;
      if(this.door_planar_region_update_time_millis_ != otherMyClass.door_planar_region_update_time_millis_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DoorNodeMessage {");
      builder.append("scene_node=");
      builder.append(this.scene_node_);      builder.append(", ");
      builder.append("opening_mechanism_type=");
      builder.append(this.opening_mechanism_type_);      builder.append(", ");
      builder.append("opening_mechanism_point=");
      builder.append(this.opening_mechanism_point_);      builder.append(", ");
      builder.append("opening_mechanism_pose=");
      builder.append(this.opening_mechanism_pose_);      builder.append(", ");
      builder.append("door_planar_region=");
      builder.append(this.door_planar_region_);      builder.append(", ");
      builder.append("door_planar_region_update_time_millis=");
      builder.append(this.door_planar_region_update_time_millis_);
      builder.append("}");
      return builder.toString();
   }
}
