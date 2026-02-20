package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A behavior tree scene object
       */
public class BehaviorTreeSceneObjectStateMessage extends Packet<BehaviorTreeSceneObjectStateMessage> implements Settable<BehaviorTreeSceneObjectStateMessage>, EpsilonComparable<BehaviorTreeSceneObjectStateMessage>
{
   public static final byte DOOR_TYPE_UNKNOWN = (byte) 0;
   public static final byte DOOR_TYPE_PUSH = (byte) 1;
   public static final byte DOOR_TYPE_PULL = (byte) 2;
   /**
            * The timestamp and modifier ID of the latest modification to this object's data fields
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_data_;
   /**
            * The object's unique ID
            */
   public long id_;
   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage definition_;
   /**
            * Persistent detection associated with this object
            */
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage persistent_detection_;
   /**
            * Transform of the object frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;
   /**
            * Used only for door panel
            */
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage door_panel_detection_;
   /**
            * Used only for door frame
            */
   public long points_in_capsule_;
   /**
            * The open angle of the door hinge (0 = closed, Zup yaw)
            */
   public float door_open_angle_;
   /**
            * Door type as defined above
            */
   public byte door_type_;
   /**
            * Whether the object is frozen
            */
   public boolean frozen_;

   public BehaviorTreeSceneObjectStateMessage()
   {
      latest_modification_to_data_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage();
      persistent_detection_ = new behavior_msgs.msg.dds.PersistentDetectionStatusMessage();
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      door_panel_detection_ = new behavior_msgs.msg.dds.PersistentDetectionStatusMessage();
   }

   public BehaviorTreeSceneObjectStateMessage(BehaviorTreeSceneObjectStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeSceneObjectStateMessage other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_modification_to_data_, latest_modification_to_data_);
      id_ = other.id_;

      behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.staticCopy(other.persistent_detection_, persistent_detection_);
      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
      behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.staticCopy(other.door_panel_detection_, door_panel_detection_);
      points_in_capsule_ = other.points_in_capsule_;

      door_open_angle_ = other.door_open_angle_;

      door_type_ = other.door_type_;

      frozen_ = other.frozen_;

   }


   /**
            * The timestamp and modifier ID of the latest modification to this object's data fields
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestModificationToData()
   {
      return latest_modification_to_data_;
   }

   /**
            * The object's unique ID
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * The object's unique ID
            */
   public long getId()
   {
      return id_;
   }


   public behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * Persistent detection associated with this object
            */
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage getPersistentDetection()
   {
      return persistent_detection_;
   }


   /**
            * Transform of the object frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
   }


   /**
            * Used only for door panel
            */
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage getDoorPanelDetection()
   {
      return door_panel_detection_;
   }

   /**
            * Used only for door frame
            */
   public void setPointsInCapsule(long points_in_capsule)
   {
      points_in_capsule_ = points_in_capsule;
   }
   /**
            * Used only for door frame
            */
   public long getPointsInCapsule()
   {
      return points_in_capsule_;
   }

   /**
            * The open angle of the door hinge (0 = closed, Zup yaw)
            */
   public void setDoorOpenAngle(float door_open_angle)
   {
      door_open_angle_ = door_open_angle;
   }
   /**
            * The open angle of the door hinge (0 = closed, Zup yaw)
            */
   public float getDoorOpenAngle()
   {
      return door_open_angle_;
   }

   /**
            * Door type as defined above
            */
   public void setDoorType(byte door_type)
   {
      door_type_ = door_type;
   }
   /**
            * Door type as defined above
            */
   public byte getDoorType()
   {
      return door_type_;
   }

   /**
            * Whether the object is frozen
            */
   public void setFrozen(boolean frozen)
   {
      frozen_ = frozen;
   }
   /**
            * Whether the object is frozen
            */
   public boolean getFrozen()
   {
      return frozen_;
   }


   public static Supplier<BehaviorTreeSceneObjectStateMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeSceneObjectStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeSceneObjectStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeSceneObjectStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_modification_to_data_.epsilonEquals(other.latest_modification_to_data_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!this.persistent_detection_.epsilonEquals(other.persistent_detection_, epsilon)) return false;
      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;
      if (!this.door_panel_detection_.epsilonEquals(other.door_panel_detection_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.points_in_capsule_, other.points_in_capsule_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.door_open_angle_, other.door_open_angle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.door_type_, other.door_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.frozen_, other.frozen_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeSceneObjectStateMessage)) return false;

      BehaviorTreeSceneObjectStateMessage otherMyClass = (BehaviorTreeSceneObjectStateMessage) other;

      if (!this.latest_modification_to_data_.equals(otherMyClass.latest_modification_to_data_)) return false;
      if(this.id_ != otherMyClass.id_) return false;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.persistent_detection_.equals(otherMyClass.persistent_detection_)) return false;
      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;
      if (!this.door_panel_detection_.equals(otherMyClass.door_panel_detection_)) return false;
      if(this.points_in_capsule_ != otherMyClass.points_in_capsule_) return false;

      if(this.door_open_angle_ != otherMyClass.door_open_angle_) return false;

      if(this.door_type_ != otherMyClass.door_type_) return false;

      if(this.frozen_ != otherMyClass.frozen_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeSceneObjectStateMessage {");
      builder.append("latest_modification_to_data=");
      builder.append(this.latest_modification_to_data_);      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("persistent_detection=");
      builder.append(this.persistent_detection_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);      builder.append(", ");
      builder.append("door_panel_detection=");
      builder.append(this.door_panel_detection_);      builder.append(", ");
      builder.append("points_in_capsule=");
      builder.append(this.points_in_capsule_);      builder.append(", ");
      builder.append("door_open_angle=");
      builder.append(this.door_open_angle_);      builder.append(", ");
      builder.append("door_type=");
      builder.append(this.door_type_);      builder.append(", ");
      builder.append("frozen=");
      builder.append(this.frozen_);
      builder.append("}");
      return builder.toString();
   }
}
