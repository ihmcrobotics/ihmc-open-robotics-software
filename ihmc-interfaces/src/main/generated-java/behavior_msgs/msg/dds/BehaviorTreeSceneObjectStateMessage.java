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
   /**
            * The timestamp and modifier ID of the latest modification to this object's data fields
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_modification_to_data_;
   /**
            * The object's unique ID
            */
   public long id_;
   /**
            * Persistent detection associated with this object
            */
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage persistent_detection_;
   /**
            * Object type (IsaacROSFoundationPoseObject ordinal)
            */
   public int object_type_;
   /**
            * Transform of the object frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;

   public BehaviorTreeSceneObjectStateMessage()
   {
      latest_modification_to_data_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      persistent_detection_ = new behavior_msgs.msg.dds.PersistentDetectionStatusMessage();
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
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

      behavior_msgs.msg.dds.PersistentDetectionStatusMessagePubSubType.staticCopy(other.persistent_detection_, persistent_detection_);
      object_type_ = other.object_type_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
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


   /**
            * Persistent detection associated with this object
            */
   public behavior_msgs.msg.dds.PersistentDetectionStatusMessage getPersistentDetection()
   {
      return persistent_detection_;
   }

   /**
            * Object type (IsaacROSFoundationPoseObject ordinal)
            */
   public void setObjectType(int object_type)
   {
      object_type_ = object_type;
   }
   /**
            * Object type (IsaacROSFoundationPoseObject ordinal)
            */
   public int getObjectType()
   {
      return object_type_;
   }


   /**
            * Transform of the object frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
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

      if (!this.persistent_detection_.epsilonEquals(other.persistent_detection_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.object_type_, other.object_type_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;

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

      if (!this.persistent_detection_.equals(otherMyClass.persistent_detection_)) return false;
      if(this.object_type_ != otherMyClass.object_type_) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;

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
      builder.append("persistent_detection=");
      builder.append(this.persistent_detection_);      builder.append(", ");
      builder.append("object_type=");
      builder.append(this.object_type_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
