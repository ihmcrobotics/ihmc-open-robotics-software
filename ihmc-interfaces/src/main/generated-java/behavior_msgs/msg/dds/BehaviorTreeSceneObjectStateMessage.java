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
            * Object type
            */
   public java.lang.StringBuilder type_;
   /**
            * Transform of the object frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;

   public BehaviorTreeSceneObjectStateMessage()
   {
      latest_modification_to_data_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
      type_ = new java.lang.StringBuilder(255);
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

      type_.setLength(0);
      type_.append(other.type_);

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
            * Object type
            */
   public void setType(java.lang.String type)
   {
      type_.setLength(0);
      type_.append(type);
   }

   /**
            * Object type
            */
   public java.lang.String getTypeAsString()
   {
      return getType().toString();
   }
   /**
            * Object type
            */
   public java.lang.StringBuilder getType()
   {
      return type_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.type_, other.type_, epsilon)) return false;

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

      if (!us.ihmc.idl.IDLTools.equals(this.type_, otherMyClass.type_)) return false;

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
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
