package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A manually placed perception scene node
       * The topic name identifies the node.
       */
public class ManuallyPlacedSceneNodeMessage extends Packet<ManuallyPlacedSceneNodeMessage> implements Settable<ManuallyPlacedSceneNodeMessage>, EpsilonComparable<ManuallyPlacedSceneNodeMessage>
{
   /**
            * The name of the scene node
            */
   public java.lang.StringBuilder name_;
   /**
            * Transform of the node's frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;

   public ManuallyPlacedSceneNodeMessage()
   {
      name_ = new java.lang.StringBuilder(255);
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public ManuallyPlacedSceneNodeMessage(ManuallyPlacedSceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(ManuallyPlacedSceneNodeMessage other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
   }

   /**
            * The name of the scene node
            */
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   /**
            * The name of the scene node
            */
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   /**
            * The name of the scene node
            */
   public java.lang.StringBuilder getName()
   {
      return name_;
   }


   /**
            * Transform of the node's frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
   }


   public static Supplier<ManuallyPlacedSceneNodeMessagePubSubType> getPubSubType()
   {
      return ManuallyPlacedSceneNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ManuallyPlacedSceneNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ManuallyPlacedSceneNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ManuallyPlacedSceneNodeMessage)) return false;

      ManuallyPlacedSceneNodeMessage otherMyClass = (ManuallyPlacedSceneNodeMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ManuallyPlacedSceneNodeMessage {");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
