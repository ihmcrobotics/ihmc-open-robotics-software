package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * The base scene node information
       * The topic name identifies the node.
       */
public class SceneNodeMessage extends Packet<SceneNodeMessage> implements Settable<SceneNodeMessage>, EpsilonComparable<SceneNodeMessage>
{
   /**
            * The ID of the node
            */
   public long id_;
   /**
            * The name of the scene node
            */
   public java.lang.StringBuilder name_;
   /**
            * Transform of the node's frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;
   /**
            * Number of child nodes; used for serialization
            */
   public int number_of_children_;

   public SceneNodeMessage()
   {
      name_ = new java.lang.StringBuilder(255);
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public SceneNodeMessage(SceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(SceneNodeMessage other)
   {
      id_ = other.id_;

      name_.setLength(0);
      name_.append(other.name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
      number_of_children_ = other.number_of_children_;

   }

   /**
            * The ID of the node
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * The ID of the node
            */
   public long getId()
   {
      return id_;
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

   /**
            * Number of child nodes; used for serialization
            */
   public void setNumberOfChildren(int number_of_children)
   {
      number_of_children_ = number_of_children;
   }
   /**
            * Number of child nodes; used for serialization
            */
   public int getNumberOfChildren()
   {
      return number_of_children_;
   }


   public static Supplier<SceneNodeMessagePubSubType> getPubSubType()
   {
      return SceneNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SceneNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SceneNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_children_, other.number_of_children_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SceneNodeMessage)) return false;

      SceneNodeMessage otherMyClass = (SceneNodeMessage) other;

      if(this.id_ != otherMyClass.id_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;
      if(this.number_of_children_ != otherMyClass.number_of_children_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SceneNodeMessage {");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);      builder.append(", ");
      builder.append("number_of_children=");
      builder.append(this.number_of_children_);
      builder.append("}");
      return builder.toString();
   }
}
