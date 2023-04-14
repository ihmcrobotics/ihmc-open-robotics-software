package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A detectable perception scene object
       * The topic name identifies the object.
       */
public class DetectableSceneObjectMessage extends Packet<DetectableSceneObjectMessage> implements Settable<DetectableSceneObjectMessage>, EpsilonComparable<DetectableSceneObjectMessage>
{
   /**
            * The name of the scene object
            */
   public java.lang.StringBuilder name_;
   /**
            * Whether or not the object is currently detected
            */
   public boolean detected_;
   /**
            * Transform of the object's frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;

   public DetectableSceneObjectMessage()
   {
      name_ = new java.lang.StringBuilder(255);
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public DetectableSceneObjectMessage(DetectableSceneObjectMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectableSceneObjectMessage other)
   {
      name_.setLength(0);
      name_.append(other.name_);

      detected_ = other.detected_;

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
   }

   /**
            * The name of the scene object
            */
   public void setName(java.lang.String name)
   {
      name_.setLength(0);
      name_.append(name);
   }

   /**
            * The name of the scene object
            */
   public java.lang.String getNameAsString()
   {
      return getName().toString();
   }
   /**
            * The name of the scene object
            */
   public java.lang.StringBuilder getName()
   {
      return name_;
   }

   /**
            * Whether or not the object is currently detected
            */
   public void setDetected(boolean detected)
   {
      detected_ = detected;
   }
   /**
            * Whether or not the object is currently detected
            */
   public boolean getDetected()
   {
      return detected_;
   }


   /**
            * Transform of the object's frame to world frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
   }


   public static Supplier<DetectableSceneObjectMessagePubSubType> getPubSubType()
   {
      return DetectableSceneObjectMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectableSceneObjectMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectableSceneObjectMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.name_, other.name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.detected_, other.detected_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectableSceneObjectMessage)) return false;

      DetectableSceneObjectMessage otherMyClass = (DetectableSceneObjectMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.name_, otherMyClass.name_)) return false;

      if(this.detected_ != otherMyClass.detected_) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectableSceneObjectMessage {");
      builder.append("name=");
      builder.append(this.name_);      builder.append(", ");
      builder.append("detected=");
      builder.append(this.detected_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
