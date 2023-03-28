package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * The topic name identifies the object
       */
public class DetectedObjectMessage extends Packet<DetectedObjectMessage> implements Settable<DetectedObjectMessage>, EpsilonComparable<DetectedObjectMessage>
{
   /**
            * Whether or not the object was detected this frame
            */
   public boolean detected_;
   /**
            * Unique ID used to identify the object
            */
   public java.lang.StringBuilder id_;
   /**
            * Transform to world
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage transform_to_world_;

   public DetectedObjectMessage()
   {
      id_ = new java.lang.StringBuilder(255);
      transform_to_world_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public DetectedObjectMessage(DetectedObjectMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectedObjectMessage other)
   {
      detected_ = other.detected_;

      id_.setLength(0);
      id_.append(other.id_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.transform_to_world_, transform_to_world_);
   }

   /**
            * Whether or not the object was detected this frame
            */
   public void setDetected(boolean detected)
   {
      detected_ = detected;
   }
   /**
            * Whether or not the object was detected this frame
            */
   public boolean getDetected()
   {
      return detected_;
   }

   /**
            * Unique ID used to identify the object
            */
   public void setId(java.lang.String id)
   {
      id_.setLength(0);
      id_.append(id);
   }

   /**
            * Unique ID used to identify the object
            */
   public java.lang.String getIdAsString()
   {
      return getId().toString();
   }
   /**
            * Unique ID used to identify the object
            */
   public java.lang.StringBuilder getId()
   {
      return id_;
   }


   /**
            * Transform to world
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getTransformToWorld()
   {
      return transform_to_world_;
   }


   public static Supplier<DetectedObjectMessagePubSubType> getPubSubType()
   {
      return DetectedObjectMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectedObjectMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectedObjectMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.detected_, other.detected_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.id_, other.id_, epsilon)) return false;

      if (!this.transform_to_world_.epsilonEquals(other.transform_to_world_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedObjectMessage)) return false;

      DetectedObjectMessage otherMyClass = (DetectedObjectMessage) other;

      if(this.detected_ != otherMyClass.detected_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.id_, otherMyClass.id_)) return false;

      if (!this.transform_to_world_.equals(otherMyClass.transform_to_world_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedObjectMessage {");
      builder.append("detected=");
      builder.append(this.detected_);      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("transform_to_world=");
      builder.append(this.transform_to_world_);
      builder.append("}");
      return builder.toString();
   }
}
