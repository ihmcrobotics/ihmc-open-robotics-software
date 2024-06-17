package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A detectable perception scene node
       * The topic name identifies the node.
       */
public class DetectableSceneNodeMessage extends Packet<DetectableSceneNodeMessage> implements Settable<DetectableSceneNodeMessage>, EpsilonComparable<DetectableSceneNodeMessage>
{
   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.SceneNodeMessage scene_node_;
   /**
            * Instant detection information
            */
   public perception_msgs.msg.dds.InstantDetectionMessage instant_detection_;
   /**
            * Whether or not the node is currently detected
            */
   public boolean currently_detected_;

   public DetectableSceneNodeMessage()
   {
      scene_node_ = new perception_msgs.msg.dds.SceneNodeMessage();
      instant_detection_ = new perception_msgs.msg.dds.InstantDetectionMessage();
   }

   public DetectableSceneNodeMessage(DetectableSceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectableSceneNodeMessage other)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.staticCopy(other.scene_node_, scene_node_);
      perception_msgs.msg.dds.InstantDetectionMessagePubSubType.staticCopy(other.instant_detection_, instant_detection_);
      currently_detected_ = other.currently_detected_;

   }


   /**
            * The base scene node information
            */
   public perception_msgs.msg.dds.SceneNodeMessage getSceneNode()
   {
      return scene_node_;
   }


   /**
            * Instant detection information
            */
   public perception_msgs.msg.dds.InstantDetectionMessage getInstantDetection()
   {
      return instant_detection_;
   }

   /**
            * Whether or not the node is currently detected
            */
   public void setCurrentlyDetected(boolean currently_detected)
   {
      currently_detected_ = currently_detected;
   }
   /**
            * Whether or not the node is currently detected
            */
   public boolean getCurrentlyDetected()
   {
      return currently_detected_;
   }


   public static Supplier<DetectableSceneNodeMessagePubSubType> getPubSubType()
   {
      return DetectableSceneNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectableSceneNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectableSceneNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.scene_node_.epsilonEquals(other.scene_node_, epsilon)) return false;
      if (!this.instant_detection_.epsilonEquals(other.instant_detection_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.currently_detected_, other.currently_detected_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectableSceneNodeMessage)) return false;

      DetectableSceneNodeMessage otherMyClass = (DetectableSceneNodeMessage) other;

      if (!this.scene_node_.equals(otherMyClass.scene_node_)) return false;
      if (!this.instant_detection_.equals(otherMyClass.instant_detection_)) return false;
      if(this.currently_detected_ != otherMyClass.currently_detected_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectableSceneNodeMessage {");
      builder.append("scene_node=");
      builder.append(this.scene_node_);      builder.append(", ");
      builder.append("instant_detection=");
      builder.append(this.instant_detection_);      builder.append(", ");
      builder.append("currently_detected=");
      builder.append(this.currently_detected_);
      builder.append("}");
      return builder.toString();
   }
}
