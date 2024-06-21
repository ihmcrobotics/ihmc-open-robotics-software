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
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PersistentDetectionMessage>  detections_;
   /**
            * Whether or not the node is currently detected
            */
   public boolean currently_detected_;

   public DetectableSceneNodeMessage()
   {
      scene_node_ = new perception_msgs.msg.dds.SceneNodeMessage();
      detections_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PersistentDetectionMessage> (50, new perception_msgs.msg.dds.PersistentDetectionMessagePubSubType());

   }

   public DetectableSceneNodeMessage(DetectableSceneNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectableSceneNodeMessage other)
   {
      perception_msgs.msg.dds.SceneNodeMessagePubSubType.staticCopy(other.scene_node_, scene_node_);
      detections_.set(other.detections_);
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
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.PersistentDetectionMessage>  getDetections()
   {
      return detections_;
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
      if (this.detections_.size() != other.detections_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detections_.size(); i++)
         {  if (!this.detections_.get(i).epsilonEquals(other.detections_.get(i), epsilon)) return false; }
      }

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
      if (!this.detections_.equals(otherMyClass.detections_)) return false;
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
      builder.append("detections=");
      builder.append(this.detections_);      builder.append(", ");
      builder.append("currently_detected=");
      builder.append(this.currently_detected_);
      builder.append("}");
      return builder.toString();
   }
}
