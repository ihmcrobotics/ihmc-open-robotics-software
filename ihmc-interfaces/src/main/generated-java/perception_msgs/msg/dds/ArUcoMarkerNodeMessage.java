package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * An perception scene node for an ArUco marker
       * The topic name identifies the node.
       */
public class ArUcoMarkerNodeMessage extends Packet<ArUcoMarkerNodeMessage> implements Settable<ArUcoMarkerNodeMessage>, EpsilonComparable<ArUcoMarkerNodeMessage>
{
   /**
            * The detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   /**
            * Break frequency filter value for nodes that are alpha filtered
            */
   public float break_frequency_;

   public ArUcoMarkerNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
   }

   public ArUcoMarkerNodeMessage(ArUcoMarkerNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(ArUcoMarkerNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      break_frequency_ = other.break_frequency_;

   }


   /**
            * The detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }

   /**
            * Break frequency filter value for nodes that are alpha filtered
            */
   public void setBreakFrequency(float break_frequency)
   {
      break_frequency_ = break_frequency;
   }
   /**
            * Break frequency filter value for nodes that are alpha filtered
            */
   public float getBreakFrequency()
   {
      return break_frequency_;
   }


   public static Supplier<ArUcoMarkerNodeMessagePubSubType> getPubSubType()
   {
      return ArUcoMarkerNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ArUcoMarkerNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ArUcoMarkerNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.break_frequency_, other.break_frequency_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ArUcoMarkerNodeMessage)) return false;

      ArUcoMarkerNodeMessage otherMyClass = (ArUcoMarkerNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if(this.break_frequency_ != otherMyClass.break_frequency_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ArUcoMarkerNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("break_frequency=");
      builder.append(this.break_frequency_);
      builder.append("}");
      return builder.toString();
   }
}
