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
            * ArUco marker ID
            */
   public int marker_id_;
   /**
            * ArUco marker size
            */
   public float marker_size_;
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
      marker_id_ = other.marker_id_;

      marker_size_ = other.marker_size_;

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
            * ArUco marker ID
            */
   public void setMarkerId(int marker_id)
   {
      marker_id_ = marker_id;
   }
   /**
            * ArUco marker ID
            */
   public int getMarkerId()
   {
      return marker_id_;
   }

   /**
            * ArUco marker size
            */
   public void setMarkerSize(float marker_size)
   {
      marker_size_ = marker_size;
   }
   /**
            * ArUco marker size
            */
   public float getMarkerSize()
   {
      return marker_size_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.marker_id_, other.marker_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.marker_size_, other.marker_size_, epsilon)) return false;

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
      if(this.marker_id_ != otherMyClass.marker_id_) return false;

      if(this.marker_size_ != otherMyClass.marker_size_) return false;

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
      builder.append("marker_id=");
      builder.append(this.marker_id_);      builder.append(", ");
      builder.append("marker_size=");
      builder.append(this.marker_size_);      builder.append(", ");
      builder.append("break_frequency=");
      builder.append(this.break_frequency_);
      builder.append("}");
      return builder.toString();
   }
}
