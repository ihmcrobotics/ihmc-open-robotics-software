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
public class CenterposeNodeMessage extends Packet<CenterposeNodeMessage> implements Settable<CenterposeNodeMessage>, EpsilonComparable<CenterposeNodeMessage>
{
   /**
            * The detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage detectable_scene_node_;
   public boolean enable_tracking_;

   public CenterposeNodeMessage()
   {
      detectable_scene_node_ = new perception_msgs.msg.dds.DetectableSceneNodeMessage();
   }

   public CenterposeNodeMessage(CenterposeNodeMessage other)
   {
      this();
      set(other);
   }

   public void set(CenterposeNodeMessage other)
   {
      perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType.staticCopy(other.detectable_scene_node_, detectable_scene_node_);
      enable_tracking_ = other.enable_tracking_;

   }


   /**
            * The detectable scene node information
            */
   public perception_msgs.msg.dds.DetectableSceneNodeMessage getDetectableSceneNode()
   {
      return detectable_scene_node_;
   }

   public void setEnableTracking(boolean enable_tracking)
   {
      enable_tracking_ = enable_tracking;
   }
   public boolean getEnableTracking()
   {
      return enable_tracking_;
   }


   public static Supplier<CenterposeNodeMessagePubSubType> getPubSubType()
   {
      return CenterposeNodeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CenterposeNodeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CenterposeNodeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.detectable_scene_node_.epsilonEquals(other.detectable_scene_node_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_tracking_, other.enable_tracking_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CenterposeNodeMessage)) return false;

      CenterposeNodeMessage otherMyClass = (CenterposeNodeMessage) other;

      if (!this.detectable_scene_node_.equals(otherMyClass.detectable_scene_node_)) return false;
      if(this.enable_tracking_ != otherMyClass.enable_tracking_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CenterposeNodeMessage {");
      builder.append("detectable_scene_node=");
      builder.append(this.detectable_scene_node_);      builder.append(", ");
      builder.append("enable_tracking=");
      builder.append(this.enable_tracking_);
      builder.append("}");
      return builder.toString();
   }
}
