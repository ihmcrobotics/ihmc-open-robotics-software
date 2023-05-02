package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Gives the current state of the complete collection of detectable scene nodes.
       * Publishing all scene nodes in one messages can simplify synchronization and
       * reduce the complexity of logic in figuring out when nodes are currently under
       * consideration.
       */
public class DetectableSceneNodesMessage extends Packet<DetectableSceneNodesMessage> implements Settable<DetectableSceneNodesMessage>, EpsilonComparable<DetectableSceneNodesMessage>
{
   /**
            * The current state of the complete collection of detectable scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage>  detectable_scene_nodes_;

   public DetectableSceneNodesMessage()
   {
      detectable_scene_nodes_ = new us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage> (5000, new perception_msgs.msg.dds.DetectableSceneNodeMessagePubSubType());

   }

   public DetectableSceneNodesMessage(DetectableSceneNodesMessage other)
   {
      this();
      set(other);
   }

   public void set(DetectableSceneNodesMessage other)
   {
      detectable_scene_nodes_.set(other.detectable_scene_nodes_);
   }


   /**
            * The current state of the complete collection of detectable scene nodes
            */
   public us.ihmc.idl.IDLSequence.Object<perception_msgs.msg.dds.DetectableSceneNodeMessage>  getDetectableSceneNodes()
   {
      return detectable_scene_nodes_;
   }


   public static Supplier<DetectableSceneNodesMessagePubSubType> getPubSubType()
   {
      return DetectableSceneNodesMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectableSceneNodesMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectableSceneNodesMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.detectable_scene_nodes_.size() != other.detectable_scene_nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.detectable_scene_nodes_.size(); i++)
         {  if (!this.detectable_scene_nodes_.get(i).epsilonEquals(other.detectable_scene_nodes_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectableSceneNodesMessage)) return false;

      DetectableSceneNodesMessage otherMyClass = (DetectableSceneNodesMessage) other;

      if (!this.detectable_scene_nodes_.equals(otherMyClass.detectable_scene_nodes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectableSceneNodesMessage {");
      builder.append("detectable_scene_nodes=");
      builder.append(this.detectable_scene_nodes_);
      builder.append("}");
      return builder.toString();
   }
}
