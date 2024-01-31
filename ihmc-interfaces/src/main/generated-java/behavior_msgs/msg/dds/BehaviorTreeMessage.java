package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeMessage extends Packet<BehaviorTreeMessage> implements Settable<BehaviorTreeMessage>, EpsilonComparable<BehaviorTreeMessage>
{
   /**
            * DEPRECATED: This is an old message replaced by BehaviorTreeStateMessage
            * Nodes in the tree in "depth first" order.
            * We must reconstruct the tree on the subscription end.
            * This is because messages cannot be recursive.
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeNodeMessage>  nodes_;

   public BehaviorTreeMessage()
   {
      nodes_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeNodeMessage> (2048, new behavior_msgs.msg.dds.BehaviorTreeNodeMessagePubSubType());

   }

   public BehaviorTreeMessage(BehaviorTreeMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeMessage other)
   {
      nodes_.set(other.nodes_);
   }


   /**
            * DEPRECATED: This is an old message replaced by BehaviorTreeStateMessage
            * Nodes in the tree in "depth first" order.
            * We must reconstruct the tree on the subscription end.
            * This is because messages cannot be recursive.
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.BehaviorTreeNodeMessage>  getNodes()
   {
      return nodes_;
   }


   public static Supplier<BehaviorTreeMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.nodes_.size() != other.nodes_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.nodes_.size(); i++)
         {  if (!this.nodes_.get(i).epsilonEquals(other.nodes_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeMessage)) return false;

      BehaviorTreeMessage otherMyClass = (BehaviorTreeMessage) other;

      if (!this.nodes_.equals(otherMyClass.nodes_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeMessage {");
      builder.append("nodes=");
      builder.append(this.nodes_);
      builder.append("}");
      return builder.toString();
   }
}
