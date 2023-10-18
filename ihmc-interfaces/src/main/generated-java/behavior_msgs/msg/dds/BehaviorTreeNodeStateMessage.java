package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeNodeStateMessage extends Packet<BehaviorTreeNodeStateMessage> implements Settable<BehaviorTreeNodeStateMessage>, EpsilonComparable<BehaviorTreeNodeStateMessage>
{
   /**
            * Node description
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * The node's unique ID
            */
   public long id_;
   /**
            * A node is active if it lies on the path of the current tree tick.
            */
   public boolean is_active_;

   public BehaviorTreeNodeStateMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public BehaviorTreeNodeStateMessage(BehaviorTreeNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeNodeStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      id_ = other.id_;

      is_active_ = other.is_active_;

   }


   /**
            * Node description
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The node's unique ID
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * The node's unique ID
            */
   public long getId()
   {
      return id_;
   }

   /**
            * A node is active if it lies on the path of the current tree tick.
            */
   public void setIsActive(boolean is_active)
   {
      is_active_ = is_active;
   }
   /**
            * A node is active if it lies on the path of the current tree tick.
            */
   public boolean getIsActive()
   {
      return is_active_;
   }


   public static Supplier<BehaviorTreeNodeStateMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeNodeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeNodeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeNodeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_active_, other.is_active_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeNodeStateMessage)) return false;

      BehaviorTreeNodeStateMessage otherMyClass = (BehaviorTreeNodeStateMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.id_ != otherMyClass.id_) return false;

      if(this.is_active_ != otherMyClass.is_active_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeNodeStateMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("is_active=");
      builder.append(this.is_active_);
      builder.append("}");
      return builder.toString();
   }
}
