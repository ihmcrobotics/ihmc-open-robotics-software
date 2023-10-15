package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeNodeDefinitionMessage extends Packet<BehaviorTreeNodeDefinitionMessage> implements Settable<BehaviorTreeNodeDefinitionMessage>, EpsilonComparable<BehaviorTreeNodeDefinitionMessage>
{
   /**
            * A human readable description of what the node does
            */
   public java.lang.StringBuilder description_;

   public BehaviorTreeNodeDefinitionMessage()
   {
      description_ = new java.lang.StringBuilder(255);
   }

   public BehaviorTreeNodeDefinitionMessage(BehaviorTreeNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeNodeDefinitionMessage other)
   {
      description_.setLength(0);
      description_.append(other.description_);
   }

   /**
            * A human readable description of what the node does
            */
   public void setDescription(java.lang.String description)
   {
      description_.setLength(0);
      description_.append(description);
   }

   /**
            * A human readable description of what the node does
            */
   public java.lang.String getDescriptionAsString()
   {
      return getDescription().toString();
   }
   /**
            * A human readable description of what the node does
            */
   public java.lang.StringBuilder getDescription()
   {
      return description_;
   }


   public static Supplier<BehaviorTreeNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return BehaviorTreeNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BehaviorTreeNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BehaviorTreeNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.description_, other.description_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeNodeDefinitionMessage)) return false;

      BehaviorTreeNodeDefinitionMessage otherMyClass = (BehaviorTreeNodeDefinitionMessage) other;

      if (!us.ihmc.idl.IDLTools.equals(this.description_, otherMyClass.description_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeNodeDefinitionMessage {");
      builder.append("description=");
      builder.append(this.description_);
      builder.append("}");
      return builder.toString();
   }
}
