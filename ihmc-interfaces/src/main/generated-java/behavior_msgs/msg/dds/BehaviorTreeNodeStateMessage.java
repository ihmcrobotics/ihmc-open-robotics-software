package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeNodeStateMessage extends Packet<BehaviorTreeNodeStateMessage> implements Settable<BehaviorTreeNodeStateMessage>, EpsilonComparable<BehaviorTreeNodeStateMessage>
{
   /**
            * A node is active if it lies on the path of the current tree tick.
            */
   public boolean is_active_;

   public BehaviorTreeNodeStateMessage()
   {
   }

   public BehaviorTreeNodeStateMessage(BehaviorTreeNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeNodeStateMessage other)
   {
      is_active_ = other.is_active_;

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

      if(this.is_active_ != otherMyClass.is_active_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeNodeStateMessage {");
      builder.append("is_active=");
      builder.append(this.is_active_);
      builder.append("}");
      return builder.toString();
   }
}
