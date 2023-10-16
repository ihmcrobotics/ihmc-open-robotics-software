package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BehaviorTreeNodeStateMessage extends Packet<BehaviorTreeNodeStateMessage> implements Settable<BehaviorTreeNodeStateMessage>, EpsilonComparable<BehaviorTreeNodeStateMessage>
{
   public static final byte RUNNING = (byte) 0;
   public static final byte FAILURE = (byte) 1;
   public static final byte SUCCESS = (byte) 2;
   public static final byte NOT_TICKED = (byte) 3;
   /**
            * The current status of the node
            */
   public byte status_;
   /**
            * The instant the node was last ticked
            */
   public ihmc_common_msgs.msg.dds.InstantMessage last_tick_instant_;

   public BehaviorTreeNodeStateMessage()
   {
      last_tick_instant_ = new ihmc_common_msgs.msg.dds.InstantMessage();
   }

   public BehaviorTreeNodeStateMessage(BehaviorTreeNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(BehaviorTreeNodeStateMessage other)
   {
      status_ = other.status_;

      ihmc_common_msgs.msg.dds.InstantMessagePubSubType.staticCopy(other.last_tick_instant_, last_tick_instant_);
   }

   /**
            * The current status of the node
            */
   public void setStatus(byte status)
   {
      status_ = status;
   }
   /**
            * The current status of the node
            */
   public byte getStatus()
   {
      return status_;
   }


   /**
            * The instant the node was last ticked
            */
   public ihmc_common_msgs.msg.dds.InstantMessage getLastTickInstant()
   {
      return last_tick_instant_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.status_, other.status_, epsilon)) return false;

      if (!this.last_tick_instant_.epsilonEquals(other.last_tick_instant_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BehaviorTreeNodeStateMessage)) return false;

      BehaviorTreeNodeStateMessage otherMyClass = (BehaviorTreeNodeStateMessage) other;

      if(this.status_ != otherMyClass.status_) return false;

      if (!this.last_tick_instant_.equals(otherMyClass.last_tick_instant_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorTreeNodeStateMessage {");
      builder.append("status=");
      builder.append(this.status_);      builder.append(", ");
      builder.append("last_tick_instant=");
      builder.append(this.last_tick_instant_);
      builder.append("}");
      return builder.toString();
   }
}
