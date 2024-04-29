package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WholeBodyCoordinatorStateMessage extends Packet<WholeBodyCoordinatorStateMessage> implements Settable<WholeBodyCoordinatorStateMessage>, EpsilonComparable<WholeBodyCoordinatorStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage state_;
   /**
            * The user requested kinematic simulation preview time
            */
   public double preview_requested_time_;

   public WholeBodyCoordinatorStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
   }

   public WholeBodyCoordinatorStateMessage(WholeBodyCoordinatorStateMessage other)
   {
      this();
      set(other);
   }

   public void set(WholeBodyCoordinatorStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      preview_requested_time_ = other.preview_requested_time_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage getState()
   {
      return state_;
   }

   /**
            * The user requested kinematic simulation preview time
            */
   public void setPreviewRequestedTime(double preview_requested_time)
   {
      preview_requested_time_ = preview_requested_time;
   }
   /**
            * The user requested kinematic simulation preview time
            */
   public double getPreviewRequestedTime()
   {
      return preview_requested_time_;
   }


   public static Supplier<WholeBodyCoordinatorStateMessagePubSubType> getPubSubType()
   {
      return WholeBodyCoordinatorStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WholeBodyCoordinatorStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WholeBodyCoordinatorStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preview_requested_time_, other.preview_requested_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WholeBodyCoordinatorStateMessage)) return false;

      WholeBodyCoordinatorStateMessage otherMyClass = (WholeBodyCoordinatorStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if(this.preview_requested_time_ != otherMyClass.preview_requested_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WholeBodyCoordinatorStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("preview_requested_time=");
      builder.append(this.preview_requested_time_);
      builder.append("}");
      return builder.toString();
   }
}
