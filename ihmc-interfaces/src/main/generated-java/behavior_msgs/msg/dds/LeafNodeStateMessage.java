package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class LeafNodeStateMessage extends Packet<LeafNodeStateMessage> implements Settable<LeafNodeStateMessage>, EpsilonComparable<LeafNodeStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage state_;
   /**
            * The action's unique ID
            */
   public long id_;
   /**
            * If the action is next for execution
            */
   public boolean is_next_for_execution_;
   /**
            * If the node is able to execution
            */
   public boolean can_execute_;
   /**
            * If the node is currently executing
            */
   public boolean is_executing_;
   /**
            * If the node had a failure during it's last execution
            */
   public boolean failed_;

   public LeafNodeStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
   }

   public LeafNodeStateMessage(LeafNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(LeafNodeStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      id_ = other.id_;

      is_next_for_execution_ = other.is_next_for_execution_;

      can_execute_ = other.can_execute_;

      is_executing_ = other.is_executing_;

      failed_ = other.failed_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage getState()
   {
      return state_;
   }

   /**
            * The action's unique ID
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * The action's unique ID
            */
   public long getId()
   {
      return id_;
   }

   /**
            * If the action is next for execution
            */
   public void setIsNextForExecution(boolean is_next_for_execution)
   {
      is_next_for_execution_ = is_next_for_execution;
   }
   /**
            * If the action is next for execution
            */
   public boolean getIsNextForExecution()
   {
      return is_next_for_execution_;
   }

   /**
            * If the node is able to execution
            */
   public void setCanExecute(boolean can_execute)
   {
      can_execute_ = can_execute;
   }
   /**
            * If the node is able to execution
            */
   public boolean getCanExecute()
   {
      return can_execute_;
   }

   /**
            * If the node is currently executing
            */
   public void setIsExecuting(boolean is_executing)
   {
      is_executing_ = is_executing;
   }
   /**
            * If the node is currently executing
            */
   public boolean getIsExecuting()
   {
      return is_executing_;
   }

   /**
            * If the node had a failure during it's last execution
            */
   public void setFailed(boolean failed)
   {
      failed_ = failed;
   }
   /**
            * If the node had a failure during it's last execution
            */
   public boolean getFailed()
   {
      return failed_;
   }


   public static Supplier<LeafNodeStateMessagePubSubType> getPubSubType()
   {
      return LeafNodeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LeafNodeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LeafNodeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_next_for_execution_, other.is_next_for_execution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.can_execute_, other.can_execute_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_executing_, other.is_executing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.failed_, other.failed_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LeafNodeStateMessage)) return false;

      LeafNodeStateMessage otherMyClass = (LeafNodeStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if(this.id_ != otherMyClass.id_) return false;

      if(this.is_next_for_execution_ != otherMyClass.is_next_for_execution_) return false;

      if(this.can_execute_ != otherMyClass.can_execute_) return false;

      if(this.is_executing_ != otherMyClass.is_executing_) return false;

      if(this.failed_ != otherMyClass.failed_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LeafNodeStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("is_next_for_execution=");
      builder.append(this.is_next_for_execution_);      builder.append(", ");
      builder.append("can_execute=");
      builder.append(this.can_execute_);      builder.append(", ");
      builder.append("is_executing=");
      builder.append(this.is_executing_);      builder.append(", ");
      builder.append("failed=");
      builder.append(this.failed_);
      builder.append("}");
      return builder.toString();
   }
}
