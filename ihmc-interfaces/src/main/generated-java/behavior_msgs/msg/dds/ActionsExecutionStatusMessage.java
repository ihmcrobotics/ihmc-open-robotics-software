package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is meant to communicate the status of currently executing actions
       */
public class ActionsExecutionStatusMessage extends Packet<ActionsExecutionStatusMessage> implements Settable<ActionsExecutionStatusMessage>, EpsilonComparable<ActionsExecutionStatusMessage>
{
   /**
            * This message contains a list of action status messages
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionExecutionStatusMessage>  action_status_list_;

   public ActionsExecutionStatusMessage()
   {
      action_status_list_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionExecutionStatusMessage> (200, new behavior_msgs.msg.dds.ActionExecutionStatusMessagePubSubType());

   }

   public ActionsExecutionStatusMessage(ActionsExecutionStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionsExecutionStatusMessage other)
   {
      action_status_list_.set(other.action_status_list_);
   }


   /**
            * This message contains a list of action status messages
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.ActionExecutionStatusMessage>  getActionStatusList()
   {
      return action_status_list_;
   }


   public static Supplier<ActionsExecutionStatusMessagePubSubType> getPubSubType()
   {
      return ActionsExecutionStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionsExecutionStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionsExecutionStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.action_status_list_.size() != other.action_status_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.action_status_list_.size(); i++)
         {  if (!this.action_status_list_.get(i).epsilonEquals(other.action_status_list_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionsExecutionStatusMessage)) return false;

      ActionsExecutionStatusMessage otherMyClass = (ActionsExecutionStatusMessage) other;

      if (!this.action_status_list_.equals(otherMyClass.action_status_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionsExecutionStatusMessage {");
      builder.append("action_status_list=");
      builder.append(this.action_status_list_);
      builder.append("}");
      return builder.toString();
   }
}
