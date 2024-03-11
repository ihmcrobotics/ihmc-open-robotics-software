package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionNodeDefinitionMessage extends Packet<ActionNodeDefinitionMessage> implements Settable<ActionNodeDefinitionMessage>, EpsilonComparable<ActionNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * Distance backward to the node to execute after.
            * For example, 1 is for the previous action, 2 is for the action before that.
            * Must be 1 or greater.
            */
   public int distance_to_execute_after_;

   public ActionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public ActionNodeDefinitionMessage(ActionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      distance_to_execute_after_ = other.distance_to_execute_after_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Distance backward to the node to execute after.
            * For example, 1 is for the previous action, 2 is for the action before that.
            * Must be 1 or greater.
            */
   public void setDistanceToExecuteAfter(int distance_to_execute_after)
   {
      distance_to_execute_after_ = distance_to_execute_after;
   }
   /**
            * Distance backward to the node to execute after.
            * For example, 1 is for the previous action, 2 is for the action before that.
            * Must be 1 or greater.
            */
   public int getDistanceToExecuteAfter()
   {
      return distance_to_execute_after_;
   }


   public static Supplier<ActionNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return ActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_to_execute_after_, other.distance_to_execute_after_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionNodeDefinitionMessage)) return false;

      ActionNodeDefinitionMessage otherMyClass = (ActionNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.distance_to_execute_after_ != otherMyClass.distance_to_execute_after_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("distance_to_execute_after=");
      builder.append(this.distance_to_execute_after_);
      builder.append("}");
      return builder.toString();
   }
}
