package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class TrashCanInteractionStateMessage extends Packet<TrashCanInteractionStateMessage> implements Settable<TrashCanInteractionStateMessage>, EpsilonComparable<TrashCanInteractionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage definition_;
   /**
            * FRONT (0), LEFT (1) or RIGHT (2)
            */
   public int stance_;

   public TrashCanInteractionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage();
   }

   public TrashCanInteractionStateMessage(TrashCanInteractionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(TrashCanInteractionStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      stance_ = other.stance_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.TrashCanInteractionDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * FRONT (0), LEFT (1) or RIGHT (2)
            */
   public void setStance(int stance)
   {
      stance_ = stance;
   }
   /**
            * FRONT (0), LEFT (1) or RIGHT (2)
            */
   public int getStance()
   {
      return stance_;
   }


   public static Supplier<TrashCanInteractionStateMessagePubSubType> getPubSubType()
   {
      return TrashCanInteractionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TrashCanInteractionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TrashCanInteractionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stance_, other.stance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TrashCanInteractionStateMessage)) return false;

      TrashCanInteractionStateMessage otherMyClass = (TrashCanInteractionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.stance_ != otherMyClass.stance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TrashCanInteractionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("stance=");
      builder.append(this.stance_);
      builder.append("}");
      return builder.toString();
   }
}
