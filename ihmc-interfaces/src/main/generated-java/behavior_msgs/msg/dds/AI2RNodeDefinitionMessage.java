package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AI2RNodeDefinitionMessage extends Packet<AI2RNodeDefinitionMessage> implements Settable<AI2RNodeDefinitionMessage>, EpsilonComparable<AI2RNodeDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage definition_;
   /**
            * Randomized Go-To action controls
            */
   public boolean randomize_go_to_action_;
   public int number_of_randomizations_;

   public AI2RNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage();
   }

   public AI2RNodeDefinitionMessage(AI2RNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(AI2RNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      randomize_go_to_action_ = other.randomize_go_to_action_;

      number_of_randomizations_ = other.number_of_randomizations_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Randomized Go-To action controls
            */
   public void setRandomizeGoToAction(boolean randomize_go_to_action)
   {
      randomize_go_to_action_ = randomize_go_to_action;
   }
   /**
            * Randomized Go-To action controls
            */
   public boolean getRandomizeGoToAction()
   {
      return randomize_go_to_action_;
   }

   public void setNumberOfRandomizations(int number_of_randomizations)
   {
      number_of_randomizations_ = number_of_randomizations;
   }
   public int getNumberOfRandomizations()
   {
      return number_of_randomizations_;
   }


   public static Supplier<AI2RNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return AI2RNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AI2RNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AI2RNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.randomize_go_to_action_, other.randomize_go_to_action_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_randomizations_, other.number_of_randomizations_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AI2RNodeDefinitionMessage)) return false;

      AI2RNodeDefinitionMessage otherMyClass = (AI2RNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.randomize_go_to_action_ != otherMyClass.randomize_go_to_action_) return false;

      if(this.number_of_randomizations_ != otherMyClass.number_of_randomizations_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AI2RNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("randomize_go_to_action=");
      builder.append(this.randomize_go_to_action_);      builder.append(", ");
      builder.append("number_of_randomizations=");
      builder.append(this.number_of_randomizations_);
      builder.append("}");
      return builder.toString();
   }
}
