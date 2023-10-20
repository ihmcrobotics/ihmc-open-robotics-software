package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionStateMessage extends Packet<FootstepPlanActionStateMessage> implements Settable<FootstepPlanActionStateMessage>, EpsilonComparable<FootstepPlanActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorActionStateMessage action_state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage definition_;
   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage>  footsteps_;

   public FootstepPlanActionStateMessage()
   {
      action_state_ = new behavior_msgs.msg.dds.BehaviorActionStateMessage();
      definition_ = new behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage();
      footsteps_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage> (50, new behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessagePubSubType());

   }

   public FootstepPlanActionStateMessage(FootstepPlanActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorActionStateMessagePubSubType.staticCopy(other.action_state_, action_state_);
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      footsteps_.set(other.footsteps_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorActionStateMessage getActionState()
   {
      return action_state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage>  getFootsteps()
   {
      return footsteps_;
   }


   public static Supplier<FootstepPlanActionStateMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_state_.epsilonEquals(other.action_state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (this.footsteps_.size() != other.footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footsteps_.size(); i++)
         {  if (!this.footsteps_.get(i).epsilonEquals(other.footsteps_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionStateMessage)) return false;

      FootstepPlanActionStateMessage otherMyClass = (FootstepPlanActionStateMessage) other;

      if (!this.action_state_.equals(otherMyClass.action_state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.footsteps_.equals(otherMyClass.footsteps_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionStateMessage {");
      builder.append("action_state=");
      builder.append(this.action_state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("footsteps=");
      builder.append(this.footsteps_);
      builder.append("}");
      return builder.toString();
   }
}
