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
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage definition_;
   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage>  footsteps_;
   public behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage footstep_plan_state_basics_;

   public FootstepPlanActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessage();
      footsteps_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessage> (50, new behavior_msgs.msg.dds.FootstepPlanActionFootstepStateMessagePubSubType());
      footstep_plan_state_basics_ = new behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage();

   }

   public FootstepPlanActionStateMessage(FootstepPlanActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.FootstepPlanActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      footsteps_.set(other.footsteps_);
      behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessagePubSubType.staticCopy(other.footstep_plan_state_basics_, footstep_plan_state_basics_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage getState()
   {
      return state_;
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


   public behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage getFootstepPlanStateBasics()
   {
      return footstep_plan_state_basics_;
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

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (this.footsteps_.size() != other.footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footsteps_.size(); i++)
         {  if (!this.footsteps_.get(i).epsilonEquals(other.footsteps_.get(i), epsilon)) return false; }
      }

      if (!this.footstep_plan_state_basics_.epsilonEquals(other.footstep_plan_state_basics_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionStateMessage)) return false;

      FootstepPlanActionStateMessage otherMyClass = (FootstepPlanActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!this.footsteps_.equals(otherMyClass.footsteps_)) return false;
      if (!this.footstep_plan_state_basics_.equals(otherMyClass.footstep_plan_state_basics_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("footsteps=");
      builder.append(this.footsteps_);      builder.append(", ");
      builder.append("footstep_plan_state_basics=");
      builder.append(this.footstep_plan_state_basics_);
      builder.append("}");
      return builder.toString();
   }
}
