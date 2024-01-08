package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WalkActionStateMessage extends Packet<WalkActionStateMessage> implements Settable<WalkActionStateMessage>, EpsilonComparable<WalkActionStateMessage>
{
   public static final byte TRIGGERED = (byte) 0;
   public static final byte FOOTSTEP_PLANNING = (byte) 1;
   public static final byte PLANNING_FAILED = (byte) 2;
   public static final byte PLANNING_SUCCEEDED = (byte) 3;
   public static final byte PLAN_COMMANDED = (byte) 4;
   public static final byte PLAN_EXECUTION_COMPLETE = (byte) 5;
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.WalkActionDefinitionMessage definition_;
   public byte execution_state_;
   public behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage footstep_plan_state_basics_;

   public WalkActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.WalkActionDefinitionMessage();
      footstep_plan_state_basics_ = new behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage();
   }

   public WalkActionStateMessage(WalkActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(WalkActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.WalkActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      execution_state_ = other.execution_state_;

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
   public behavior_msgs.msg.dds.WalkActionDefinitionMessage getDefinition()
   {
      return definition_;
   }

   public void setExecutionState(byte execution_state)
   {
      execution_state_ = execution_state;
   }
   public byte getExecutionState()
   {
      return execution_state_;
   }


   public behavior_msgs.msg.dds.FootstepPlanActionStateBasicsMessage getFootstepPlanStateBasics()
   {
      return footstep_plan_state_basics_;
   }


   public static Supplier<WalkActionStateMessagePubSubType> getPubSubType()
   {
      return WalkActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WalkActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WalkActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_state_, other.execution_state_, epsilon)) return false;

      if (!this.footstep_plan_state_basics_.epsilonEquals(other.footstep_plan_state_basics_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WalkActionStateMessage)) return false;

      WalkActionStateMessage otherMyClass = (WalkActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.execution_state_ != otherMyClass.execution_state_) return false;

      if (!this.footstep_plan_state_basics_.equals(otherMyClass.footstep_plan_state_basics_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WalkActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("execution_state=");
      builder.append(this.execution_state_);      builder.append(", ");
      builder.append("footstep_plan_state_basics=");
      builder.append(this.footstep_plan_state_basics_);
      builder.append("}");
      return builder.toString();
   }
}
