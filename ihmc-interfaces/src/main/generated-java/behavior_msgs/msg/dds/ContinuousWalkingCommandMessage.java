package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ContinuousWalkingCommandMessage extends Packet<ContinuousWalkingCommandMessage> implements Settable<ContinuousWalkingCommandMessage>, EpsilonComparable<ContinuousWalkingCommandMessage>
{
   /**
            * flag to enable/disable publishing to controller
            */
   public boolean publish_to_controller_;
   /**
            * flag to enable/disable continuous walking state machine
            */
   public boolean enable_continuous_walking_;
   /**
            * number of steps to send to controller
            */
   public int number_of_steps_to_send_;
   /**
            * forward joystick value
            */
   public double forward_value_;
   /**
            * lateral joystick value
            */
   public double lateral_value_;
   /**
            * turning joystick value
            */
   public double turning_value_;
   /**
            * flag to enable/disable hybrid planning
            */
   public boolean use_hybrid_planner_;
   /**
            * flag to enable/disable planning with astar planner
            */
   public boolean use_astar_footstep_planner_;
   /**
            * flag to enable/disable planning with monte-carlo footstep planner
            */
   public boolean use_monte_carlo_footstep_planner_;
   /**
            * flag to enable/disable using previous plan as reference
            */
   public boolean use_previous_plan_as_reference_;
   /**
            * flag to enable/disable using monte-carlo plan as reference
            */
   public boolean use_monte_carlo_plan_as_reference_;

   public ContinuousWalkingCommandMessage()
   {
   }

   public ContinuousWalkingCommandMessage(ContinuousWalkingCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(ContinuousWalkingCommandMessage other)
   {
      publish_to_controller_ = other.publish_to_controller_;

      enable_continuous_walking_ = other.enable_continuous_walking_;

      number_of_steps_to_send_ = other.number_of_steps_to_send_;

      forward_value_ = other.forward_value_;

      lateral_value_ = other.lateral_value_;

      turning_value_ = other.turning_value_;

      use_hybrid_planner_ = other.use_hybrid_planner_;

      use_astar_footstep_planner_ = other.use_astar_footstep_planner_;

      use_monte_carlo_footstep_planner_ = other.use_monte_carlo_footstep_planner_;

      use_previous_plan_as_reference_ = other.use_previous_plan_as_reference_;

      use_monte_carlo_plan_as_reference_ = other.use_monte_carlo_plan_as_reference_;

   }

   /**
            * flag to enable/disable publishing to controller
            */
   public void setPublishToController(boolean publish_to_controller)
   {
      publish_to_controller_ = publish_to_controller;
   }
   /**
            * flag to enable/disable publishing to controller
            */
   public boolean getPublishToController()
   {
      return publish_to_controller_;
   }

   /**
            * flag to enable/disable continuous walking state machine
            */
   public void setEnableContinuousWalking(boolean enable_continuous_walking)
   {
      enable_continuous_walking_ = enable_continuous_walking;
   }
   /**
            * flag to enable/disable continuous walking state machine
            */
   public boolean getEnableContinuousWalking()
   {
      return enable_continuous_walking_;
   }

   /**
            * number of steps to send to controller
            */
   public void setNumberOfStepsToSend(int number_of_steps_to_send)
   {
      number_of_steps_to_send_ = number_of_steps_to_send;
   }
   /**
            * number of steps to send to controller
            */
   public int getNumberOfStepsToSend()
   {
      return number_of_steps_to_send_;
   }

   /**
            * forward joystick value
            */
   public void setForwardValue(double forward_value)
   {
      forward_value_ = forward_value;
   }
   /**
            * forward joystick value
            */
   public double getForwardValue()
   {
      return forward_value_;
   }

   /**
            * lateral joystick value
            */
   public void setLateralValue(double lateral_value)
   {
      lateral_value_ = lateral_value;
   }
   /**
            * lateral joystick value
            */
   public double getLateralValue()
   {
      return lateral_value_;
   }

   /**
            * turning joystick value
            */
   public void setTurningValue(double turning_value)
   {
      turning_value_ = turning_value;
   }
   /**
            * turning joystick value
            */
   public double getTurningValue()
   {
      return turning_value_;
   }

   /**
            * flag to enable/disable hybrid planning
            */
   public void setUseHybridPlanner(boolean use_hybrid_planner)
   {
      use_hybrid_planner_ = use_hybrid_planner;
   }
   /**
            * flag to enable/disable hybrid planning
            */
   public boolean getUseHybridPlanner()
   {
      return use_hybrid_planner_;
   }

   /**
            * flag to enable/disable planning with astar planner
            */
   public void setUseAstarFootstepPlanner(boolean use_astar_footstep_planner)
   {
      use_astar_footstep_planner_ = use_astar_footstep_planner;
   }
   /**
            * flag to enable/disable planning with astar planner
            */
   public boolean getUseAstarFootstepPlanner()
   {
      return use_astar_footstep_planner_;
   }

   /**
            * flag to enable/disable planning with monte-carlo footstep planner
            */
   public void setUseMonteCarloFootstepPlanner(boolean use_monte_carlo_footstep_planner)
   {
      use_monte_carlo_footstep_planner_ = use_monte_carlo_footstep_planner;
   }
   /**
            * flag to enable/disable planning with monte-carlo footstep planner
            */
   public boolean getUseMonteCarloFootstepPlanner()
   {
      return use_monte_carlo_footstep_planner_;
   }

   /**
            * flag to enable/disable using previous plan as reference
            */
   public void setUsePreviousPlanAsReference(boolean use_previous_plan_as_reference)
   {
      use_previous_plan_as_reference_ = use_previous_plan_as_reference;
   }
   /**
            * flag to enable/disable using previous plan as reference
            */
   public boolean getUsePreviousPlanAsReference()
   {
      return use_previous_plan_as_reference_;
   }

   /**
            * flag to enable/disable using monte-carlo plan as reference
            */
   public void setUseMonteCarloPlanAsReference(boolean use_monte_carlo_plan_as_reference)
   {
      use_monte_carlo_plan_as_reference_ = use_monte_carlo_plan_as_reference;
   }
   /**
            * flag to enable/disable using monte-carlo plan as reference
            */
   public boolean getUseMonteCarloPlanAsReference()
   {
      return use_monte_carlo_plan_as_reference_;
   }


   public static Supplier<ContinuousWalkingCommandMessagePubSubType> getPubSubType()
   {
      return ContinuousWalkingCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContinuousWalkingCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContinuousWalkingCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.publish_to_controller_, other.publish_to_controller_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_continuous_walking_, other.enable_continuous_walking_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_steps_to_send_, other.number_of_steps_to_send_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_value_, other.forward_value_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lateral_value_, other.lateral_value_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turning_value_, other.turning_value_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_hybrid_planner_, other.use_hybrid_planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_astar_footstep_planner_, other.use_astar_footstep_planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_monte_carlo_footstep_planner_, other.use_monte_carlo_footstep_planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_previous_plan_as_reference_, other.use_previous_plan_as_reference_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_monte_carlo_plan_as_reference_, other.use_monte_carlo_plan_as_reference_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContinuousWalkingCommandMessage)) return false;

      ContinuousWalkingCommandMessage otherMyClass = (ContinuousWalkingCommandMessage) other;

      if(this.publish_to_controller_ != otherMyClass.publish_to_controller_) return false;

      if(this.enable_continuous_walking_ != otherMyClass.enable_continuous_walking_) return false;

      if(this.number_of_steps_to_send_ != otherMyClass.number_of_steps_to_send_) return false;

      if(this.forward_value_ != otherMyClass.forward_value_) return false;

      if(this.lateral_value_ != otherMyClass.lateral_value_) return false;

      if(this.turning_value_ != otherMyClass.turning_value_) return false;

      if(this.use_hybrid_planner_ != otherMyClass.use_hybrid_planner_) return false;

      if(this.use_astar_footstep_planner_ != otherMyClass.use_astar_footstep_planner_) return false;

      if(this.use_monte_carlo_footstep_planner_ != otherMyClass.use_monte_carlo_footstep_planner_) return false;

      if(this.use_previous_plan_as_reference_ != otherMyClass.use_previous_plan_as_reference_) return false;

      if(this.use_monte_carlo_plan_as_reference_ != otherMyClass.use_monte_carlo_plan_as_reference_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContinuousWalkingCommandMessage {");
      builder.append("publish_to_controller=");
      builder.append(this.publish_to_controller_);      builder.append(", ");
      builder.append("enable_continuous_walking=");
      builder.append(this.enable_continuous_walking_);      builder.append(", ");
      builder.append("number_of_steps_to_send=");
      builder.append(this.number_of_steps_to_send_);      builder.append(", ");
      builder.append("forward_value=");
      builder.append(this.forward_value_);      builder.append(", ");
      builder.append("lateral_value=");
      builder.append(this.lateral_value_);      builder.append(", ");
      builder.append("turning_value=");
      builder.append(this.turning_value_);      builder.append(", ");
      builder.append("use_hybrid_planner=");
      builder.append(this.use_hybrid_planner_);      builder.append(", ");
      builder.append("use_astar_footstep_planner=");
      builder.append(this.use_astar_footstep_planner_);      builder.append(", ");
      builder.append("use_monte_carlo_footstep_planner=");
      builder.append(this.use_monte_carlo_footstep_planner_);      builder.append(", ");
      builder.append("use_previous_plan_as_reference=");
      builder.append(this.use_previous_plan_as_reference_);      builder.append(", ");
      builder.append("use_monte_carlo_plan_as_reference=");
      builder.append(this.use_monte_carlo_plan_as_reference_);
      builder.append("}");
      return builder.toString();
   }
}
