package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message sends the current request for the Continuous Hiking process.
       * This holds parameters on how the Continuous Hiking process will behave.
       */
public class ContinuousHikingCommandMessage extends Packet<ContinuousHikingCommandMessage> implements Settable<ContinuousHikingCommandMessage>, EpsilonComparable<ContinuousHikingCommandMessage>
{
   /**
            * Flag to start Continuous Hiking
            */
   public boolean enable_continuous_hiking_;
   /**
            * Flag that allows the user to set a number of steps that will be taken, and Continuous Hiking will stop after that. This way the robot doesn't walk forever
            */
   public long steps_before_safety_stop_;
   /**
            * Flag to walk straight forward with Continuous Hiking. This is generally a keyboard input, and is different then trying to walk with the Joystick
            */
   public boolean walk_forwards_;
   /**
            * Flag to square up to the goal when finishing walking. We may want the robot to end at an exact location.
            * Setting this to true tells the robot to step on the final goal positions.
            */
   public boolean square_up_to_goal_;
   /**
            * Flag to enable/disable planning with A* planner. This is the default planner to use
            */
   public boolean use_astar_footstep_planner_;
   /**
            * Flag to enable/disable planning with monte-carlo footstep planner
            */
   public boolean use_monte_carlo_footstep_planner_;
   /**
            * Flag to enable/disable using previous plan as reference. Generally this should be set to true to reduce planning time
            */
   public boolean use_previous_plan_as_reference_;
   /**
            * Flag to enable/disable using monte-carlo plan as reference
            */
   public boolean use_monte_carlo_plan_as_reference_;
   /**
            * Walk with the Joystick Controller
            */
   public boolean use_joystick_controller_;
   /**
            * Forward joystick value
            */
   public double forward_value_;
   /**
            * Backward walking from Joystick Controller
            */
   public boolean walk_backwards_;
   /**
            * Lateral joystick value
            */
   public double lateral_value_;
   /**
            * Turning joystick value
            */
   public double turning_value_;

   public ContinuousHikingCommandMessage()
   {
   }

   public ContinuousHikingCommandMessage(ContinuousHikingCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(ContinuousHikingCommandMessage other)
   {
      enable_continuous_hiking_ = other.enable_continuous_hiking_;

      steps_before_safety_stop_ = other.steps_before_safety_stop_;

      walk_forwards_ = other.walk_forwards_;

      square_up_to_goal_ = other.square_up_to_goal_;

      use_astar_footstep_planner_ = other.use_astar_footstep_planner_;

      use_monte_carlo_footstep_planner_ = other.use_monte_carlo_footstep_planner_;

      use_previous_plan_as_reference_ = other.use_previous_plan_as_reference_;

      use_monte_carlo_plan_as_reference_ = other.use_monte_carlo_plan_as_reference_;

      use_joystick_controller_ = other.use_joystick_controller_;

      forward_value_ = other.forward_value_;

      walk_backwards_ = other.walk_backwards_;

      lateral_value_ = other.lateral_value_;

      turning_value_ = other.turning_value_;

   }

   /**
            * Flag to start Continuous Hiking
            */
   public void setEnableContinuousHiking(boolean enable_continuous_hiking)
   {
      enable_continuous_hiking_ = enable_continuous_hiking;
   }
   /**
            * Flag to start Continuous Hiking
            */
   public boolean getEnableContinuousHiking()
   {
      return enable_continuous_hiking_;
   }

   /**
            * Flag that allows the user to set a number of steps that will be taken, and Continuous Hiking will stop after that. This way the robot doesn't walk forever
            */
   public void setStepsBeforeSafetyStop(long steps_before_safety_stop)
   {
      steps_before_safety_stop_ = steps_before_safety_stop;
   }
   /**
            * Flag that allows the user to set a number of steps that will be taken, and Continuous Hiking will stop after that. This way the robot doesn't walk forever
            */
   public long getStepsBeforeSafetyStop()
   {
      return steps_before_safety_stop_;
   }

   /**
            * Flag to walk straight forward with Continuous Hiking. This is generally a keyboard input, and is different then trying to walk with the Joystick
            */
   public void setWalkForwards(boolean walk_forwards)
   {
      walk_forwards_ = walk_forwards;
   }
   /**
            * Flag to walk straight forward with Continuous Hiking. This is generally a keyboard input, and is different then trying to walk with the Joystick
            */
   public boolean getWalkForwards()
   {
      return walk_forwards_;
   }

   /**
            * Flag to square up to the goal when finishing walking. We may want the robot to end at an exact location.
            * Setting this to true tells the robot to step on the final goal positions.
            */
   public void setSquareUpToGoal(boolean square_up_to_goal)
   {
      square_up_to_goal_ = square_up_to_goal;
   }
   /**
            * Flag to square up to the goal when finishing walking. We may want the robot to end at an exact location.
            * Setting this to true tells the robot to step on the final goal positions.
            */
   public boolean getSquareUpToGoal()
   {
      return square_up_to_goal_;
   }

   /**
            * Flag to enable/disable planning with A* planner. This is the default planner to use
            */
   public void setUseAstarFootstepPlanner(boolean use_astar_footstep_planner)
   {
      use_astar_footstep_planner_ = use_astar_footstep_planner;
   }
   /**
            * Flag to enable/disable planning with A* planner. This is the default planner to use
            */
   public boolean getUseAstarFootstepPlanner()
   {
      return use_astar_footstep_planner_;
   }

   /**
            * Flag to enable/disable planning with monte-carlo footstep planner
            */
   public void setUseMonteCarloFootstepPlanner(boolean use_monte_carlo_footstep_planner)
   {
      use_monte_carlo_footstep_planner_ = use_monte_carlo_footstep_planner;
   }
   /**
            * Flag to enable/disable planning with monte-carlo footstep planner
            */
   public boolean getUseMonteCarloFootstepPlanner()
   {
      return use_monte_carlo_footstep_planner_;
   }

   /**
            * Flag to enable/disable using previous plan as reference. Generally this should be set to true to reduce planning time
            */
   public void setUsePreviousPlanAsReference(boolean use_previous_plan_as_reference)
   {
      use_previous_plan_as_reference_ = use_previous_plan_as_reference;
   }
   /**
            * Flag to enable/disable using previous plan as reference. Generally this should be set to true to reduce planning time
            */
   public boolean getUsePreviousPlanAsReference()
   {
      return use_previous_plan_as_reference_;
   }

   /**
            * Flag to enable/disable using monte-carlo plan as reference
            */
   public void setUseMonteCarloPlanAsReference(boolean use_monte_carlo_plan_as_reference)
   {
      use_monte_carlo_plan_as_reference_ = use_monte_carlo_plan_as_reference;
   }
   /**
            * Flag to enable/disable using monte-carlo plan as reference
            */
   public boolean getUseMonteCarloPlanAsReference()
   {
      return use_monte_carlo_plan_as_reference_;
   }

   /**
            * Walk with the Joystick Controller
            */
   public void setUseJoystickController(boolean use_joystick_controller)
   {
      use_joystick_controller_ = use_joystick_controller;
   }
   /**
            * Walk with the Joystick Controller
            */
   public boolean getUseJoystickController()
   {
      return use_joystick_controller_;
   }

   /**
            * Forward joystick value
            */
   public void setForwardValue(double forward_value)
   {
      forward_value_ = forward_value;
   }
   /**
            * Forward joystick value
            */
   public double getForwardValue()
   {
      return forward_value_;
   }

   /**
            * Backward walking from Joystick Controller
            */
   public void setWalkBackwards(boolean walk_backwards)
   {
      walk_backwards_ = walk_backwards;
   }
   /**
            * Backward walking from Joystick Controller
            */
   public boolean getWalkBackwards()
   {
      return walk_backwards_;
   }

   /**
            * Lateral joystick value
            */
   public void setLateralValue(double lateral_value)
   {
      lateral_value_ = lateral_value;
   }
   /**
            * Lateral joystick value
            */
   public double getLateralValue()
   {
      return lateral_value_;
   }

   /**
            * Turning joystick value
            */
   public void setTurningValue(double turning_value)
   {
      turning_value_ = turning_value;
   }
   /**
            * Turning joystick value
            */
   public double getTurningValue()
   {
      return turning_value_;
   }


   public static Supplier<ContinuousHikingCommandMessagePubSubType> getPubSubType()
   {
      return ContinuousHikingCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContinuousHikingCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContinuousHikingCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_continuous_hiking_, other.enable_continuous_hiking_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.steps_before_safety_stop_, other.steps_before_safety_stop_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.walk_forwards_, other.walk_forwards_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.square_up_to_goal_, other.square_up_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_astar_footstep_planner_, other.use_astar_footstep_planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_monte_carlo_footstep_planner_, other.use_monte_carlo_footstep_planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_previous_plan_as_reference_, other.use_previous_plan_as_reference_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_monte_carlo_plan_as_reference_, other.use_monte_carlo_plan_as_reference_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_joystick_controller_, other.use_joystick_controller_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_value_, other.forward_value_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.walk_backwards_, other.walk_backwards_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lateral_value_, other.lateral_value_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.turning_value_, other.turning_value_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContinuousHikingCommandMessage)) return false;

      ContinuousHikingCommandMessage otherMyClass = (ContinuousHikingCommandMessage) other;

      if(this.enable_continuous_hiking_ != otherMyClass.enable_continuous_hiking_) return false;

      if(this.steps_before_safety_stop_ != otherMyClass.steps_before_safety_stop_) return false;

      if(this.walk_forwards_ != otherMyClass.walk_forwards_) return false;

      if(this.square_up_to_goal_ != otherMyClass.square_up_to_goal_) return false;

      if(this.use_astar_footstep_planner_ != otherMyClass.use_astar_footstep_planner_) return false;

      if(this.use_monte_carlo_footstep_planner_ != otherMyClass.use_monte_carlo_footstep_planner_) return false;

      if(this.use_previous_plan_as_reference_ != otherMyClass.use_previous_plan_as_reference_) return false;

      if(this.use_monte_carlo_plan_as_reference_ != otherMyClass.use_monte_carlo_plan_as_reference_) return false;

      if(this.use_joystick_controller_ != otherMyClass.use_joystick_controller_) return false;

      if(this.forward_value_ != otherMyClass.forward_value_) return false;

      if(this.walk_backwards_ != otherMyClass.walk_backwards_) return false;

      if(this.lateral_value_ != otherMyClass.lateral_value_) return false;

      if(this.turning_value_ != otherMyClass.turning_value_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContinuousHikingCommandMessage {");
      builder.append("enable_continuous_hiking=");
      builder.append(this.enable_continuous_hiking_);      builder.append(", ");
      builder.append("steps_before_safety_stop=");
      builder.append(this.steps_before_safety_stop_);      builder.append(", ");
      builder.append("walk_forwards=");
      builder.append(this.walk_forwards_);      builder.append(", ");
      builder.append("square_up_to_goal=");
      builder.append(this.square_up_to_goal_);      builder.append(", ");
      builder.append("use_astar_footstep_planner=");
      builder.append(this.use_astar_footstep_planner_);      builder.append(", ");
      builder.append("use_monte_carlo_footstep_planner=");
      builder.append(this.use_monte_carlo_footstep_planner_);      builder.append(", ");
      builder.append("use_previous_plan_as_reference=");
      builder.append(this.use_previous_plan_as_reference_);      builder.append(", ");
      builder.append("use_monte_carlo_plan_as_reference=");
      builder.append(this.use_monte_carlo_plan_as_reference_);      builder.append(", ");
      builder.append("use_joystick_controller=");
      builder.append(this.use_joystick_controller_);      builder.append(", ");
      builder.append("forward_value=");
      builder.append(this.forward_value_);      builder.append(", ");
      builder.append("walk_backwards=");
      builder.append(this.walk_backwards_);      builder.append(", ");
      builder.append("lateral_value=");
      builder.append(this.lateral_value_);      builder.append(", ");
      builder.append("turning_value=");
      builder.append(this.turning_value_);
      builder.append("}");
      return builder.toString();
   }
}
