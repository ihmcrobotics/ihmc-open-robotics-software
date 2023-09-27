package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is meant to communicate the status of currently executing actions
       */
public class ActionExecutionStatusMessage extends Packet<ActionExecutionStatusMessage> implements Settable<ActionExecutionStatusMessage>, EpsilonComparable<ActionExecutionStatusMessage>
{
   /**
            * Executing action index
            */
   public int action_index_;
   public java.lang.StringBuilder execution_rejection_tooltip_;
   /**
            * Nominal execution duration
            */
   public double nominal_execution_duration_;
   /**
            * Time since execution started
            */
   public double elapsed_execution_time_;
   /**
            * Total number of footsteps; used for walking actions
            */
   public int total_number_of_footsteps_;
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int number_of_incomplete_footsteps_;
   /**
            * Current position distance to goal
            */
   public double current_position_distance_to_goal_;
   /**
            * Start position distance to goal
            */
   public double start_position_distance_to_goal_;
   /**
            * Position distance to goal tolerance
            */
   public double position_distance_to_goal_tolerance_;
   /**
            * Current orientation distance to goal
            */
   public double current_orientation_distance_to_goal_;
   /**
            * Start orientation distance to goal
            */
   public double start_orientation_distance_to_goal_;
   /**
            * Orientation distance to goal tolerance
            */
   public double orientation_distance_to_goal_tolerance_;
   /**
            * Linear hand wrench magnitude
            */
   public double hand_wrench_magnitude_linear_;

   public ActionExecutionStatusMessage()
   {
      execution_rejection_tooltip_ = new java.lang.StringBuilder(255);
   }

   public ActionExecutionStatusMessage(ActionExecutionStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionExecutionStatusMessage other)
   {
      action_index_ = other.action_index_;

      execution_rejection_tooltip_.setLength(0);
      execution_rejection_tooltip_.append(other.execution_rejection_tooltip_);

      nominal_execution_duration_ = other.nominal_execution_duration_;

      elapsed_execution_time_ = other.elapsed_execution_time_;

      total_number_of_footsteps_ = other.total_number_of_footsteps_;

      number_of_incomplete_footsteps_ = other.number_of_incomplete_footsteps_;

      current_position_distance_to_goal_ = other.current_position_distance_to_goal_;

      start_position_distance_to_goal_ = other.start_position_distance_to_goal_;

      position_distance_to_goal_tolerance_ = other.position_distance_to_goal_tolerance_;

      current_orientation_distance_to_goal_ = other.current_orientation_distance_to_goal_;

      start_orientation_distance_to_goal_ = other.start_orientation_distance_to_goal_;

      orientation_distance_to_goal_tolerance_ = other.orientation_distance_to_goal_tolerance_;

      hand_wrench_magnitude_linear_ = other.hand_wrench_magnitude_linear_;

   }

   /**
            * Executing action index
            */
   public void setActionIndex(int action_index)
   {
      action_index_ = action_index;
   }
   /**
            * Executing action index
            */
   public int getActionIndex()
   {
      return action_index_;
   }

   public void setExecutionRejectionTooltip(java.lang.String execution_rejection_tooltip)
   {
      execution_rejection_tooltip_.setLength(0);
      execution_rejection_tooltip_.append(execution_rejection_tooltip);
   }

   public java.lang.String getExecutionRejectionTooltipAsString()
   {
      return getExecutionRejectionTooltip().toString();
   }
   public java.lang.StringBuilder getExecutionRejectionTooltip()
   {
      return execution_rejection_tooltip_;
   }

   /**
            * Nominal execution duration
            */
   public void setNominalExecutionDuration(double nominal_execution_duration)
   {
      nominal_execution_duration_ = nominal_execution_duration;
   }
   /**
            * Nominal execution duration
            */
   public double getNominalExecutionDuration()
   {
      return nominal_execution_duration_;
   }

   /**
            * Time since execution started
            */
   public void setElapsedExecutionTime(double elapsed_execution_time)
   {
      elapsed_execution_time_ = elapsed_execution_time;
   }
   /**
            * Time since execution started
            */
   public double getElapsedExecutionTime()
   {
      return elapsed_execution_time_;
   }

   /**
            * Total number of footsteps; used for walking actions
            */
   public void setTotalNumberOfFootsteps(int total_number_of_footsteps)
   {
      total_number_of_footsteps_ = total_number_of_footsteps;
   }
   /**
            * Total number of footsteps; used for walking actions
            */
   public int getTotalNumberOfFootsteps()
   {
      return total_number_of_footsteps_;
   }

   /**
            * Incomplete footsteps; used for walking actions
            */
   public void setNumberOfIncompleteFootsteps(int number_of_incomplete_footsteps)
   {
      number_of_incomplete_footsteps_ = number_of_incomplete_footsteps;
   }
   /**
            * Incomplete footsteps; used for walking actions
            */
   public int getNumberOfIncompleteFootsteps()
   {
      return number_of_incomplete_footsteps_;
   }

   /**
            * Current position distance to goal
            */
   public void setCurrentPositionDistanceToGoal(double current_position_distance_to_goal)
   {
      current_position_distance_to_goal_ = current_position_distance_to_goal;
   }
   /**
            * Current position distance to goal
            */
   public double getCurrentPositionDistanceToGoal()
   {
      return current_position_distance_to_goal_;
   }

   /**
            * Start position distance to goal
            */
   public void setStartPositionDistanceToGoal(double start_position_distance_to_goal)
   {
      start_position_distance_to_goal_ = start_position_distance_to_goal;
   }
   /**
            * Start position distance to goal
            */
   public double getStartPositionDistanceToGoal()
   {
      return start_position_distance_to_goal_;
   }

   /**
            * Position distance to goal tolerance
            */
   public void setPositionDistanceToGoalTolerance(double position_distance_to_goal_tolerance)
   {
      position_distance_to_goal_tolerance_ = position_distance_to_goal_tolerance;
   }
   /**
            * Position distance to goal tolerance
            */
   public double getPositionDistanceToGoalTolerance()
   {
      return position_distance_to_goal_tolerance_;
   }

   /**
            * Current orientation distance to goal
            */
   public void setCurrentOrientationDistanceToGoal(double current_orientation_distance_to_goal)
   {
      current_orientation_distance_to_goal_ = current_orientation_distance_to_goal;
   }
   /**
            * Current orientation distance to goal
            */
   public double getCurrentOrientationDistanceToGoal()
   {
      return current_orientation_distance_to_goal_;
   }

   /**
            * Start orientation distance to goal
            */
   public void setStartOrientationDistanceToGoal(double start_orientation_distance_to_goal)
   {
      start_orientation_distance_to_goal_ = start_orientation_distance_to_goal;
   }
   /**
            * Start orientation distance to goal
            */
   public double getStartOrientationDistanceToGoal()
   {
      return start_orientation_distance_to_goal_;
   }

   /**
            * Orientation distance to goal tolerance
            */
   public void setOrientationDistanceToGoalTolerance(double orientation_distance_to_goal_tolerance)
   {
      orientation_distance_to_goal_tolerance_ = orientation_distance_to_goal_tolerance;
   }
   /**
            * Orientation distance to goal tolerance
            */
   public double getOrientationDistanceToGoalTolerance()
   {
      return orientation_distance_to_goal_tolerance_;
   }

   /**
            * Linear hand wrench magnitude
            */
   public void setHandWrenchMagnitudeLinear(double hand_wrench_magnitude_linear)
   {
      hand_wrench_magnitude_linear_ = hand_wrench_magnitude_linear;
   }
   /**
            * Linear hand wrench magnitude
            */
   public double getHandWrenchMagnitudeLinear()
   {
      return hand_wrench_magnitude_linear_;
   }


   public static Supplier<ActionExecutionStatusMessagePubSubType> getPubSubType()
   {
      return ActionExecutionStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionExecutionStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionExecutionStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.action_index_, other.action_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.execution_rejection_tooltip_, other.execution_rejection_tooltip_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nominal_execution_duration_, other.nominal_execution_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.elapsed_execution_time_, other.elapsed_execution_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_number_of_footsteps_, other.total_number_of_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_incomplete_footsteps_, other.number_of_incomplete_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_distance_to_goal_, other.current_position_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_position_distance_to_goal_, other.start_position_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_distance_to_goal_tolerance_, other.position_distance_to_goal_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_orientation_distance_to_goal_, other.current_orientation_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_orientation_distance_to_goal_, other.start_orientation_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_distance_to_goal_tolerance_, other.orientation_distance_to_goal_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.hand_wrench_magnitude_linear_, other.hand_wrench_magnitude_linear_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionExecutionStatusMessage)) return false;

      ActionExecutionStatusMessage otherMyClass = (ActionExecutionStatusMessage) other;

      if(this.action_index_ != otherMyClass.action_index_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.execution_rejection_tooltip_, otherMyClass.execution_rejection_tooltip_)) return false;

      if(this.nominal_execution_duration_ != otherMyClass.nominal_execution_duration_) return false;

      if(this.elapsed_execution_time_ != otherMyClass.elapsed_execution_time_) return false;

      if(this.total_number_of_footsteps_ != otherMyClass.total_number_of_footsteps_) return false;

      if(this.number_of_incomplete_footsteps_ != otherMyClass.number_of_incomplete_footsteps_) return false;

      if(this.current_position_distance_to_goal_ != otherMyClass.current_position_distance_to_goal_) return false;

      if(this.start_position_distance_to_goal_ != otherMyClass.start_position_distance_to_goal_) return false;

      if(this.position_distance_to_goal_tolerance_ != otherMyClass.position_distance_to_goal_tolerance_) return false;

      if(this.current_orientation_distance_to_goal_ != otherMyClass.current_orientation_distance_to_goal_) return false;

      if(this.start_orientation_distance_to_goal_ != otherMyClass.start_orientation_distance_to_goal_) return false;

      if(this.orientation_distance_to_goal_tolerance_ != otherMyClass.orientation_distance_to_goal_tolerance_) return false;

      if(this.hand_wrench_magnitude_linear_ != otherMyClass.hand_wrench_magnitude_linear_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionExecutionStatusMessage {");
      builder.append("action_index=");
      builder.append(this.action_index_);      builder.append(", ");
      builder.append("execution_rejection_tooltip=");
      builder.append(this.execution_rejection_tooltip_);      builder.append(", ");
      builder.append("nominal_execution_duration=");
      builder.append(this.nominal_execution_duration_);      builder.append(", ");
      builder.append("elapsed_execution_time=");
      builder.append(this.elapsed_execution_time_);      builder.append(", ");
      builder.append("total_number_of_footsteps=");
      builder.append(this.total_number_of_footsteps_);      builder.append(", ");
      builder.append("number_of_incomplete_footsteps=");
      builder.append(this.number_of_incomplete_footsteps_);      builder.append(", ");
      builder.append("current_position_distance_to_goal=");
      builder.append(this.current_position_distance_to_goal_);      builder.append(", ");
      builder.append("start_position_distance_to_goal=");
      builder.append(this.start_position_distance_to_goal_);      builder.append(", ");
      builder.append("position_distance_to_goal_tolerance=");
      builder.append(this.position_distance_to_goal_tolerance_);      builder.append(", ");
      builder.append("current_orientation_distance_to_goal=");
      builder.append(this.current_orientation_distance_to_goal_);      builder.append(", ");
      builder.append("start_orientation_distance_to_goal=");
      builder.append(this.start_orientation_distance_to_goal_);      builder.append(", ");
      builder.append("orientation_distance_to_goal_tolerance=");
      builder.append(this.orientation_distance_to_goal_tolerance_);      builder.append(", ");
      builder.append("hand_wrench_magnitude_linear=");
      builder.append(this.hand_wrench_magnitude_linear_);
      builder.append("}");
      return builder.toString();
   }
}
