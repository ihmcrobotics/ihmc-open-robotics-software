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
            * Translation current distance to goal
            */
   public double translation_current_distance_to_goal_;
   /**
            * Translation start distance to goal
            */
   public double translation_start_distance_to_goal_;
   /**
            * Orientation current distance to goal
            */
   public double orientation_current_distance_to_goal_;
   /**
            * Orientation start distance to goal
            */
   public double orientation_start_distance_to_goal_;

   public ActionExecutionStatusMessage()
   {
   }

   public ActionExecutionStatusMessage(ActionExecutionStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionExecutionStatusMessage other)
   {
      action_index_ = other.action_index_;

      nominal_execution_duration_ = other.nominal_execution_duration_;

      elapsed_execution_time_ = other.elapsed_execution_time_;

      total_number_of_footsteps_ = other.total_number_of_footsteps_;

      number_of_incomplete_footsteps_ = other.number_of_incomplete_footsteps_;

      translation_current_distance_to_goal_ = other.translation_current_distance_to_goal_;

      translation_start_distance_to_goal_ = other.translation_start_distance_to_goal_;

      orientation_current_distance_to_goal_ = other.orientation_current_distance_to_goal_;

      orientation_start_distance_to_goal_ = other.orientation_start_distance_to_goal_;

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
            * Translation current distance to goal
            */
   public void setTranslationCurrentDistanceToGoal(double translation_current_distance_to_goal)
   {
      translation_current_distance_to_goal_ = translation_current_distance_to_goal;
   }
   /**
            * Translation current distance to goal
            */
   public double getTranslationCurrentDistanceToGoal()
   {
      return translation_current_distance_to_goal_;
   }

   /**
            * Translation start distance to goal
            */
   public void setTranslationStartDistanceToGoal(double translation_start_distance_to_goal)
   {
      translation_start_distance_to_goal_ = translation_start_distance_to_goal;
   }
   /**
            * Translation start distance to goal
            */
   public double getTranslationStartDistanceToGoal()
   {
      return translation_start_distance_to_goal_;
   }

   /**
            * Orientation current distance to goal
            */
   public void setOrientationCurrentDistanceToGoal(double orientation_current_distance_to_goal)
   {
      orientation_current_distance_to_goal_ = orientation_current_distance_to_goal;
   }
   /**
            * Orientation current distance to goal
            */
   public double getOrientationCurrentDistanceToGoal()
   {
      return orientation_current_distance_to_goal_;
   }

   /**
            * Orientation start distance to goal
            */
   public void setOrientationStartDistanceToGoal(double orientation_start_distance_to_goal)
   {
      orientation_start_distance_to_goal_ = orientation_start_distance_to_goal;
   }
   /**
            * Orientation start distance to goal
            */
   public double getOrientationStartDistanceToGoal()
   {
      return orientation_start_distance_to_goal_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nominal_execution_duration_, other.nominal_execution_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.elapsed_execution_time_, other.elapsed_execution_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_number_of_footsteps_, other.total_number_of_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_incomplete_footsteps_, other.number_of_incomplete_footsteps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.translation_current_distance_to_goal_, other.translation_current_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.translation_start_distance_to_goal_, other.translation_start_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_current_distance_to_goal_, other.orientation_current_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_start_distance_to_goal_, other.orientation_start_distance_to_goal_, epsilon)) return false;


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

      if(this.nominal_execution_duration_ != otherMyClass.nominal_execution_duration_) return false;

      if(this.elapsed_execution_time_ != otherMyClass.elapsed_execution_time_) return false;

      if(this.total_number_of_footsteps_ != otherMyClass.total_number_of_footsteps_) return false;

      if(this.number_of_incomplete_footsteps_ != otherMyClass.number_of_incomplete_footsteps_) return false;

      if(this.translation_current_distance_to_goal_ != otherMyClass.translation_current_distance_to_goal_) return false;

      if(this.translation_start_distance_to_goal_ != otherMyClass.translation_start_distance_to_goal_) return false;

      if(this.orientation_current_distance_to_goal_ != otherMyClass.orientation_current_distance_to_goal_) return false;

      if(this.orientation_start_distance_to_goal_ != otherMyClass.orientation_start_distance_to_goal_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionExecutionStatusMessage {");
      builder.append("action_index=");
      builder.append(this.action_index_);      builder.append(", ");
      builder.append("nominal_execution_duration=");
      builder.append(this.nominal_execution_duration_);      builder.append(", ");
      builder.append("elapsed_execution_time=");
      builder.append(this.elapsed_execution_time_);      builder.append(", ");
      builder.append("total_number_of_footsteps=");
      builder.append(this.total_number_of_footsteps_);      builder.append(", ");
      builder.append("number_of_incomplete_footsteps=");
      builder.append(this.number_of_incomplete_footsteps_);      builder.append(", ");
      builder.append("translation_current_distance_to_goal=");
      builder.append(this.translation_current_distance_to_goal_);      builder.append(", ");
      builder.append("translation_start_distance_to_goal=");
      builder.append(this.translation_start_distance_to_goal_);      builder.append(", ");
      builder.append("orientation_current_distance_to_goal=");
      builder.append(this.orientation_current_distance_to_goal_);      builder.append(", ");
      builder.append("orientation_start_distance_to_goal=");
      builder.append(this.orientation_start_distance_to_goal_);
      builder.append("}");
      return builder.toString();
   }
}
