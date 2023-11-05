package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ActionNodeStateMessage extends Packet<ActionNodeStateMessage> implements Settable<ActionNodeStateMessage>, EpsilonComparable<ActionNodeStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage state_;
   /**
            * The action's unique ID
            */
   public long id_;
   /**
            * If the action is next for execution
            */
   public boolean is_next_for_execution_;
   /**
            * If the action is to be executed concurrently
            */
   public boolean is_to_be_executed_concurrently_;
   /**
            * If the node is able to execution
            */
   public boolean can_execute_;
   /**
            * If the node is currently executing
            */
   public boolean is_executing_;
   /**
            * Nominal execution duration
            */
   public double nominal_execution_duration_;
   /**
            * Time since execution started
            */
   public double elapsed_execution_time_;
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

   public ActionNodeStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage();
   }

   public ActionNodeStateMessage(ActionNodeStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionNodeStateMessage other)
   {
      behavior_msgs.msg.dds.BehaviorTreeNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      id_ = other.id_;

      is_next_for_execution_ = other.is_next_for_execution_;

      is_to_be_executed_concurrently_ = other.is_to_be_executed_concurrently_;

      can_execute_ = other.can_execute_;

      is_executing_ = other.is_executing_;

      nominal_execution_duration_ = other.nominal_execution_duration_;

      elapsed_execution_time_ = other.elapsed_execution_time_;

      current_position_distance_to_goal_ = other.current_position_distance_to_goal_;

      start_position_distance_to_goal_ = other.start_position_distance_to_goal_;

      position_distance_to_goal_tolerance_ = other.position_distance_to_goal_tolerance_;

      current_orientation_distance_to_goal_ = other.current_orientation_distance_to_goal_;

      start_orientation_distance_to_goal_ = other.start_orientation_distance_to_goal_;

      orientation_distance_to_goal_tolerance_ = other.orientation_distance_to_goal_tolerance_;

   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.BehaviorTreeNodeStateMessage getState()
   {
      return state_;
   }

   /**
            * The action's unique ID
            */
   public void setId(long id)
   {
      id_ = id;
   }
   /**
            * The action's unique ID
            */
   public long getId()
   {
      return id_;
   }

   /**
            * If the action is next for execution
            */
   public void setIsNextForExecution(boolean is_next_for_execution)
   {
      is_next_for_execution_ = is_next_for_execution;
   }
   /**
            * If the action is next for execution
            */
   public boolean getIsNextForExecution()
   {
      return is_next_for_execution_;
   }

   /**
            * If the action is to be executed concurrently
            */
   public void setIsToBeExecutedConcurrently(boolean is_to_be_executed_concurrently)
   {
      is_to_be_executed_concurrently_ = is_to_be_executed_concurrently;
   }
   /**
            * If the action is to be executed concurrently
            */
   public boolean getIsToBeExecutedConcurrently()
   {
      return is_to_be_executed_concurrently_;
   }

   /**
            * If the node is able to execution
            */
   public void setCanExecute(boolean can_execute)
   {
      can_execute_ = can_execute;
   }
   /**
            * If the node is able to execution
            */
   public boolean getCanExecute()
   {
      return can_execute_;
   }

   /**
            * If the node is currently executing
            */
   public void setIsExecuting(boolean is_executing)
   {
      is_executing_ = is_executing;
   }
   /**
            * If the node is currently executing
            */
   public boolean getIsExecuting()
   {
      return is_executing_;
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


   public static Supplier<ActionNodeStateMessagePubSubType> getPubSubType()
   {
      return ActionNodeStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionNodeStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionNodeStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.id_, other.id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_next_for_execution_, other.is_next_for_execution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_to_be_executed_concurrently_, other.is_to_be_executed_concurrently_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.can_execute_, other.can_execute_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_executing_, other.is_executing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nominal_execution_duration_, other.nominal_execution_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.elapsed_execution_time_, other.elapsed_execution_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_position_distance_to_goal_, other.current_position_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_position_distance_to_goal_, other.start_position_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_distance_to_goal_tolerance_, other.position_distance_to_goal_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.current_orientation_distance_to_goal_, other.current_orientation_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.start_orientation_distance_to_goal_, other.start_orientation_distance_to_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.orientation_distance_to_goal_tolerance_, other.orientation_distance_to_goal_tolerance_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionNodeStateMessage)) return false;

      ActionNodeStateMessage otherMyClass = (ActionNodeStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if(this.id_ != otherMyClass.id_) return false;

      if(this.is_next_for_execution_ != otherMyClass.is_next_for_execution_) return false;

      if(this.is_to_be_executed_concurrently_ != otherMyClass.is_to_be_executed_concurrently_) return false;

      if(this.can_execute_ != otherMyClass.can_execute_) return false;

      if(this.is_executing_ != otherMyClass.is_executing_) return false;

      if(this.nominal_execution_duration_ != otherMyClass.nominal_execution_duration_) return false;

      if(this.elapsed_execution_time_ != otherMyClass.elapsed_execution_time_) return false;

      if(this.current_position_distance_to_goal_ != otherMyClass.current_position_distance_to_goal_) return false;

      if(this.start_position_distance_to_goal_ != otherMyClass.start_position_distance_to_goal_) return false;

      if(this.position_distance_to_goal_tolerance_ != otherMyClass.position_distance_to_goal_tolerance_) return false;

      if(this.current_orientation_distance_to_goal_ != otherMyClass.current_orientation_distance_to_goal_) return false;

      if(this.start_orientation_distance_to_goal_ != otherMyClass.start_orientation_distance_to_goal_) return false;

      if(this.orientation_distance_to_goal_tolerance_ != otherMyClass.orientation_distance_to_goal_tolerance_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionNodeStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("id=");
      builder.append(this.id_);      builder.append(", ");
      builder.append("is_next_for_execution=");
      builder.append(this.is_next_for_execution_);      builder.append(", ");
      builder.append("is_to_be_executed_concurrently=");
      builder.append(this.is_to_be_executed_concurrently_);      builder.append(", ");
      builder.append("can_execute=");
      builder.append(this.can_execute_);      builder.append(", ");
      builder.append("is_executing=");
      builder.append(this.is_executing_);      builder.append(", ");
      builder.append("nominal_execution_duration=");
      builder.append(this.nominal_execution_duration_);      builder.append(", ");
      builder.append("elapsed_execution_time=");
      builder.append(this.elapsed_execution_time_);      builder.append(", ");
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
      builder.append(this.orientation_distance_to_goal_tolerance_);
      builder.append("}");
      return builder.toString();
   }
}
