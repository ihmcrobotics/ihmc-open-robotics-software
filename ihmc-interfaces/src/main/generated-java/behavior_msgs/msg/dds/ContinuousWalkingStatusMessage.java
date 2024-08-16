package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ContinuousWalkingStatusMessage extends Packet<ContinuousWalkingStatusMessage> implements Settable<ContinuousWalkingStatusMessage>, EpsilonComparable<ContinuousWalkingStatusMessage>
{
   /**
            * flag to check if planning from left stance
            */
   public boolean left_stance_to_plan_from_;
   /**
            * monte-carlo footstep planning time
            */
   public double monte_carlo_planning_time_;
   /**
            * a-star footstep planning time
            */
   public double a_star_planning_time_;
   /**
            * a-star number of footsteps
            */
   public int a_star_num_steps_;
   /**
            * monte-carlo number of footsteps
            */
   public int monte_carlo_num_steps_;

   public ContinuousWalkingStatusMessage()
   {
   }

   public ContinuousWalkingStatusMessage(ContinuousWalkingStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ContinuousWalkingStatusMessage other)
   {
      left_stance_to_plan_from_ = other.left_stance_to_plan_from_;

      monte_carlo_planning_time_ = other.monte_carlo_planning_time_;

      a_star_planning_time_ = other.a_star_planning_time_;

      a_star_num_steps_ = other.a_star_num_steps_;

      monte_carlo_num_steps_ = other.monte_carlo_num_steps_;

   }

   /**
            * flag to check if planning from left stance
            */
   public void setLeftStanceToPlanFrom(boolean left_stance_to_plan_from)
   {
      left_stance_to_plan_from_ = left_stance_to_plan_from;
   }
   /**
            * flag to check if planning from left stance
            */
   public boolean getLeftStanceToPlanFrom()
   {
      return left_stance_to_plan_from_;
   }

   /**
            * monte-carlo footstep planning time
            */
   public void setMonteCarloPlanningTime(double monte_carlo_planning_time)
   {
      monte_carlo_planning_time_ = monte_carlo_planning_time;
   }
   /**
            * monte-carlo footstep planning time
            */
   public double getMonteCarloPlanningTime()
   {
      return monte_carlo_planning_time_;
   }

   /**
            * a-star footstep planning time
            */
   public void setAStarPlanningTime(double a_star_planning_time)
   {
      a_star_planning_time_ = a_star_planning_time;
   }
   /**
            * a-star footstep planning time
            */
   public double getAStarPlanningTime()
   {
      return a_star_planning_time_;
   }

   /**
            * a-star number of footsteps
            */
   public void setAStarNumSteps(int a_star_num_steps)
   {
      a_star_num_steps_ = a_star_num_steps;
   }
   /**
            * a-star number of footsteps
            */
   public int getAStarNumSteps()
   {
      return a_star_num_steps_;
   }

   /**
            * monte-carlo number of footsteps
            */
   public void setMonteCarloNumSteps(int monte_carlo_num_steps)
   {
      monte_carlo_num_steps_ = monte_carlo_num_steps;
   }
   /**
            * monte-carlo number of footsteps
            */
   public int getMonteCarloNumSteps()
   {
      return monte_carlo_num_steps_;
   }


   public static Supplier<ContinuousWalkingStatusMessagePubSubType> getPubSubType()
   {
      return ContinuousWalkingStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContinuousWalkingStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContinuousWalkingStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.left_stance_to_plan_from_, other.left_stance_to_plan_from_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.monte_carlo_planning_time_, other.monte_carlo_planning_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.a_star_planning_time_, other.a_star_planning_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.a_star_num_steps_, other.a_star_num_steps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.monte_carlo_num_steps_, other.monte_carlo_num_steps_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContinuousWalkingStatusMessage)) return false;

      ContinuousWalkingStatusMessage otherMyClass = (ContinuousWalkingStatusMessage) other;

      if(this.left_stance_to_plan_from_ != otherMyClass.left_stance_to_plan_from_) return false;

      if(this.monte_carlo_planning_time_ != otherMyClass.monte_carlo_planning_time_) return false;

      if(this.a_star_planning_time_ != otherMyClass.a_star_planning_time_) return false;

      if(this.a_star_num_steps_ != otherMyClass.a_star_num_steps_) return false;

      if(this.monte_carlo_num_steps_ != otherMyClass.monte_carlo_num_steps_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContinuousWalkingStatusMessage {");
      builder.append("left_stance_to_plan_from=");
      builder.append(this.left_stance_to_plan_from_);      builder.append(", ");
      builder.append("monte_carlo_planning_time=");
      builder.append(this.monte_carlo_planning_time_);      builder.append(", ");
      builder.append("a_star_planning_time=");
      builder.append(this.a_star_planning_time_);      builder.append(", ");
      builder.append("a_star_num_steps=");
      builder.append(this.a_star_num_steps_);      builder.append(", ");
      builder.append("monte_carlo_num_steps=");
      builder.append(this.monte_carlo_num_steps_);
      builder.append("}");
      return builder.toString();
   }
}
