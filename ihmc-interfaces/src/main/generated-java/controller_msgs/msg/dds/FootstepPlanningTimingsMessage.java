package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPlanningTimingsMessage extends Packet<FootstepPlanningTimingsMessage> implements Settable<FootstepPlanningTimingsMessage>, EpsilonComparable<FootstepPlanningTimingsMessage>
{

   /**
            * Total time measured in the planner process between receiving request message and publishing output message
            */
   public double total_elapsed_seconds_;

   /**
            * Elapsed time between receiving request message and starting to plan body path
            */
   public double time_before_planning_seconds_;

   /**
            * Elapsed time for planning body path
            */
   public double time_planning_body_path_seconds_;

   /**
            * Elapsed time for step planning
            */
   public double time_planning_steps_seconds_;

   /**
            * Number of iterations performed during step planning
            */
   public long step_planning_iterations_;

   public FootstepPlanningTimingsMessage()
   {






   }

   public FootstepPlanningTimingsMessage(FootstepPlanningTimingsMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanningTimingsMessage other)
   {

      total_elapsed_seconds_ = other.total_elapsed_seconds_;


      time_before_planning_seconds_ = other.time_before_planning_seconds_;


      time_planning_body_path_seconds_ = other.time_planning_body_path_seconds_;


      time_planning_steps_seconds_ = other.time_planning_steps_seconds_;


      step_planning_iterations_ = other.step_planning_iterations_;

   }


   /**
            * Total time measured in the planner process between receiving request message and publishing output message
            */
   public void setTotalElapsedSeconds(double total_elapsed_seconds)
   {
      total_elapsed_seconds_ = total_elapsed_seconds;
   }
   /**
            * Total time measured in the planner process between receiving request message and publishing output message
            */
   public double getTotalElapsedSeconds()
   {
      return total_elapsed_seconds_;
   }


   /**
            * Elapsed time between receiving request message and starting to plan body path
            */
   public void setTimeBeforePlanningSeconds(double time_before_planning_seconds)
   {
      time_before_planning_seconds_ = time_before_planning_seconds;
   }
   /**
            * Elapsed time between receiving request message and starting to plan body path
            */
   public double getTimeBeforePlanningSeconds()
   {
      return time_before_planning_seconds_;
   }


   /**
            * Elapsed time for planning body path
            */
   public void setTimePlanningBodyPathSeconds(double time_planning_body_path_seconds)
   {
      time_planning_body_path_seconds_ = time_planning_body_path_seconds;
   }
   /**
            * Elapsed time for planning body path
            */
   public double getTimePlanningBodyPathSeconds()
   {
      return time_planning_body_path_seconds_;
   }


   /**
            * Elapsed time for step planning
            */
   public void setTimePlanningStepsSeconds(double time_planning_steps_seconds)
   {
      time_planning_steps_seconds_ = time_planning_steps_seconds;
   }
   /**
            * Elapsed time for step planning
            */
   public double getTimePlanningStepsSeconds()
   {
      return time_planning_steps_seconds_;
   }


   /**
            * Number of iterations performed during step planning
            */
   public void setStepPlanningIterations(long step_planning_iterations)
   {
      step_planning_iterations_ = step_planning_iterations;
   }
   /**
            * Number of iterations performed during step planning
            */
   public long getStepPlanningIterations()
   {
      return step_planning_iterations_;
   }


   public static Supplier<FootstepPlanningTimingsMessagePubSubType> getPubSubType()
   {
      return FootstepPlanningTimingsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanningTimingsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningTimingsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.total_elapsed_seconds_, other.total_elapsed_seconds_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_before_planning_seconds_, other.time_before_planning_seconds_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_planning_body_path_seconds_, other.time_planning_body_path_seconds_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_planning_steps_seconds_, other.time_planning_steps_seconds_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_planning_iterations_, other.step_planning_iterations_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanningTimingsMessage)) return false;

      FootstepPlanningTimingsMessage otherMyClass = (FootstepPlanningTimingsMessage) other;


      if(this.total_elapsed_seconds_ != otherMyClass.total_elapsed_seconds_) return false;


      if(this.time_before_planning_seconds_ != otherMyClass.time_before_planning_seconds_) return false;


      if(this.time_planning_body_path_seconds_ != otherMyClass.time_planning_body_path_seconds_) return false;


      if(this.time_planning_steps_seconds_ != otherMyClass.time_planning_steps_seconds_) return false;


      if(this.step_planning_iterations_ != otherMyClass.step_planning_iterations_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanningTimingsMessage {");

      builder.append("total_elapsed_seconds=");
      builder.append(this.total_elapsed_seconds_);      builder.append(", ");

      builder.append("time_before_planning_seconds=");
      builder.append(this.time_before_planning_seconds_);      builder.append(", ");

      builder.append("time_planning_body_path_seconds=");
      builder.append(this.time_planning_body_path_seconds_);      builder.append(", ");

      builder.append("time_planning_steps_seconds=");
      builder.append(this.time_planning_steps_seconds_);      builder.append(", ");

      builder.append("step_planning_iterations=");
      builder.append(this.step_planning_iterations_);
      builder.append("}");
      return builder.toString();
   }
}
