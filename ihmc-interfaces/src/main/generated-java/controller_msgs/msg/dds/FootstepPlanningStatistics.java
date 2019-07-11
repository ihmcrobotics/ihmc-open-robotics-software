package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       * This messages gives statistical information about a footstep planning
       */
public class FootstepPlanningStatistics extends Packet<FootstepPlanningStatistics> implements Settable<FootstepPlanningStatistics>, EpsilonComparable<FootstepPlanningStatistics>
{
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_HIGH_OR_LOW = (byte) 0;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_FORWARD_AND_DOWN = (byte) 1;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_FAR = (byte) 2;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_FAR_AND_HIGH = (byte) 3;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_NOT_WIDE_ENOUGH = (byte) 4;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_IN_PLACE = (byte) 5;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_NOT_ENOUGH_AREA = (byte) 6;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_COULD_NOT_SNAP = (byte) 7;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_COULD_NOT_WIGGLE_INSIDE = (byte) 8;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_SURFACE_NORMAL_TOO_STEEP_TO_SNAP = (byte) 9;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_TOO_MUCH_PENETRATION_AFTER_WIGGLE = (byte) 10;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_NOT_LONG_ENOUGH = (byte) 11;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_STEP_TOO_WIDE = (byte) 12;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_OBSTACLE_BLOCKING_BODY = (byte) 13;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_OBSTACLE_HITTING_BODY = (byte) 14;
   public static final byte BIPEDAL_FOOTSTEP_PLANNER_NODE_REJECTION_REASON_AT_CLIFF_BOTTOM = (byte) 15;
   /**
            * Total time taken by the toolbox to generate plan
            */
   public double time_taken_ = -1.0;
   /**
            * Total number of unique steps considered while planning
            */
   public int number_of_steps_considered_;
   /**
            * Array holding the percentage of rejected steps corresponding to each rejection reason
            */
   public us.ihmc.idl.IDLSequence.Double  rejection_fractions_;
   /**
            * Fraction of total considered steps that were rejected
            */
   public double fraction_of_rejected_steps_;

   public FootstepPlanningStatistics()
   {
      rejection_fractions_ = new us.ihmc.idl.IDLSequence.Double (100, "type_6");

   }

   public FootstepPlanningStatistics(FootstepPlanningStatistics other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanningStatistics other)
   {
      time_taken_ = other.time_taken_;

      number_of_steps_considered_ = other.number_of_steps_considered_;

      rejection_fractions_.set(other.rejection_fractions_);
      fraction_of_rejected_steps_ = other.fraction_of_rejected_steps_;

   }

   /**
            * Total time taken by the toolbox to generate plan
            */
   public void setTimeTaken(double time_taken)
   {
      time_taken_ = time_taken;
   }
   /**
            * Total time taken by the toolbox to generate plan
            */
   public double getTimeTaken()
   {
      return time_taken_;
   }

   /**
            * Total number of unique steps considered while planning
            */
   public void setNumberOfStepsConsidered(int number_of_steps_considered)
   {
      number_of_steps_considered_ = number_of_steps_considered;
   }
   /**
            * Total number of unique steps considered while planning
            */
   public int getNumberOfStepsConsidered()
   {
      return number_of_steps_considered_;
   }


   /**
            * Array holding the percentage of rejected steps corresponding to each rejection reason
            */
   public us.ihmc.idl.IDLSequence.Double  getRejectionFractions()
   {
      return rejection_fractions_;
   }

   /**
            * Fraction of total considered steps that were rejected
            */
   public void setFractionOfRejectedSteps(double fraction_of_rejected_steps)
   {
      fraction_of_rejected_steps_ = fraction_of_rejected_steps;
   }
   /**
            * Fraction of total considered steps that were rejected
            */
   public double getFractionOfRejectedSteps()
   {
      return fraction_of_rejected_steps_;
   }


   public static Supplier<FootstepPlanningStatisticsPubSubType> getPubSubType()
   {
      return FootstepPlanningStatisticsPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanningStatisticsPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningStatistics other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_taken_, other.time_taken_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_steps_considered_, other.number_of_steps_considered_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.rejection_fractions_, other.rejection_fractions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.fraction_of_rejected_steps_, other.fraction_of_rejected_steps_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanningStatistics)) return false;

      FootstepPlanningStatistics otherMyClass = (FootstepPlanningStatistics) other;

      if(this.time_taken_ != otherMyClass.time_taken_) return false;

      if(this.number_of_steps_considered_ != otherMyClass.number_of_steps_considered_) return false;

      if (!this.rejection_fractions_.equals(otherMyClass.rejection_fractions_)) return false;
      if(this.fraction_of_rejected_steps_ != otherMyClass.fraction_of_rejected_steps_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanningStatistics {");
      builder.append("time_taken=");
      builder.append(this.time_taken_);      builder.append(", ");
      builder.append("number_of_steps_considered=");
      builder.append(this.number_of_steps_considered_);      builder.append(", ");
      builder.append("rejection_fractions=");
      builder.append(this.rejection_fractions_);      builder.append(", ");
      builder.append("fraction_of_rejected_steps=");
      builder.append(this.fraction_of_rejected_steps_);
      builder.append("}");
      return builder.toString();
   }
}
