package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ValkyrieFootstepPlanningStatus extends Packet<ValkyrieFootstepPlanningStatus> implements Settable<ValkyrieFootstepPlanningStatus>, EpsilonComparable<ValkyrieFootstepPlanningStatus>
{
   /**
          * Planner is actively planning. If any other status is indicated, the planner has terminated
          */
   public static final byte STATUS_PLANNING = (byte) 0;
   /**
          * A solution was found that reaches the goal
          */
   public static final byte STATUS_FOUND_SOLUTION = (byte) 1;
   /**
          * The timeout was reached and the planner did not find valid solution
          */
   public static final byte STATUS_TIMED_OUT = (byte) 2;
   /**
          * Planner exhaustively searched all possibilities and could not find a plan
          */
   public static final byte STATUS_NO_SOLUTION_EXISTS = (byte) 3;
   /**
          * The goal steps in the request packet are not valid
          */
   public static final byte STATUS_INVALID_GOAL = (byte) 4;
   /**
            * ID that matches the plan_id of the active request packet
            */
   public int plan_id_ = -1;
   /**
            * Current planner status
            */
   public byte planner_status_ = (byte) 255;
   /**
            * Contains the current best solution, i.e. lowest cost + heuristic
            * If the planner found a solution, this contains the solution path
            */
   public controller_msgs.msg.dds.FootstepDataListMessage footstep_data_list_;

   public ValkyrieFootstepPlanningStatus()
   {
      footstep_data_list_ = new controller_msgs.msg.dds.FootstepDataListMessage();
   }

   public ValkyrieFootstepPlanningStatus(ValkyrieFootstepPlanningStatus other)
   {
      this();
      set(other);
   }

   public void set(ValkyrieFootstepPlanningStatus other)
   {
      plan_id_ = other.plan_id_;

      planner_status_ = other.planner_status_;

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.footstep_data_list_, footstep_data_list_);
   }

   /**
            * ID that matches the plan_id of the active request packet
            */
   public void setPlanId(int plan_id)
   {
      plan_id_ = plan_id;
   }
   /**
            * ID that matches the plan_id of the active request packet
            */
   public int getPlanId()
   {
      return plan_id_;
   }

   /**
            * Current planner status
            */
   public void setPlannerStatus(byte planner_status)
   {
      planner_status_ = planner_status;
   }
   /**
            * Current planner status
            */
   public byte getPlannerStatus()
   {
      return planner_status_;
   }


   /**
            * Contains the current best solution, i.e. lowest cost + heuristic
            * If the planner found a solution, this contains the solution path
            */
   public controller_msgs.msg.dds.FootstepDataListMessage getFootstepDataList()
   {
      return footstep_data_list_;
   }


   public static Supplier<ValkyrieFootstepPlanningStatusPubSubType> getPubSubType()
   {
      return ValkyrieFootstepPlanningStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ValkyrieFootstepPlanningStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ValkyrieFootstepPlanningStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_status_, other.planner_status_, epsilon)) return false;

      if (!this.footstep_data_list_.epsilonEquals(other.footstep_data_list_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ValkyrieFootstepPlanningStatus)) return false;

      ValkyrieFootstepPlanningStatus otherMyClass = (ValkyrieFootstepPlanningStatus) other;

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if(this.planner_status_ != otherMyClass.planner_status_) return false;

      if (!this.footstep_data_list_.equals(otherMyClass.footstep_data_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ValkyrieFootstepPlanningStatus {");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("planner_status=");
      builder.append(this.planner_status_);      builder.append(", ");
      builder.append("footstep_data_list=");
      builder.append(this.footstep_data_list_);
      builder.append("}");
      return builder.toString();
   }
}
