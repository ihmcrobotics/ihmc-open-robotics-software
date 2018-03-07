package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * (Obsolete) This message is part of the old IHMC footstep planning module.
 */
public class FootstepPlanRequestPacket implements Settable<FootstepPlanRequestPacket>, EpsilonComparable<FootstepPlanRequestPacket>
{
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_START_SEARCH = (byte) 0;
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_STOP_SEARCH = (byte) 1;
   public static final byte FOOTSTEP_PLAN_REQUEST_TYPE_UPDATE_START = (byte) 2;
   private controller_msgs.msg.dds.FootstepDataMessage start_footstep_;
   private double theta_start_;
   private double max_sub_optimality_ = 1.0;
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage> goals_;
   private byte footstep_plan_request_type_ = (byte) 255;

   public FootstepPlanRequestPacket()
   {
      start_footstep_ = new controller_msgs.msg.dds.FootstepDataMessage();

      goals_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>(100, controller_msgs.msg.dds.FootstepDataMessage.class,
                                                                                               new controller_msgs.msg.dds.FootstepDataMessagePubSubType());
   }

   public FootstepPlanRequestPacket(FootstepPlanRequestPacket other)
   {
      set(other);
   }

   public void set(FootstepPlanRequestPacket other)
   {
      controller_msgs.msg.dds.FootstepDataMessagePubSubType.staticCopy(other.start_footstep_, start_footstep_);
      theta_start_ = other.theta_start_;

      max_sub_optimality_ = other.max_sub_optimality_;

      goals_.set(other.goals_);
      footstep_plan_request_type_ = other.footstep_plan_request_type_;
   }

   public controller_msgs.msg.dds.FootstepDataMessage getStartFootstep()
   {
      return start_footstep_;
   }

   public double getThetaStart()
   {
      return theta_start_;
   }

   public void setThetaStart(double theta_start)
   {
      theta_start_ = theta_start;
   }

   public double getMaxSubOptimality()
   {
      return max_sub_optimality_;
   }

   public void setMaxSubOptimality(double max_sub_optimality)
   {
      max_sub_optimality_ = max_sub_optimality;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage> getGoals()
   {
      return goals_;
   }

   public byte getFootstepPlanRequestType()
   {
      return footstep_plan_request_type_;
   }

   public void setFootstepPlanRequestType(byte footstep_plan_request_type)
   {
      footstep_plan_request_type_ = footstep_plan_request_type;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanRequestPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.start_footstep_.epsilonEquals(other.start_footstep_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.theta_start_, other.theta_start_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_sub_optimality_, other.max_sub_optimality_, epsilon))
         return false;

      if (this.goals_.size() == other.goals_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.goals_.size(); i++)
         {
            if (!this.goals_.get(i).epsilonEquals(other.goals_.get(i), epsilon))
               return false;
         }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_plan_request_type_, other.footstep_plan_request_type_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof FootstepPlanRequestPacket))
         return false;

      FootstepPlanRequestPacket otherMyClass = (FootstepPlanRequestPacket) other;

      if (!this.start_footstep_.equals(otherMyClass.start_footstep_))
         return false;

      if (this.theta_start_ != otherMyClass.theta_start_)
         return false;

      if (this.max_sub_optimality_ != otherMyClass.max_sub_optimality_)
         return false;

      if (!this.goals_.equals(otherMyClass.goals_))
         return false;

      if (this.footstep_plan_request_type_ != otherMyClass.footstep_plan_request_type_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanRequestPacket {");
      builder.append("start_footstep=");
      builder.append(this.start_footstep_);

      builder.append(", ");
      builder.append("theta_start=");
      builder.append(this.theta_start_);

      builder.append(", ");
      builder.append("max_sub_optimality=");
      builder.append(this.max_sub_optimality_);

      builder.append(", ");
      builder.append("goals=");
      builder.append(this.goals_);

      builder.append(", ");
      builder.append("footstep_plan_request_type=");
      builder.append(this.footstep_plan_request_type_);

      builder.append("}");
      return builder.toString();
   }
}