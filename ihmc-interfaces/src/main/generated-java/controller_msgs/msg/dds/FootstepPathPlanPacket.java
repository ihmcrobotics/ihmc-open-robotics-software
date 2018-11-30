package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * (Obsolete) This message is part of the old IHMC footstep planning module.
       */
public class FootstepPathPlanPacket extends Packet<FootstepPathPlanPacket> implements Settable<FootstepPathPlanPacket>, EpsilonComparable<FootstepPathPlanPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public boolean goals_valid_;
   public controller_msgs.msg.dds.FootstepDataMessage start_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  original_goals_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  path_plan_;
   public us.ihmc.idl.IDLSequence.Boolean  footstep_unknown_;
   public double sub_optimality_;
   public double path_cost_ = 1000000.0;

   public FootstepPathPlanPacket()
   {
      start_ = new controller_msgs.msg.dds.FootstepDataMessage();
      original_goals_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage> (100, new controller_msgs.msg.dds.FootstepDataMessagePubSubType());
      path_plan_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage> (100, new controller_msgs.msg.dds.FootstepDataMessagePubSubType());
      footstep_unknown_ = new us.ihmc.idl.IDLSequence.Boolean (100, "type_7");


   }

   public FootstepPathPlanPacket(FootstepPathPlanPacket other)
   {
      this();
      set(other);
   }

   public void set(FootstepPathPlanPacket other)
   {
      sequence_id_ = other.sequence_id_;

      goals_valid_ = other.goals_valid_;

      controller_msgs.msg.dds.FootstepDataMessagePubSubType.staticCopy(other.start_, start_);
      original_goals_.set(other.original_goals_);
      path_plan_.set(other.path_plan_);
      footstep_unknown_.set(other.footstep_unknown_);
      sub_optimality_ = other.sub_optimality_;

      path_cost_ = other.path_cost_;

   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   public void setGoalsValid(boolean goals_valid)
   {
      goals_valid_ = goals_valid;
   }
   public boolean getGoalsValid()
   {
      return goals_valid_;
   }


   public controller_msgs.msg.dds.FootstepDataMessage getStart()
   {
      return start_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  getOriginalGoals()
   {
      return original_goals_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  getPathPlan()
   {
      return path_plan_;
   }


   public us.ihmc.idl.IDLSequence.Boolean  getFootstepUnknown()
   {
      return footstep_unknown_;
   }

   public void setSubOptimality(double sub_optimality)
   {
      sub_optimality_ = sub_optimality;
   }
   public double getSubOptimality()
   {
      return sub_optimality_;
   }

   public void setPathCost(double path_cost)
   {
      path_cost_ = path_cost;
   }
   public double getPathCost()
   {
      return path_cost_;
   }


   public static Supplier<FootstepPathPlanPacketPubSubType> getPubSubType()
   {
      return FootstepPathPlanPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPathPlanPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPathPlanPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.goals_valid_, other.goals_valid_, epsilon)) return false;

      if (!this.start_.epsilonEquals(other.start_, epsilon)) return false;
      if (this.original_goals_.size() != other.original_goals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.original_goals_.size(); i++)
         {  if (!this.original_goals_.get(i).epsilonEquals(other.original_goals_.get(i), epsilon)) return false; }
      }

      if (this.path_plan_.size() != other.path_plan_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.path_plan_.size(); i++)
         {  if (!this.path_plan_.get(i).epsilonEquals(other.path_plan_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBooleanSequence(this.footstep_unknown_, other.footstep_unknown_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sub_optimality_, other.sub_optimality_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.path_cost_, other.path_cost_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPathPlanPacket)) return false;

      FootstepPathPlanPacket otherMyClass = (FootstepPathPlanPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.goals_valid_ != otherMyClass.goals_valid_) return false;

      if (!this.start_.equals(otherMyClass.start_)) return false;
      if (!this.original_goals_.equals(otherMyClass.original_goals_)) return false;
      if (!this.path_plan_.equals(otherMyClass.path_plan_)) return false;
      if (!this.footstep_unknown_.equals(otherMyClass.footstep_unknown_)) return false;
      if(this.sub_optimality_ != otherMyClass.sub_optimality_) return false;

      if(this.path_cost_ != otherMyClass.path_cost_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPathPlanPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("goals_valid=");
      builder.append(this.goals_valid_);      builder.append(", ");
      builder.append("start=");
      builder.append(this.start_);      builder.append(", ");
      builder.append("original_goals=");
      builder.append(this.original_goals_);      builder.append(", ");
      builder.append("path_plan=");
      builder.append(this.path_plan_);      builder.append(", ");
      builder.append("footstep_unknown=");
      builder.append(this.footstep_unknown_);      builder.append(", ");
      builder.append("sub_optimality=");
      builder.append(this.sub_optimality_);      builder.append(", ");
      builder.append("path_cost=");
      builder.append(this.path_cost_);
      builder.append("}");
      return builder.toString();
   }
}
