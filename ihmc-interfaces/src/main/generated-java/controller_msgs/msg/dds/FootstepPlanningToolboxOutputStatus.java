package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPlanningToolboxOutputStatus extends Packet<FootstepPlanningToolboxOutputStatus> implements Settable<FootstepPlanningToolboxOutputStatus>, EpsilonComparable<FootstepPlanningToolboxOutputStatus>
{
   public static final byte FOOTSTEP_PLANNING_RESULT_OPTIMAL_SOLUTION = (byte) 0;
   public static final byte FOOTSTEP_PLANNING_RESULT_SUB_OPTIMAL_SOLUTION = (byte) 1;
   public static final byte FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = (byte) 2;
   public static final byte FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS = (byte) 3;
   public static final byte FOOTSTEP_PLANNING_RESULT_SNAPPING_FAILED = (byte) 4;
   public static final byte FOOTSTEP_PLANNING_RESULT_PLANNER_FAILED = (byte) 5;
   public static final byte FOOTSTEP_PLANNING_RESULT_INVALID_GOAL = (byte) 6;
   public static final int NO_PLAN_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public controller_msgs.msg.dds.FootstepDataListMessage footstep_data_list_;
   public byte footstep_planning_result_ = (byte) 255;
   public int plan_id_ = -1;
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  body_path_;
   public us.ihmc.euclid.geometry.Pose3D low_level_planner_goal_;
   public controller_msgs.msg.dds.FootstepPlanningStatistics footstep_planning_statistics_;

   public FootstepPlanningToolboxOutputStatus()
   {
      footstep_data_list_ = new controller_msgs.msg.dds.FootstepDataListMessage();
      planar_regions_list_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();
      body_path_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());
      low_level_planner_goal_ = new us.ihmc.euclid.geometry.Pose3D();
      footstep_planning_statistics_ = new controller_msgs.msg.dds.FootstepPlanningStatistics();

   }

   public FootstepPlanningToolboxOutputStatus(FootstepPlanningToolboxOutputStatus other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanningToolboxOutputStatus other)
   {
      sequence_id_ = other.sequence_id_;

      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.footstep_data_list_, footstep_data_list_);
      footstep_planning_result_ = other.footstep_planning_result_;

      plan_id_ = other.plan_id_;

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_, planar_regions_list_);
      body_path_.set(other.body_path_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.low_level_planner_goal_, low_level_planner_goal_);
      controller_msgs.msg.dds.FootstepPlanningStatisticsPubSubType.staticCopy(other.footstep_planning_statistics_, footstep_planning_statistics_);
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


   public controller_msgs.msg.dds.FootstepDataListMessage getFootstepDataList()
   {
      return footstep_data_list_;
   }

   public void setFootstepPlanningResult(byte footstep_planning_result)
   {
      footstep_planning_result_ = footstep_planning_result;
   }
   public byte getFootstepPlanningResult()
   {
      return footstep_planning_result_;
   }

   public void setPlanId(int plan_id)
   {
      plan_id_ = plan_id;
   }
   public int getPlanId()
   {
      return plan_id_;
   }


   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsList()
   {
      return planar_regions_list_;
   }


   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getBodyPath()
   {
      return body_path_;
   }


   public us.ihmc.euclid.geometry.Pose3D getLowLevelPlannerGoal()
   {
      return low_level_planner_goal_;
   }


   public controller_msgs.msg.dds.FootstepPlanningStatistics getFootstepPlanningStatistics()
   {
      return footstep_planning_statistics_;
   }


   public static Supplier<FootstepPlanningToolboxOutputStatusPubSubType> getPubSubType()
   {
      return FootstepPlanningToolboxOutputStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanningToolboxOutputStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.footstep_data_list_.epsilonEquals(other.footstep_data_list_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_planning_result_, other.footstep_planning_result_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (!this.planar_regions_list_.epsilonEquals(other.planar_regions_list_, epsilon)) return false;
      if (this.body_path_.size() != other.body_path_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.body_path_.size(); i++)
         {  if (!this.body_path_.get(i).epsilonEquals(other.body_path_.get(i), epsilon)) return false; }
      }

      if (!this.low_level_planner_goal_.epsilonEquals(other.low_level_planner_goal_, epsilon)) return false;
      if (!this.footstep_planning_statistics_.epsilonEquals(other.footstep_planning_statistics_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanningToolboxOutputStatus)) return false;

      FootstepPlanningToolboxOutputStatus otherMyClass = (FootstepPlanningToolboxOutputStatus) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.footstep_data_list_.equals(otherMyClass.footstep_data_list_)) return false;
      if(this.footstep_planning_result_ != otherMyClass.footstep_planning_result_) return false;

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if (!this.planar_regions_list_.equals(otherMyClass.planar_regions_list_)) return false;
      if (!this.body_path_.equals(otherMyClass.body_path_)) return false;
      if (!this.low_level_planner_goal_.equals(otherMyClass.low_level_planner_goal_)) return false;
      if (!this.footstep_planning_statistics_.equals(otherMyClass.footstep_planning_statistics_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanningToolboxOutputStatus {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("footstep_data_list=");
      builder.append(this.footstep_data_list_);      builder.append(", ");
      builder.append("footstep_planning_result=");
      builder.append(this.footstep_planning_result_);      builder.append(", ");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("planar_regions_list=");
      builder.append(this.planar_regions_list_);      builder.append(", ");
      builder.append("body_path=");
      builder.append(this.body_path_);      builder.append(", ");
      builder.append("low_level_planner_goal=");
      builder.append(this.low_level_planner_goal_);      builder.append(", ");
      builder.append("footstep_planning_statistics=");
      builder.append(this.footstep_planning_statistics_);
      builder.append("}");
      return builder.toString();
   }
}
