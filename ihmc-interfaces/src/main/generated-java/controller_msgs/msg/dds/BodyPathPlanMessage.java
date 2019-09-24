package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class BodyPathPlanMessage extends Packet<BodyPathPlanMessage> implements Settable<BodyPathPlanMessage>, EpsilonComparable<BodyPathPlanMessage>
{
   public static final byte FOOTSTEP_PLANNING_RESULT_OPTIMAL_SOLUTION = (byte) 0;
   public static final byte FOOTSTEP_PLANNING_RESULT_SUB_OPTIMAL_SOLUTION = (byte) 1;
   public static final byte FOOTSTEP_PLANNING_RESULT_TIMED_OUT_BEFORE_SOLUTION = (byte) 2;
   public static final byte FOOTSTEP_PLANNING_RESULT_NO_PATH_EXISTS = (byte) 3;
   public static final byte FOOTSTEP_PLANNING_RESULT_SNAPPING_FAILED = (byte) 4;
   public static final byte FOOTSTEP_PLANNING_RESULT_PLANNER_FAILED = (byte) 5;
   public static final int NO_PLAN_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public byte footstep_planning_result_ = (byte) 255;
   public int plan_id_ = -1;
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_;
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  body_path_;
   public us.ihmc.euclid.geometry.Pose2D path_planner_start_pose_;
   public us.ihmc.euclid.geometry.Pose2D path_planner_goal_pose_;

   public BodyPathPlanMessage()
   {
      planar_regions_list_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();
      body_path_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (100, new geometry_msgs.msg.dds.PosePubSubType());
      path_planner_start_pose_ = new us.ihmc.euclid.geometry.Pose2D();
      path_planner_goal_pose_ = new us.ihmc.euclid.geometry.Pose2D();

   }

   public BodyPathPlanMessage(BodyPathPlanMessage other)
   {
      this();
      set(other);
   }

   public void set(BodyPathPlanMessage other)
   {
      sequence_id_ = other.sequence_id_;

      footstep_planning_result_ = other.footstep_planning_result_;

      plan_id_ = other.plan_id_;

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_, planar_regions_list_);
      body_path_.set(other.body_path_);
      geometry_msgs.msg.dds.Pose2DPubSubType.staticCopy(other.path_planner_start_pose_, path_planner_start_pose_);
      geometry_msgs.msg.dds.Pose2DPubSubType.staticCopy(other.path_planner_goal_pose_, path_planner_goal_pose_);
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


   public us.ihmc.euclid.geometry.Pose2D getPathPlannerStartPose()
   {
      return path_planner_start_pose_;
   }


   public us.ihmc.euclid.geometry.Pose2D getPathPlannerGoalPose()
   {
      return path_planner_goal_pose_;
   }


   public static Supplier<BodyPathPlanMessagePubSubType> getPubSubType()
   {
      return BodyPathPlanMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BodyPathPlanMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BodyPathPlanMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_planning_result_, other.footstep_planning_result_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.plan_id_, other.plan_id_, epsilon)) return false;

      if (!this.planar_regions_list_.epsilonEquals(other.planar_regions_list_, epsilon)) return false;
      if (this.body_path_.size() != other.body_path_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.body_path_.size(); i++)
         {  if (!this.body_path_.get(i).epsilonEquals(other.body_path_.get(i), epsilon)) return false; }
      }

      if (!this.path_planner_start_pose_.epsilonEquals(other.path_planner_start_pose_, epsilon)) return false;
      if (!this.path_planner_goal_pose_.epsilonEquals(other.path_planner_goal_pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BodyPathPlanMessage)) return false;

      BodyPathPlanMessage otherMyClass = (BodyPathPlanMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.footstep_planning_result_ != otherMyClass.footstep_planning_result_) return false;

      if(this.plan_id_ != otherMyClass.plan_id_) return false;

      if (!this.planar_regions_list_.equals(otherMyClass.planar_regions_list_)) return false;
      if (!this.body_path_.equals(otherMyClass.body_path_)) return false;
      if (!this.path_planner_start_pose_.equals(otherMyClass.path_planner_start_pose_)) return false;
      if (!this.path_planner_goal_pose_.equals(otherMyClass.path_planner_goal_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BodyPathPlanMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("footstep_planning_result=");
      builder.append(this.footstep_planning_result_);      builder.append(", ");
      builder.append("plan_id=");
      builder.append(this.plan_id_);      builder.append(", ");
      builder.append("planar_regions_list=");
      builder.append(this.planar_regions_list_);      builder.append(", ");
      builder.append("body_path=");
      builder.append(this.body_path_);      builder.append(", ");
      builder.append("path_planner_start_pose=");
      builder.append(this.path_planner_start_pose_);      builder.append(", ");
      builder.append("path_planner_goal_pose=");
      builder.append(this.path_planner_goal_pose_);
      builder.append("}");
      return builder.toString();
   }
}
