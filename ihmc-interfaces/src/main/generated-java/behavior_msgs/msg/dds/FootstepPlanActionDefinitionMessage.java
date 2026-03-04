package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class FootstepPlanActionDefinitionMessage extends Packet<FootstepPlanActionDefinitionMessage> implements Settable<FootstepPlanActionDefinitionMessage>, EpsilonComparable<FootstepPlanActionDefinitionMessage>
{
   /**
          * Quick footstep planner
          */
   public static final byte QUICK = (byte) 0;
   /**
          * Turn walk turn planner
          */
   public static final byte TURN_WALK_TURN = (byte) 1;
   /**
          * A star planner
          */
   public static final byte A_STAR = (byte) 2;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * Swing duration
            */
   public double swing_duration_;
   /**
            * Transfer duration
            */
   public double transfer_duration_;
   /**
            * OVERRIDE (0) or QUEUE (1)
            */
   public byte execution_mode_;
   /**
            * Is using the footstep planner or manually placing
            */
   public boolean is_manually_placed_;
   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage>  footsteps_;
   /**
            * The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D goal_stance_point_;
   /**
            * The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D goal_focal_point_;
   /**
            * Left goal foot X to the goal pose
            */
   public double left_goal_foot_x_to_gizmo_;
   /**
            * Left goal foot Y to the goal pose
            */
   public double left_goal_foot_y_to_gizmo_;
   /**
            * Left goal foot Yaw to the goal pose
            */
   public double left_goal_foot_yaw_to_gizmo_;
   /**
            * Right goal foot X to the goal pose
            */
   public double right_goal_foot_x_to_gizmo_;
   /**
            * Right goal foot Y to the goal pose
            */
   public double right_goal_foot_y_to_gizmo_;
   /**
            * Right goal foot Yaw to the goal pose
            */
   public double right_goal_foot_yaw_to_gizmo_;
   /**
            * Initial stance side when planning
            */
   public byte planner_initial_stance_side_;
   /**
            * The type of planner as defined above
            */
   public byte planner_;
   /**
            * Whether to walk, turning first, matching the goal orientation
            */
   public boolean planner_walk_with_goal_orientation_;
   /**
            * Whether to use the body path planner or not
            */
   public boolean planner_plan_with_body_path_;
   /**
            * The footstep planner parameters
            */
   public ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage planner_parameters_;
   /**
            * Quick footstep planner parameters
            */
   public double quick_hip_width_;
   public double quick_step_length_;
   public double quick_next_pelvis_yaw_limit_;
   public double quick_inward_limit_;
   public double quick_outward_limit_;
   public double quick_step_angle_limit_;
   public double quick_swing_time_distance_lower_;
   public double quick_swing_time_distance_upper_;
   public double quick_min_swing_time_;
   public double quick_max_swing_time_;

   public FootstepPlanActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      parent_frame_name_ = new java.lang.StringBuilder(255);
      footsteps_ = new us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage> (50, new behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessagePubSubType());
      goal_stance_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      goal_focal_point_ = new us.ihmc.euclid.tuple3D.Point3D();
      planner_parameters_ = new ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage();

   }

   public FootstepPlanActionDefinitionMessage(FootstepPlanActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

      execution_mode_ = other.execution_mode_;

      is_manually_placed_ = other.is_manually_placed_;

      footsteps_.set(other.footsteps_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_stance_point_, goal_stance_point_);
      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.goal_focal_point_, goal_focal_point_);
      left_goal_foot_x_to_gizmo_ = other.left_goal_foot_x_to_gizmo_;

      left_goal_foot_y_to_gizmo_ = other.left_goal_foot_y_to_gizmo_;

      left_goal_foot_yaw_to_gizmo_ = other.left_goal_foot_yaw_to_gizmo_;

      right_goal_foot_x_to_gizmo_ = other.right_goal_foot_x_to_gizmo_;

      right_goal_foot_y_to_gizmo_ = other.right_goal_foot_y_to_gizmo_;

      right_goal_foot_yaw_to_gizmo_ = other.right_goal_foot_yaw_to_gizmo_;

      planner_initial_stance_side_ = other.planner_initial_stance_side_;

      planner_ = other.planner_;

      planner_walk_with_goal_orientation_ = other.planner_walk_with_goal_orientation_;

      planner_plan_with_body_path_ = other.planner_plan_with_body_path_;

      ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessagePubSubType.staticCopy(other.planner_parameters_, planner_parameters_);
      quick_hip_width_ = other.quick_hip_width_;

      quick_step_length_ = other.quick_step_length_;

      quick_next_pelvis_yaw_limit_ = other.quick_next_pelvis_yaw_limit_;

      quick_inward_limit_ = other.quick_inward_limit_;

      quick_outward_limit_ = other.quick_outward_limit_;

      quick_step_angle_limit_ = other.quick_step_angle_limit_;

      quick_swing_time_distance_lower_ = other.quick_swing_time_distance_lower_;

      quick_swing_time_distance_upper_ = other.quick_swing_time_distance_upper_;

      quick_min_swing_time_ = other.quick_min_swing_time_;

      quick_max_swing_time_ = other.quick_max_swing_time_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public void setParentFrameName(java.lang.String parent_frame_name)
   {
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(parent_frame_name);
   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.String getParentFrameNameAsString()
   {
      return getParentFrameName().toString();
   }
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder getParentFrameName()
   {
      return parent_frame_name_;
   }

   /**
            * Swing duration
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * Swing duration
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   /**
            * Transfer duration
            */
   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   /**
            * Transfer duration
            */
   public double getTransferDuration()
   {
      return transfer_duration_;
   }

   /**
            * OVERRIDE (0) or QUEUE (1)
            */
   public void setExecutionMode(byte execution_mode)
   {
      execution_mode_ = execution_mode;
   }
   /**
            * OVERRIDE (0) or QUEUE (1)
            */
   public byte getExecutionMode()
   {
      return execution_mode_;
   }

   /**
            * Is using the footstep planner or manually placing
            */
   public void setIsManuallyPlaced(boolean is_manually_placed)
   {
      is_manually_placed_ = is_manually_placed;
   }
   /**
            * Is using the footstep planner or manually placing
            */
   public boolean getIsManuallyPlaced()
   {
      return is_manually_placed_;
   }


   /**
            * The footsteps, with a maximum of 50
            */
   public us.ihmc.idl.IDLSequence.Object<behavior_msgs.msg.dds.FootstepPlanActionFootstepDefinitionMessage>  getFootsteps()
   {
      return footsteps_;
   }


   /**
            * The position to which the goal stance is aligned
            */
   public us.ihmc.euclid.tuple3D.Point3D getGoalStancePoint()
   {
      return goal_stance_point_;
   }


   /**
            * The point that the robot should be facing in the goal stance
            */
   public us.ihmc.euclid.tuple3D.Point3D getGoalFocalPoint()
   {
      return goal_focal_point_;
   }

   /**
            * Left goal foot X to the goal pose
            */
   public void setLeftGoalFootXToGizmo(double left_goal_foot_x_to_gizmo)
   {
      left_goal_foot_x_to_gizmo_ = left_goal_foot_x_to_gizmo;
   }
   /**
            * Left goal foot X to the goal pose
            */
   public double getLeftGoalFootXToGizmo()
   {
      return left_goal_foot_x_to_gizmo_;
   }

   /**
            * Left goal foot Y to the goal pose
            */
   public void setLeftGoalFootYToGizmo(double left_goal_foot_y_to_gizmo)
   {
      left_goal_foot_y_to_gizmo_ = left_goal_foot_y_to_gizmo;
   }
   /**
            * Left goal foot Y to the goal pose
            */
   public double getLeftGoalFootYToGizmo()
   {
      return left_goal_foot_y_to_gizmo_;
   }

   /**
            * Left goal foot Yaw to the goal pose
            */
   public void setLeftGoalFootYawToGizmo(double left_goal_foot_yaw_to_gizmo)
   {
      left_goal_foot_yaw_to_gizmo_ = left_goal_foot_yaw_to_gizmo;
   }
   /**
            * Left goal foot Yaw to the goal pose
            */
   public double getLeftGoalFootYawToGizmo()
   {
      return left_goal_foot_yaw_to_gizmo_;
   }

   /**
            * Right goal foot X to the goal pose
            */
   public void setRightGoalFootXToGizmo(double right_goal_foot_x_to_gizmo)
   {
      right_goal_foot_x_to_gizmo_ = right_goal_foot_x_to_gizmo;
   }
   /**
            * Right goal foot X to the goal pose
            */
   public double getRightGoalFootXToGizmo()
   {
      return right_goal_foot_x_to_gizmo_;
   }

   /**
            * Right goal foot Y to the goal pose
            */
   public void setRightGoalFootYToGizmo(double right_goal_foot_y_to_gizmo)
   {
      right_goal_foot_y_to_gizmo_ = right_goal_foot_y_to_gizmo;
   }
   /**
            * Right goal foot Y to the goal pose
            */
   public double getRightGoalFootYToGizmo()
   {
      return right_goal_foot_y_to_gizmo_;
   }

   /**
            * Right goal foot Yaw to the goal pose
            */
   public void setRightGoalFootYawToGizmo(double right_goal_foot_yaw_to_gizmo)
   {
      right_goal_foot_yaw_to_gizmo_ = right_goal_foot_yaw_to_gizmo;
   }
   /**
            * Right goal foot Yaw to the goal pose
            */
   public double getRightGoalFootYawToGizmo()
   {
      return right_goal_foot_yaw_to_gizmo_;
   }

   /**
            * Initial stance side when planning
            */
   public void setPlannerInitialStanceSide(byte planner_initial_stance_side)
   {
      planner_initial_stance_side_ = planner_initial_stance_side;
   }
   /**
            * Initial stance side when planning
            */
   public byte getPlannerInitialStanceSide()
   {
      return planner_initial_stance_side_;
   }

   /**
            * The type of planner as defined above
            */
   public void setPlanner(byte planner)
   {
      planner_ = planner;
   }
   /**
            * The type of planner as defined above
            */
   public byte getPlanner()
   {
      return planner_;
   }

   /**
            * Whether to walk, turning first, matching the goal orientation
            */
   public void setPlannerWalkWithGoalOrientation(boolean planner_walk_with_goal_orientation)
   {
      planner_walk_with_goal_orientation_ = planner_walk_with_goal_orientation;
   }
   /**
            * Whether to walk, turning first, matching the goal orientation
            */
   public boolean getPlannerWalkWithGoalOrientation()
   {
      return planner_walk_with_goal_orientation_;
   }

   /**
            * Whether to use the body path planner or not
            */
   public void setPlannerPlanWithBodyPath(boolean planner_plan_with_body_path)
   {
      planner_plan_with_body_path_ = planner_plan_with_body_path;
   }
   /**
            * Whether to use the body path planner or not
            */
   public boolean getPlannerPlanWithBodyPath()
   {
      return planner_plan_with_body_path_;
   }


   /**
            * The footstep planner parameters
            */
   public ihmc_common_msgs.msg.dds.PrimitiveDataVectorMessage getPlannerParameters()
   {
      return planner_parameters_;
   }

   /**
            * Quick footstep planner parameters
            */
   public void setQuickHipWidth(double quick_hip_width)
   {
      quick_hip_width_ = quick_hip_width;
   }
   /**
            * Quick footstep planner parameters
            */
   public double getQuickHipWidth()
   {
      return quick_hip_width_;
   }

   public void setQuickStepLength(double quick_step_length)
   {
      quick_step_length_ = quick_step_length;
   }
   public double getQuickStepLength()
   {
      return quick_step_length_;
   }

   public void setQuickNextPelvisYawLimit(double quick_next_pelvis_yaw_limit)
   {
      quick_next_pelvis_yaw_limit_ = quick_next_pelvis_yaw_limit;
   }
   public double getQuickNextPelvisYawLimit()
   {
      return quick_next_pelvis_yaw_limit_;
   }

   public void setQuickInwardLimit(double quick_inward_limit)
   {
      quick_inward_limit_ = quick_inward_limit;
   }
   public double getQuickInwardLimit()
   {
      return quick_inward_limit_;
   }

   public void setQuickOutwardLimit(double quick_outward_limit)
   {
      quick_outward_limit_ = quick_outward_limit;
   }
   public double getQuickOutwardLimit()
   {
      return quick_outward_limit_;
   }

   public void setQuickStepAngleLimit(double quick_step_angle_limit)
   {
      quick_step_angle_limit_ = quick_step_angle_limit;
   }
   public double getQuickStepAngleLimit()
   {
      return quick_step_angle_limit_;
   }

   public void setQuickSwingTimeDistanceLower(double quick_swing_time_distance_lower)
   {
      quick_swing_time_distance_lower_ = quick_swing_time_distance_lower;
   }
   public double getQuickSwingTimeDistanceLower()
   {
      return quick_swing_time_distance_lower_;
   }

   public void setQuickSwingTimeDistanceUpper(double quick_swing_time_distance_upper)
   {
      quick_swing_time_distance_upper_ = quick_swing_time_distance_upper;
   }
   public double getQuickSwingTimeDistanceUpper()
   {
      return quick_swing_time_distance_upper_;
   }

   public void setQuickMinSwingTime(double quick_min_swing_time)
   {
      quick_min_swing_time_ = quick_min_swing_time;
   }
   public double getQuickMinSwingTime()
   {
      return quick_min_swing_time_;
   }

   public void setQuickMaxSwingTime(double quick_max_swing_time)
   {
      quick_max_swing_time_ = quick_max_swing_time;
   }
   public double getQuickMaxSwingTime()
   {
      return quick_max_swing_time_;
   }


   public static Supplier<FootstepPlanActionDefinitionMessagePubSubType> getPubSubType()
   {
      return FootstepPlanActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_mode_, other.execution_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_manually_placed_, other.is_manually_placed_, epsilon)) return false;

      if (this.footsteps_.size() != other.footsteps_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footsteps_.size(); i++)
         {  if (!this.footsteps_.get(i).epsilonEquals(other.footsteps_.get(i), epsilon)) return false; }
      }

      if (!this.goal_stance_point_.epsilonEquals(other.goal_stance_point_, epsilon)) return false;
      if (!this.goal_focal_point_.epsilonEquals(other.goal_focal_point_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_goal_foot_x_to_gizmo_, other.left_goal_foot_x_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_goal_foot_y_to_gizmo_, other.left_goal_foot_y_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.left_goal_foot_yaw_to_gizmo_, other.left_goal_foot_yaw_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_goal_foot_x_to_gizmo_, other.right_goal_foot_x_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_goal_foot_y_to_gizmo_, other.right_goal_foot_y_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.right_goal_foot_yaw_to_gizmo_, other.right_goal_foot_yaw_to_gizmo_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_initial_stance_side_, other.planner_initial_stance_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_, other.planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.planner_walk_with_goal_orientation_, other.planner_walk_with_goal_orientation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.planner_plan_with_body_path_, other.planner_plan_with_body_path_, epsilon)) return false;

      if (!this.planner_parameters_.epsilonEquals(other.planner_parameters_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_hip_width_, other.quick_hip_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_step_length_, other.quick_step_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_next_pelvis_yaw_limit_, other.quick_next_pelvis_yaw_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_inward_limit_, other.quick_inward_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_outward_limit_, other.quick_outward_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_step_angle_limit_, other.quick_step_angle_limit_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_swing_time_distance_lower_, other.quick_swing_time_distance_lower_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_swing_time_distance_upper_, other.quick_swing_time_distance_upper_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_min_swing_time_, other.quick_min_swing_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.quick_max_swing_time_, other.quick_max_swing_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanActionDefinitionMessage)) return false;

      FootstepPlanActionDefinitionMessage otherMyClass = (FootstepPlanActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;

      if(this.execution_mode_ != otherMyClass.execution_mode_) return false;

      if(this.is_manually_placed_ != otherMyClass.is_manually_placed_) return false;

      if (!this.footsteps_.equals(otherMyClass.footsteps_)) return false;
      if (!this.goal_stance_point_.equals(otherMyClass.goal_stance_point_)) return false;
      if (!this.goal_focal_point_.equals(otherMyClass.goal_focal_point_)) return false;
      if(this.left_goal_foot_x_to_gizmo_ != otherMyClass.left_goal_foot_x_to_gizmo_) return false;

      if(this.left_goal_foot_y_to_gizmo_ != otherMyClass.left_goal_foot_y_to_gizmo_) return false;

      if(this.left_goal_foot_yaw_to_gizmo_ != otherMyClass.left_goal_foot_yaw_to_gizmo_) return false;

      if(this.right_goal_foot_x_to_gizmo_ != otherMyClass.right_goal_foot_x_to_gizmo_) return false;

      if(this.right_goal_foot_y_to_gizmo_ != otherMyClass.right_goal_foot_y_to_gizmo_) return false;

      if(this.right_goal_foot_yaw_to_gizmo_ != otherMyClass.right_goal_foot_yaw_to_gizmo_) return false;

      if(this.planner_initial_stance_side_ != otherMyClass.planner_initial_stance_side_) return false;

      if(this.planner_ != otherMyClass.planner_) return false;

      if(this.planner_walk_with_goal_orientation_ != otherMyClass.planner_walk_with_goal_orientation_) return false;

      if(this.planner_plan_with_body_path_ != otherMyClass.planner_plan_with_body_path_) return false;

      if (!this.planner_parameters_.equals(otherMyClass.planner_parameters_)) return false;
      if(this.quick_hip_width_ != otherMyClass.quick_hip_width_) return false;

      if(this.quick_step_length_ != otherMyClass.quick_step_length_) return false;

      if(this.quick_next_pelvis_yaw_limit_ != otherMyClass.quick_next_pelvis_yaw_limit_) return false;

      if(this.quick_inward_limit_ != otherMyClass.quick_inward_limit_) return false;

      if(this.quick_outward_limit_ != otherMyClass.quick_outward_limit_) return false;

      if(this.quick_step_angle_limit_ != otherMyClass.quick_step_angle_limit_) return false;

      if(this.quick_swing_time_distance_lower_ != otherMyClass.quick_swing_time_distance_lower_) return false;

      if(this.quick_swing_time_distance_upper_ != otherMyClass.quick_swing_time_distance_upper_) return false;

      if(this.quick_min_swing_time_ != otherMyClass.quick_min_swing_time_) return false;

      if(this.quick_max_swing_time_ != otherMyClass.quick_max_swing_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);      builder.append(", ");
      builder.append("execution_mode=");
      builder.append(this.execution_mode_);      builder.append(", ");
      builder.append("is_manually_placed=");
      builder.append(this.is_manually_placed_);      builder.append(", ");
      builder.append("footsteps=");
      builder.append(this.footsteps_);      builder.append(", ");
      builder.append("goal_stance_point=");
      builder.append(this.goal_stance_point_);      builder.append(", ");
      builder.append("goal_focal_point=");
      builder.append(this.goal_focal_point_);      builder.append(", ");
      builder.append("left_goal_foot_x_to_gizmo=");
      builder.append(this.left_goal_foot_x_to_gizmo_);      builder.append(", ");
      builder.append("left_goal_foot_y_to_gizmo=");
      builder.append(this.left_goal_foot_y_to_gizmo_);      builder.append(", ");
      builder.append("left_goal_foot_yaw_to_gizmo=");
      builder.append(this.left_goal_foot_yaw_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_x_to_gizmo=");
      builder.append(this.right_goal_foot_x_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_y_to_gizmo=");
      builder.append(this.right_goal_foot_y_to_gizmo_);      builder.append(", ");
      builder.append("right_goal_foot_yaw_to_gizmo=");
      builder.append(this.right_goal_foot_yaw_to_gizmo_);      builder.append(", ");
      builder.append("planner_initial_stance_side=");
      builder.append(this.planner_initial_stance_side_);      builder.append(", ");
      builder.append("planner=");
      builder.append(this.planner_);      builder.append(", ");
      builder.append("planner_walk_with_goal_orientation=");
      builder.append(this.planner_walk_with_goal_orientation_);      builder.append(", ");
      builder.append("planner_plan_with_body_path=");
      builder.append(this.planner_plan_with_body_path_);      builder.append(", ");
      builder.append("planner_parameters=");
      builder.append(this.planner_parameters_);      builder.append(", ");
      builder.append("quick_hip_width=");
      builder.append(this.quick_hip_width_);      builder.append(", ");
      builder.append("quick_step_length=");
      builder.append(this.quick_step_length_);      builder.append(", ");
      builder.append("quick_next_pelvis_yaw_limit=");
      builder.append(this.quick_next_pelvis_yaw_limit_);      builder.append(", ");
      builder.append("quick_inward_limit=");
      builder.append(this.quick_inward_limit_);      builder.append(", ");
      builder.append("quick_outward_limit=");
      builder.append(this.quick_outward_limit_);      builder.append(", ");
      builder.append("quick_step_angle_limit=");
      builder.append(this.quick_step_angle_limit_);      builder.append(", ");
      builder.append("quick_swing_time_distance_lower=");
      builder.append(this.quick_swing_time_distance_lower_);      builder.append(", ");
      builder.append("quick_swing_time_distance_upper=");
      builder.append(this.quick_swing_time_distance_upper_);      builder.append(", ");
      builder.append("quick_min_swing_time=");
      builder.append(this.quick_min_swing_time_);      builder.append(", ");
      builder.append("quick_max_swing_time=");
      builder.append(this.quick_max_swing_time_);
      builder.append("}");
      return builder.toString();
   }
}
