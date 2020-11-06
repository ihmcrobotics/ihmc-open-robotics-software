package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPlanningRequestPacket extends Packet<FootstepPlanningRequestPacket> implements Settable<FootstepPlanningRequestPacket>, EpsilonComparable<FootstepPlanningRequestPacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public static final byte SWING_PLANNER_TYPE_NONE = (byte) 0;
   public static final byte SWING_PLANNER_TYPE_POSITION = (byte) 1;
   public static final byte SWING_PLANNER_TYPE_PROPORTION = (byte) 2;
   public static final int NO_PLAN_ID = -1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Starting left foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D start_left_foot_pose_;
   /**
            * Starting right foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D start_right_foot_pose_;
   /**
            * Starting left foothold. This field is optional and only should be populated when the initial stance foot has a partial foothold
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  initial_left_contact_points_2d_;
   /**
            * Starting right foothold. This field is optional and only should be populated when the initial stance foot has a partial foothold
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  initial_right_contact_points_2d_;
   /**
            * Goal left foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D goal_left_foot_pose_;
   /**
            * Goal right foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D goal_right_foot_pose_;
   /**
            * Requested initial stance side. If not specified the planner will choose
            */
   public byte requested_initial_stance_side_ = (byte) 255;
   /**
            * If true, the planner will snap the provided goal steps. Otherwise the provided poses will be trusted as valid footholds.
            */
   public boolean snap_goal_steps_ = true;
   /**
            * If snap_goal_steps is true and the goal steps can't be snapped, this specifies whether to abort or go ahead and plan.
            */
   public boolean abort_if_goal_step_snapping_fails_;
   /**
            * If plan_body_path is true and the planner fails, this specifies whether to abort or use a straight-line body path
            */
   public boolean abort_if_body_path_planner_fails_;
   /**
            * If true, will plan a body path. If false, will follow a straight-line path to the goal
            */
   public boolean plan_body_path_;
   /**
            * If true, does A* search. If false, a simple turn-walk-turn path is returned with no checks on step feasibility.
            */
   public boolean perform_a_star_search_ = true;
   /**
            * Requested body path waypoints. If non-empty, planner will follow this path and will not plan a body path
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  body_path_waypoints_;
   /**
            * (In beta) acceptable xy distance from the given goal for the planner to terminate
            */
   public double goal_distance_proximity_ = -1.0;
   /**
            * (In beta) acceptable yaw offset from the given goal for the planner to terminate
            */
   public double goal_yaw_proximity_ = -1.0;
   /**
            * Specifies the desired robot heading. Zero (default) is facing forward, pi is walking backwards, positive angles is facing left (right foot leads).
            * The planner generates turn-walk-turn plans and this describes the robot's orientation during the walk portion.
            */
   public double requested_path_heading_;
   /**
            * Planner timeout in seconds. If max_iterations is set also, the planner terminates whenever either is reached
            */
   public double timeout_ = 5.0;
   /**
            * Maximum iterations. Set to a non-positive number to disable. If timeout is also set, the planner terminates whener either is reached.
            */
   public int max_iterations_ = -1;
   /**
            * Max body path length if using body path
            */
   public double horizon_length_;
   /**
            * Planar regions to use, if you don't want to assume flat ground
            */
   public controller_msgs.msg.dds.PlanarRegionsListMessage planar_regions_list_message_;
   /**
            * Explicitly tell the planner to use flat ground
            */
   public boolean assume_flat_ground_;
   /**
            * Set this id to keep track of your request
            */
   public int planner_request_id_ = -1;
   /**
            * Period of time in seconds the planner will publish it's status. If this is a non-positive number no status is published until it's completed.
            */
   public double status_publish_period_ = 1.0;
   /**
            * Specifies swing planner to use. See the above enumeration
            */
   public byte requested_swing_planner_;
   /**
            * Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
            * For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
            */
   public boolean generate_log_;

   public FootstepPlanningRequestPacket()
   {
      start_left_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      start_right_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      initial_left_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());
      initial_right_contact_points_2d_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (10, new geometry_msgs.msg.dds.PointPubSubType());
      goal_left_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      goal_right_foot_pose_ = new us.ihmc.euclid.geometry.Pose3D();
      body_path_waypoints_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D> (50, new geometry_msgs.msg.dds.PosePubSubType());
      planar_regions_list_message_ = new controller_msgs.msg.dds.PlanarRegionsListMessage();

   }

   public FootstepPlanningRequestPacket(FootstepPlanningRequestPacket other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlanningRequestPacket other)
   {
      sequence_id_ = other.sequence_id_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.start_left_foot_pose_, start_left_foot_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.start_right_foot_pose_, start_right_foot_pose_);
      initial_left_contact_points_2d_.set(other.initial_left_contact_points_2d_);
      initial_right_contact_points_2d_.set(other.initial_right_contact_points_2d_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.goal_left_foot_pose_, goal_left_foot_pose_);
      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.goal_right_foot_pose_, goal_right_foot_pose_);
      requested_initial_stance_side_ = other.requested_initial_stance_side_;

      snap_goal_steps_ = other.snap_goal_steps_;

      abort_if_goal_step_snapping_fails_ = other.abort_if_goal_step_snapping_fails_;

      abort_if_body_path_planner_fails_ = other.abort_if_body_path_planner_fails_;

      plan_body_path_ = other.plan_body_path_;

      perform_a_star_search_ = other.perform_a_star_search_;

      body_path_waypoints_.set(other.body_path_waypoints_);
      goal_distance_proximity_ = other.goal_distance_proximity_;

      goal_yaw_proximity_ = other.goal_yaw_proximity_;

      requested_path_heading_ = other.requested_path_heading_;

      timeout_ = other.timeout_;

      max_iterations_ = other.max_iterations_;

      horizon_length_ = other.horizon_length_;

      controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType.staticCopy(other.planar_regions_list_message_, planar_regions_list_message_);
      assume_flat_ground_ = other.assume_flat_ground_;

      planner_request_id_ = other.planner_request_id_;

      status_publish_period_ = other.status_publish_period_;

      requested_swing_planner_ = other.requested_swing_planner_;

      generate_log_ = other.generate_log_;

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


   /**
            * Starting left foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D getStartLeftFootPose()
   {
      return start_left_foot_pose_;
   }


   /**
            * Starting right foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D getStartRightFootPose()
   {
      return start_right_foot_pose_;
   }


   /**
            * Starting left foothold. This field is optional and only should be populated when the initial stance foot has a partial foothold
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getInitialLeftContactPoints2d()
   {
      return initial_left_contact_points_2d_;
   }


   /**
            * Starting right foothold. This field is optional and only should be populated when the initial stance foot has a partial foothold
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getInitialRightContactPoints2d()
   {
      return initial_right_contact_points_2d_;
   }


   /**
            * Goal left foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D getGoalLeftFootPose()
   {
      return goal_left_foot_pose_;
   }


   /**
            * Goal right foot pose
            */
   public us.ihmc.euclid.geometry.Pose3D getGoalRightFootPose()
   {
      return goal_right_foot_pose_;
   }

   /**
            * Requested initial stance side. If not specified the planner will choose
            */
   public void setRequestedInitialStanceSide(byte requested_initial_stance_side)
   {
      requested_initial_stance_side_ = requested_initial_stance_side;
   }
   /**
            * Requested initial stance side. If not specified the planner will choose
            */
   public byte getRequestedInitialStanceSide()
   {
      return requested_initial_stance_side_;
   }

   /**
            * If true, the planner will snap the provided goal steps. Otherwise the provided poses will be trusted as valid footholds.
            */
   public void setSnapGoalSteps(boolean snap_goal_steps)
   {
      snap_goal_steps_ = snap_goal_steps;
   }
   /**
            * If true, the planner will snap the provided goal steps. Otherwise the provided poses will be trusted as valid footholds.
            */
   public boolean getSnapGoalSteps()
   {
      return snap_goal_steps_;
   }

   /**
            * If snap_goal_steps is true and the goal steps can't be snapped, this specifies whether to abort or go ahead and plan.
            */
   public void setAbortIfGoalStepSnappingFails(boolean abort_if_goal_step_snapping_fails)
   {
      abort_if_goal_step_snapping_fails_ = abort_if_goal_step_snapping_fails;
   }
   /**
            * If snap_goal_steps is true and the goal steps can't be snapped, this specifies whether to abort or go ahead and plan.
            */
   public boolean getAbortIfGoalStepSnappingFails()
   {
      return abort_if_goal_step_snapping_fails_;
   }

   /**
            * If plan_body_path is true and the planner fails, this specifies whether to abort or use a straight-line body path
            */
   public void setAbortIfBodyPathPlannerFails(boolean abort_if_body_path_planner_fails)
   {
      abort_if_body_path_planner_fails_ = abort_if_body_path_planner_fails;
   }
   /**
            * If plan_body_path is true and the planner fails, this specifies whether to abort or use a straight-line body path
            */
   public boolean getAbortIfBodyPathPlannerFails()
   {
      return abort_if_body_path_planner_fails_;
   }

   /**
            * If true, will plan a body path. If false, will follow a straight-line path to the goal
            */
   public void setPlanBodyPath(boolean plan_body_path)
   {
      plan_body_path_ = plan_body_path;
   }
   /**
            * If true, will plan a body path. If false, will follow a straight-line path to the goal
            */
   public boolean getPlanBodyPath()
   {
      return plan_body_path_;
   }

   /**
            * If true, does A* search. If false, a simple turn-walk-turn path is returned with no checks on step feasibility.
            */
   public void setPerformAStarSearch(boolean perform_a_star_search)
   {
      perform_a_star_search_ = perform_a_star_search;
   }
   /**
            * If true, does A* search. If false, a simple turn-walk-turn path is returned with no checks on step feasibility.
            */
   public boolean getPerformAStarSearch()
   {
      return perform_a_star_search_;
   }


   /**
            * Requested body path waypoints. If non-empty, planner will follow this path and will not plan a body path
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.geometry.Pose3D>  getBodyPathWaypoints()
   {
      return body_path_waypoints_;
   }

   /**
            * (In beta) acceptable xy distance from the given goal for the planner to terminate
            */
   public void setGoalDistanceProximity(double goal_distance_proximity)
   {
      goal_distance_proximity_ = goal_distance_proximity;
   }
   /**
            * (In beta) acceptable xy distance from the given goal for the planner to terminate
            */
   public double getGoalDistanceProximity()
   {
      return goal_distance_proximity_;
   }

   /**
            * (In beta) acceptable yaw offset from the given goal for the planner to terminate
            */
   public void setGoalYawProximity(double goal_yaw_proximity)
   {
      goal_yaw_proximity_ = goal_yaw_proximity;
   }
   /**
            * (In beta) acceptable yaw offset from the given goal for the planner to terminate
            */
   public double getGoalYawProximity()
   {
      return goal_yaw_proximity_;
   }

   /**
            * Specifies the desired robot heading. Zero (default) is facing forward, pi is walking backwards, positive angles is facing left (right foot leads).
            * The planner generates turn-walk-turn plans and this describes the robot's orientation during the walk portion.
            */
   public void setRequestedPathHeading(double requested_path_heading)
   {
      requested_path_heading_ = requested_path_heading;
   }
   /**
            * Specifies the desired robot heading. Zero (default) is facing forward, pi is walking backwards, positive angles is facing left (right foot leads).
            * The planner generates turn-walk-turn plans and this describes the robot's orientation during the walk portion.
            */
   public double getRequestedPathHeading()
   {
      return requested_path_heading_;
   }

   /**
            * Planner timeout in seconds. If max_iterations is set also, the planner terminates whenever either is reached
            */
   public void setTimeout(double timeout)
   {
      timeout_ = timeout;
   }
   /**
            * Planner timeout in seconds. If max_iterations is set also, the planner terminates whenever either is reached
            */
   public double getTimeout()
   {
      return timeout_;
   }

   /**
            * Maximum iterations. Set to a non-positive number to disable. If timeout is also set, the planner terminates whener either is reached.
            */
   public void setMaxIterations(int max_iterations)
   {
      max_iterations_ = max_iterations;
   }
   /**
            * Maximum iterations. Set to a non-positive number to disable. If timeout is also set, the planner terminates whener either is reached.
            */
   public int getMaxIterations()
   {
      return max_iterations_;
   }

   /**
            * Max body path length if using body path
            */
   public void setHorizonLength(double horizon_length)
   {
      horizon_length_ = horizon_length;
   }
   /**
            * Max body path length if using body path
            */
   public double getHorizonLength()
   {
      return horizon_length_;
   }


   /**
            * Planar regions to use, if you don't want to assume flat ground
            */
   public controller_msgs.msg.dds.PlanarRegionsListMessage getPlanarRegionsListMessage()
   {
      return planar_regions_list_message_;
   }

   /**
            * Explicitly tell the planner to use flat ground
            */
   public void setAssumeFlatGround(boolean assume_flat_ground)
   {
      assume_flat_ground_ = assume_flat_ground;
   }
   /**
            * Explicitly tell the planner to use flat ground
            */
   public boolean getAssumeFlatGround()
   {
      return assume_flat_ground_;
   }

   /**
            * Set this id to keep track of your request
            */
   public void setPlannerRequestId(int planner_request_id)
   {
      planner_request_id_ = planner_request_id;
   }
   /**
            * Set this id to keep track of your request
            */
   public int getPlannerRequestId()
   {
      return planner_request_id_;
   }

   /**
            * Period of time in seconds the planner will publish it's status. If this is a non-positive number no status is published until it's completed.
            */
   public void setStatusPublishPeriod(double status_publish_period)
   {
      status_publish_period_ = status_publish_period;
   }
   /**
            * Period of time in seconds the planner will publish it's status. If this is a non-positive number no status is published until it's completed.
            */
   public double getStatusPublishPeriod()
   {
      return status_publish_period_;
   }

   /**
            * Specifies swing planner to use. See the above enumeration
            */
   public void setRequestedSwingPlanner(byte requested_swing_planner)
   {
      requested_swing_planner_ = requested_swing_planner;
   }
   /**
            * Specifies swing planner to use. See the above enumeration
            */
   public byte getRequestedSwingPlanner()
   {
      return requested_swing_planner_;
   }

   /**
            * Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
            * For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
            */
   public void setGenerateLog(boolean generate_log)
   {
      generate_log_ = generate_log;
   }
   /**
            * Generate log of this plan. Logs are written to ~/.ihmc/logs by default, set the environment variable IHMC_FOOTSTEP_PLANNER_LOG_DIR to override this directory.
            * For example, export IHMC_FOOTSTEP_PLANNER_LOG_DIR=/home/user/myLogs/
            */
   public boolean getGenerateLog()
   {
      return generate_log_;
   }


   public static Supplier<FootstepPlanningRequestPacketPubSubType> getPubSubType()
   {
      return FootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlanningRequestPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningRequestPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!this.start_left_foot_pose_.epsilonEquals(other.start_left_foot_pose_, epsilon)) return false;
      if (!this.start_right_foot_pose_.epsilonEquals(other.start_right_foot_pose_, epsilon)) return false;
      if (this.initial_left_contact_points_2d_.size() != other.initial_left_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.initial_left_contact_points_2d_.size(); i++)
         {  if (!this.initial_left_contact_points_2d_.get(i).epsilonEquals(other.initial_left_contact_points_2d_.get(i), epsilon)) return false; }
      }

      if (this.initial_right_contact_points_2d_.size() != other.initial_right_contact_points_2d_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.initial_right_contact_points_2d_.size(); i++)
         {  if (!this.initial_right_contact_points_2d_.get(i).epsilonEquals(other.initial_right_contact_points_2d_.get(i), epsilon)) return false; }
      }

      if (!this.goal_left_foot_pose_.epsilonEquals(other.goal_left_foot_pose_, epsilon)) return false;
      if (!this.goal_right_foot_pose_.epsilonEquals(other.goal_right_foot_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_initial_stance_side_, other.requested_initial_stance_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.snap_goal_steps_, other.snap_goal_steps_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.abort_if_goal_step_snapping_fails_, other.abort_if_goal_step_snapping_fails_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.abort_if_body_path_planner_fails_, other.abort_if_body_path_planner_fails_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.plan_body_path_, other.plan_body_path_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.perform_a_star_search_, other.perform_a_star_search_, epsilon)) return false;

      if (this.body_path_waypoints_.size() != other.body_path_waypoints_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.body_path_waypoints_.size(); i++)
         {  if (!this.body_path_waypoints_.get(i).epsilonEquals(other.body_path_waypoints_.get(i), epsilon)) return false; }
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_distance_proximity_, other.goal_distance_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_yaw_proximity_, other.goal_yaw_proximity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_path_heading_, other.requested_path_heading_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_iterations_, other.max_iterations_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.horizon_length_, other.horizon_length_, epsilon)) return false;

      if (!this.planar_regions_list_message_.epsilonEquals(other.planar_regions_list_message_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.assume_flat_ground_, other.assume_flat_ground_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planner_request_id_, other.planner_request_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.status_publish_period_, other.status_publish_period_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.requested_swing_planner_, other.requested_swing_planner_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.generate_log_, other.generate_log_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlanningRequestPacket)) return false;

      FootstepPlanningRequestPacket otherMyClass = (FootstepPlanningRequestPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if (!this.start_left_foot_pose_.equals(otherMyClass.start_left_foot_pose_)) return false;
      if (!this.start_right_foot_pose_.equals(otherMyClass.start_right_foot_pose_)) return false;
      if (!this.initial_left_contact_points_2d_.equals(otherMyClass.initial_left_contact_points_2d_)) return false;
      if (!this.initial_right_contact_points_2d_.equals(otherMyClass.initial_right_contact_points_2d_)) return false;
      if (!this.goal_left_foot_pose_.equals(otherMyClass.goal_left_foot_pose_)) return false;
      if (!this.goal_right_foot_pose_.equals(otherMyClass.goal_right_foot_pose_)) return false;
      if(this.requested_initial_stance_side_ != otherMyClass.requested_initial_stance_side_) return false;

      if(this.snap_goal_steps_ != otherMyClass.snap_goal_steps_) return false;

      if(this.abort_if_goal_step_snapping_fails_ != otherMyClass.abort_if_goal_step_snapping_fails_) return false;

      if(this.abort_if_body_path_planner_fails_ != otherMyClass.abort_if_body_path_planner_fails_) return false;

      if(this.plan_body_path_ != otherMyClass.plan_body_path_) return false;

      if(this.perform_a_star_search_ != otherMyClass.perform_a_star_search_) return false;

      if (!this.body_path_waypoints_.equals(otherMyClass.body_path_waypoints_)) return false;
      if(this.goal_distance_proximity_ != otherMyClass.goal_distance_proximity_) return false;

      if(this.goal_yaw_proximity_ != otherMyClass.goal_yaw_proximity_) return false;

      if(this.requested_path_heading_ != otherMyClass.requested_path_heading_) return false;

      if(this.timeout_ != otherMyClass.timeout_) return false;

      if(this.max_iterations_ != otherMyClass.max_iterations_) return false;

      if(this.horizon_length_ != otherMyClass.horizon_length_) return false;

      if (!this.planar_regions_list_message_.equals(otherMyClass.planar_regions_list_message_)) return false;
      if(this.assume_flat_ground_ != otherMyClass.assume_flat_ground_) return false;

      if(this.planner_request_id_ != otherMyClass.planner_request_id_) return false;

      if(this.status_publish_period_ != otherMyClass.status_publish_period_) return false;

      if(this.requested_swing_planner_ != otherMyClass.requested_swing_planner_) return false;

      if(this.generate_log_ != otherMyClass.generate_log_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlanningRequestPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("start_left_foot_pose=");
      builder.append(this.start_left_foot_pose_);      builder.append(", ");
      builder.append("start_right_foot_pose=");
      builder.append(this.start_right_foot_pose_);      builder.append(", ");
      builder.append("initial_left_contact_points_2d=");
      builder.append(this.initial_left_contact_points_2d_);      builder.append(", ");
      builder.append("initial_right_contact_points_2d=");
      builder.append(this.initial_right_contact_points_2d_);      builder.append(", ");
      builder.append("goal_left_foot_pose=");
      builder.append(this.goal_left_foot_pose_);      builder.append(", ");
      builder.append("goal_right_foot_pose=");
      builder.append(this.goal_right_foot_pose_);      builder.append(", ");
      builder.append("requested_initial_stance_side=");
      builder.append(this.requested_initial_stance_side_);      builder.append(", ");
      builder.append("snap_goal_steps=");
      builder.append(this.snap_goal_steps_);      builder.append(", ");
      builder.append("abort_if_goal_step_snapping_fails=");
      builder.append(this.abort_if_goal_step_snapping_fails_);      builder.append(", ");
      builder.append("abort_if_body_path_planner_fails=");
      builder.append(this.abort_if_body_path_planner_fails_);      builder.append(", ");
      builder.append("plan_body_path=");
      builder.append(this.plan_body_path_);      builder.append(", ");
      builder.append("perform_a_star_search=");
      builder.append(this.perform_a_star_search_);      builder.append(", ");
      builder.append("body_path_waypoints=");
      builder.append(this.body_path_waypoints_);      builder.append(", ");
      builder.append("goal_distance_proximity=");
      builder.append(this.goal_distance_proximity_);      builder.append(", ");
      builder.append("goal_yaw_proximity=");
      builder.append(this.goal_yaw_proximity_);      builder.append(", ");
      builder.append("requested_path_heading=");
      builder.append(this.requested_path_heading_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");
      builder.append("max_iterations=");
      builder.append(this.max_iterations_);      builder.append(", ");
      builder.append("horizon_length=");
      builder.append(this.horizon_length_);      builder.append(", ");
      builder.append("planar_regions_list_message=");
      builder.append(this.planar_regions_list_message_);      builder.append(", ");
      builder.append("assume_flat_ground=");
      builder.append(this.assume_flat_ground_);      builder.append(", ");
      builder.append("planner_request_id=");
      builder.append(this.planner_request_id_);      builder.append(", ");
      builder.append("status_publish_period=");
      builder.append(this.status_publish_period_);      builder.append(", ");
      builder.append("requested_swing_planner=");
      builder.append(this.requested_swing_planner_);      builder.append(", ");
      builder.append("generate_log=");
      builder.append(this.generate_log_);
      builder.append("}");
      return builder.toString();
   }
}
