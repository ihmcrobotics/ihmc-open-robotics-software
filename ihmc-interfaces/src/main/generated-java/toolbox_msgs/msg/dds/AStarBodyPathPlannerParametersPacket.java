package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       * This message defines the parameters for the A* body path planner.
       */
public class AStarBodyPathPlannerParametersPacket extends Packet<AStarBodyPathPlannerParametersPacket> implements Settable<AStarBodyPathPlannerParametersPacket>, EpsilonComparable<AStarBodyPathPlannerParametersPacket>
{
   public static final double DEFAULT_NO_VALUE = -11.1;
   /**
            * Unique ID used to identify this message.
            */
   public long sequence_id_;
   /**
            * Whether or not the planner checks for collisions.
            */
   public boolean check_for_collisions_;
   /**
            * Whether the body path plan is post-processed with the smoother.
            */
   public boolean perform_smoothing_;
   /**
            * The node height of a vertex is determined as the average height of all cells within this radius.
            */
   public double snap_radius_;
   /**
            * When computing the vertex height, cells that are this distance below the max height are ignored when taking the average.
            */
   public double min_snap_height_threshold_;
   /**
            * Weight assigned to minimizing the incline the path takes.
            */
   public double incline_cost_weight_;
   /**
            * Deadband applied to the incline in the search.
            */
   public double incline_cost_deadband_;
   /**
            * The maximum incline, in degrees, that is allowed for the body path planner to traverse.
            */
   public double max_incline_;
   /**
            * Width of the collision box used for checking collisions with the environment.
            */
   public double collision_box_size_y_;
   /**
            * Depth of the collision box used for checking collisions with the environment.
            */
   public double collision_box_size_x_;
   /**
            * Intersection height below which collisions are ignored.
            */
   public double collision_box_ground_clearance_;
   /**
            * Weight placed on the gradient for avoiding collisions in the smoother.
            */
   public double smoother_collision_weight_;
   /**
            * Weight placed on the gradient for minimizing the angle between successive segments of the body path.
            */
   public double smoother_smoothness_weight_;
   /**
            * Discount applied to the smoothness gradients of turn points.
            */
   public double smoother_turn_point_smoothness_discount_;
   /**
            * Minimum curvature in degrees to penalize with a gradient.
            */
   public double smoother_min_curvature_to_penalize_;
   /**
            * Weight placed on the gradient for making vertices of the body path plan an equal distance apart.
            */
   public double smoother_equal_spacing_weight_;
   /**
            * Weight placed on a gradient that drives the waypoint towards the initial value.
            */
   public double smoother_displacement_weight_;
   /**
            * Gain applied to the smoother gradient for iterative modifications.
            */
   public double smoother_hill_climb_gain_;
   /**
            * Minimum gradient vector magnitude to terminate the smoother iterations.
            */
   public double smoother_gradient_threshold_to_terminate_;
   /**
            * Distance from the start to perform collision checking. Avoids false-positive collisions of the robot or gantry, for example.
            */
   public double collision_start_tolerance_;
   /**
            * Weight on the obstacle clearance cost from the terrain map
            */
   public double obstacle_clearance_cost_weight_;

   public AStarBodyPathPlannerParametersPacket()
   {
   }

   public AStarBodyPathPlannerParametersPacket(AStarBodyPathPlannerParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(AStarBodyPathPlannerParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      check_for_collisions_ = other.check_for_collisions_;

      perform_smoothing_ = other.perform_smoothing_;

      snap_radius_ = other.snap_radius_;

      min_snap_height_threshold_ = other.min_snap_height_threshold_;

      incline_cost_weight_ = other.incline_cost_weight_;

      incline_cost_deadband_ = other.incline_cost_deadband_;

      max_incline_ = other.max_incline_;

      collision_box_size_y_ = other.collision_box_size_y_;

      collision_box_size_x_ = other.collision_box_size_x_;

      collision_box_ground_clearance_ = other.collision_box_ground_clearance_;

      smoother_collision_weight_ = other.smoother_collision_weight_;

      smoother_smoothness_weight_ = other.smoother_smoothness_weight_;

      smoother_turn_point_smoothness_discount_ = other.smoother_turn_point_smoothness_discount_;

      smoother_min_curvature_to_penalize_ = other.smoother_min_curvature_to_penalize_;

      smoother_equal_spacing_weight_ = other.smoother_equal_spacing_weight_;

      smoother_displacement_weight_ = other.smoother_displacement_weight_;

      smoother_hill_climb_gain_ = other.smoother_hill_climb_gain_;

      smoother_gradient_threshold_to_terminate_ = other.smoother_gradient_threshold_to_terminate_;

      collision_start_tolerance_ = other.collision_start_tolerance_;

      obstacle_clearance_cost_weight_ = other.obstacle_clearance_cost_weight_;

   }

   /**
            * Unique ID used to identify this message.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Whether or not the planner checks for collisions.
            */
   public void setCheckForCollisions(boolean check_for_collisions)
   {
      check_for_collisions_ = check_for_collisions;
   }
   /**
            * Whether or not the planner checks for collisions.
            */
   public boolean getCheckForCollisions()
   {
      return check_for_collisions_;
   }

   /**
            * Whether the body path plan is post-processed with the smoother.
            */
   public void setPerformSmoothing(boolean perform_smoothing)
   {
      perform_smoothing_ = perform_smoothing;
   }
   /**
            * Whether the body path plan is post-processed with the smoother.
            */
   public boolean getPerformSmoothing()
   {
      return perform_smoothing_;
   }

   /**
            * The node height of a vertex is determined as the average height of all cells within this radius.
            */
   public void setSnapRadius(double snap_radius)
   {
      snap_radius_ = snap_radius;
   }
   /**
            * The node height of a vertex is determined as the average height of all cells within this radius.
            */
   public double getSnapRadius()
   {
      return snap_radius_;
   }

   /**
            * When computing the vertex height, cells that are this distance below the max height are ignored when taking the average.
            */
   public void setMinSnapHeightThreshold(double min_snap_height_threshold)
   {
      min_snap_height_threshold_ = min_snap_height_threshold;
   }
   /**
            * When computing the vertex height, cells that are this distance below the max height are ignored when taking the average.
            */
   public double getMinSnapHeightThreshold()
   {
      return min_snap_height_threshold_;
   }

   /**
            * Weight assigned to minimizing the incline the path takes.
            */
   public void setInclineCostWeight(double incline_cost_weight)
   {
      incline_cost_weight_ = incline_cost_weight;
   }
   /**
            * Weight assigned to minimizing the incline the path takes.
            */
   public double getInclineCostWeight()
   {
      return incline_cost_weight_;
   }

   /**
            * Deadband applied to the incline in the search.
            */
   public void setInclineCostDeadband(double incline_cost_deadband)
   {
      incline_cost_deadband_ = incline_cost_deadband;
   }
   /**
            * Deadband applied to the incline in the search.
            */
   public double getInclineCostDeadband()
   {
      return incline_cost_deadband_;
   }

   /**
            * The maximum incline, in degrees, that is allowed for the body path planner to traverse.
            */
   public void setMaxIncline(double max_incline)
   {
      max_incline_ = max_incline;
   }
   /**
            * The maximum incline, in degrees, that is allowed for the body path planner to traverse.
            */
   public double getMaxIncline()
   {
      return max_incline_;
   }

   /**
            * Width of the collision box used for checking collisions with the environment.
            */
   public void setCollisionBoxSizeY(double collision_box_size_y)
   {
      collision_box_size_y_ = collision_box_size_y;
   }
   /**
            * Width of the collision box used for checking collisions with the environment.
            */
   public double getCollisionBoxSizeY()
   {
      return collision_box_size_y_;
   }

   /**
            * Depth of the collision box used for checking collisions with the environment.
            */
   public void setCollisionBoxSizeX(double collision_box_size_x)
   {
      collision_box_size_x_ = collision_box_size_x;
   }
   /**
            * Depth of the collision box used for checking collisions with the environment.
            */
   public double getCollisionBoxSizeX()
   {
      return collision_box_size_x_;
   }

   /**
            * Intersection height below which collisions are ignored.
            */
   public void setCollisionBoxGroundClearance(double collision_box_ground_clearance)
   {
      collision_box_ground_clearance_ = collision_box_ground_clearance;
   }
   /**
            * Intersection height below which collisions are ignored.
            */
   public double getCollisionBoxGroundClearance()
   {
      return collision_box_ground_clearance_;
   }

   /**
            * Weight placed on the gradient for avoiding collisions in the smoother.
            */
   public void setSmootherCollisionWeight(double smoother_collision_weight)
   {
      smoother_collision_weight_ = smoother_collision_weight;
   }
   /**
            * Weight placed on the gradient for avoiding collisions in the smoother.
            */
   public double getSmootherCollisionWeight()
   {
      return smoother_collision_weight_;
   }

   /**
            * Weight placed on the gradient for minimizing the angle between successive segments of the body path.
            */
   public void setSmootherSmoothnessWeight(double smoother_smoothness_weight)
   {
      smoother_smoothness_weight_ = smoother_smoothness_weight;
   }
   /**
            * Weight placed on the gradient for minimizing the angle between successive segments of the body path.
            */
   public double getSmootherSmoothnessWeight()
   {
      return smoother_smoothness_weight_;
   }

   /**
            * Discount applied to the smoothness gradients of turn points.
            */
   public void setSmootherTurnPointSmoothnessDiscount(double smoother_turn_point_smoothness_discount)
   {
      smoother_turn_point_smoothness_discount_ = smoother_turn_point_smoothness_discount;
   }
   /**
            * Discount applied to the smoothness gradients of turn points.
            */
   public double getSmootherTurnPointSmoothnessDiscount()
   {
      return smoother_turn_point_smoothness_discount_;
   }

   /**
            * Minimum curvature in degrees to penalize with a gradient.
            */
   public void setSmootherMinCurvatureToPenalize(double smoother_min_curvature_to_penalize)
   {
      smoother_min_curvature_to_penalize_ = smoother_min_curvature_to_penalize;
   }
   /**
            * Minimum curvature in degrees to penalize with a gradient.
            */
   public double getSmootherMinCurvatureToPenalize()
   {
      return smoother_min_curvature_to_penalize_;
   }

   /**
            * Weight placed on the gradient for making vertices of the body path plan an equal distance apart.
            */
   public void setSmootherEqualSpacingWeight(double smoother_equal_spacing_weight)
   {
      smoother_equal_spacing_weight_ = smoother_equal_spacing_weight;
   }
   /**
            * Weight placed on the gradient for making vertices of the body path plan an equal distance apart.
            */
   public double getSmootherEqualSpacingWeight()
   {
      return smoother_equal_spacing_weight_;
   }

   /**
            * Weight placed on a gradient that drives the waypoint towards the initial value.
            */
   public void setSmootherDisplacementWeight(double smoother_displacement_weight)
   {
      smoother_displacement_weight_ = smoother_displacement_weight;
   }
   /**
            * Weight placed on a gradient that drives the waypoint towards the initial value.
            */
   public double getSmootherDisplacementWeight()
   {
      return smoother_displacement_weight_;
   }

   /**
            * Gain applied to the smoother gradient for iterative modifications.
            */
   public void setSmootherHillClimbGain(double smoother_hill_climb_gain)
   {
      smoother_hill_climb_gain_ = smoother_hill_climb_gain;
   }
   /**
            * Gain applied to the smoother gradient for iterative modifications.
            */
   public double getSmootherHillClimbGain()
   {
      return smoother_hill_climb_gain_;
   }

   /**
            * Minimum gradient vector magnitude to terminate the smoother iterations.
            */
   public void setSmootherGradientThresholdToTerminate(double smoother_gradient_threshold_to_terminate)
   {
      smoother_gradient_threshold_to_terminate_ = smoother_gradient_threshold_to_terminate;
   }
   /**
            * Minimum gradient vector magnitude to terminate the smoother iterations.
            */
   public double getSmootherGradientThresholdToTerminate()
   {
      return smoother_gradient_threshold_to_terminate_;
   }

   /**
            * Distance from the start to perform collision checking. Avoids false-positive collisions of the robot or gantry, for example.
            */
   public void setCollisionStartTolerance(double collision_start_tolerance)
   {
      collision_start_tolerance_ = collision_start_tolerance;
   }
   /**
            * Distance from the start to perform collision checking. Avoids false-positive collisions of the robot or gantry, for example.
            */
   public double getCollisionStartTolerance()
   {
      return collision_start_tolerance_;
   }

   /**
            * Weight on the obstacle clearance cost from the terrain map
            */
   public void setObstacleClearanceCostWeight(double obstacle_clearance_cost_weight)
   {
      obstacle_clearance_cost_weight_ = obstacle_clearance_cost_weight;
   }
   /**
            * Weight on the obstacle clearance cost from the terrain map
            */
   public double getObstacleClearanceCostWeight()
   {
      return obstacle_clearance_cost_weight_;
   }


   public static Supplier<AStarBodyPathPlannerParametersPacketPubSubType> getPubSubType()
   {
      return AStarBodyPathPlannerParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AStarBodyPathPlannerParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AStarBodyPathPlannerParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.check_for_collisions_, other.check_for_collisions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.perform_smoothing_, other.perform_smoothing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.snap_radius_, other.snap_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_snap_height_threshold_, other.min_snap_height_threshold_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.incline_cost_weight_, other.incline_cost_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.incline_cost_deadband_, other.incline_cost_deadband_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_incline_, other.max_incline_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_box_size_y_, other.collision_box_size_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_box_size_x_, other.collision_box_size_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_box_ground_clearance_, other.collision_box_ground_clearance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_collision_weight_, other.smoother_collision_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_smoothness_weight_, other.smoother_smoothness_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_turn_point_smoothness_discount_, other.smoother_turn_point_smoothness_discount_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_min_curvature_to_penalize_, other.smoother_min_curvature_to_penalize_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_equal_spacing_weight_, other.smoother_equal_spacing_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_displacement_weight_, other.smoother_displacement_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_hill_climb_gain_, other.smoother_hill_climb_gain_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.smoother_gradient_threshold_to_terminate_, other.smoother_gradient_threshold_to_terminate_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.collision_start_tolerance_, other.collision_start_tolerance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.obstacle_clearance_cost_weight_, other.obstacle_clearance_cost_weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AStarBodyPathPlannerParametersPacket)) return false;

      AStarBodyPathPlannerParametersPacket otherMyClass = (AStarBodyPathPlannerParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.check_for_collisions_ != otherMyClass.check_for_collisions_) return false;

      if(this.perform_smoothing_ != otherMyClass.perform_smoothing_) return false;

      if(this.snap_radius_ != otherMyClass.snap_radius_) return false;

      if(this.min_snap_height_threshold_ != otherMyClass.min_snap_height_threshold_) return false;

      if(this.incline_cost_weight_ != otherMyClass.incline_cost_weight_) return false;

      if(this.incline_cost_deadband_ != otherMyClass.incline_cost_deadband_) return false;

      if(this.max_incline_ != otherMyClass.max_incline_) return false;

      if(this.collision_box_size_y_ != otherMyClass.collision_box_size_y_) return false;

      if(this.collision_box_size_x_ != otherMyClass.collision_box_size_x_) return false;

      if(this.collision_box_ground_clearance_ != otherMyClass.collision_box_ground_clearance_) return false;

      if(this.smoother_collision_weight_ != otherMyClass.smoother_collision_weight_) return false;

      if(this.smoother_smoothness_weight_ != otherMyClass.smoother_smoothness_weight_) return false;

      if(this.smoother_turn_point_smoothness_discount_ != otherMyClass.smoother_turn_point_smoothness_discount_) return false;

      if(this.smoother_min_curvature_to_penalize_ != otherMyClass.smoother_min_curvature_to_penalize_) return false;

      if(this.smoother_equal_spacing_weight_ != otherMyClass.smoother_equal_spacing_weight_) return false;

      if(this.smoother_displacement_weight_ != otherMyClass.smoother_displacement_weight_) return false;

      if(this.smoother_hill_climb_gain_ != otherMyClass.smoother_hill_climb_gain_) return false;

      if(this.smoother_gradient_threshold_to_terminate_ != otherMyClass.smoother_gradient_threshold_to_terminate_) return false;

      if(this.collision_start_tolerance_ != otherMyClass.collision_start_tolerance_) return false;

      if(this.obstacle_clearance_cost_weight_ != otherMyClass.obstacle_clearance_cost_weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AStarBodyPathPlannerParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("check_for_collisions=");
      builder.append(this.check_for_collisions_);      builder.append(", ");
      builder.append("perform_smoothing=");
      builder.append(this.perform_smoothing_);      builder.append(", ");
      builder.append("snap_radius=");
      builder.append(this.snap_radius_);      builder.append(", ");
      builder.append("min_snap_height_threshold=");
      builder.append(this.min_snap_height_threshold_);      builder.append(", ");
      builder.append("incline_cost_weight=");
      builder.append(this.incline_cost_weight_);      builder.append(", ");
      builder.append("incline_cost_deadband=");
      builder.append(this.incline_cost_deadband_);      builder.append(", ");
      builder.append("max_incline=");
      builder.append(this.max_incline_);      builder.append(", ");
      builder.append("collision_box_size_y=");
      builder.append(this.collision_box_size_y_);      builder.append(", ");
      builder.append("collision_box_size_x=");
      builder.append(this.collision_box_size_x_);      builder.append(", ");
      builder.append("collision_box_ground_clearance=");
      builder.append(this.collision_box_ground_clearance_);      builder.append(", ");
      builder.append("smoother_collision_weight=");
      builder.append(this.smoother_collision_weight_);      builder.append(", ");
      builder.append("smoother_smoothness_weight=");
      builder.append(this.smoother_smoothness_weight_);      builder.append(", ");
      builder.append("smoother_turn_point_smoothness_discount=");
      builder.append(this.smoother_turn_point_smoothness_discount_);      builder.append(", ");
      builder.append("smoother_min_curvature_to_penalize=");
      builder.append(this.smoother_min_curvature_to_penalize_);      builder.append(", ");
      builder.append("smoother_equal_spacing_weight=");
      builder.append(this.smoother_equal_spacing_weight_);      builder.append(", ");
      builder.append("smoother_displacement_weight=");
      builder.append(this.smoother_displacement_weight_);      builder.append(", ");
      builder.append("smoother_hill_climb_gain=");
      builder.append(this.smoother_hill_climb_gain_);      builder.append(", ");
      builder.append("smoother_gradient_threshold_to_terminate=");
      builder.append(this.smoother_gradient_threshold_to_terminate_);      builder.append(", ");
      builder.append("collision_start_tolerance=");
      builder.append(this.collision_start_tolerance_);      builder.append(", ");
      builder.append("obstacle_clearance_cost_weight=");
      builder.append(this.obstacle_clearance_cost_weight_);
      builder.append("}");
      return builder.toString();
   }
}
