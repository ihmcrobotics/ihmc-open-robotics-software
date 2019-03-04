package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class FootstepPlannerCostParametersPacket extends Packet<FootstepPlannerCostParametersPacket> implements Settable<FootstepPlannerCostParametersPacket>, EpsilonComparable<FootstepPlannerCostParametersPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * When using a cost based planning approach this value defined how the yaw of a footstep will be
            * weighted in comparison to its position.
            */
   public double yaw_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the pitch will be weighted.
            */
   public double pitch_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the roll will be weighted.
            */
   public double roll_weight_ = -1.0;
   /**
            * When using a cost based planning approach, this value defines how the forward (or backward) displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getLateralWeight()}
            */
   public double forward_weight_ = -1.0;
   /**
            * When using a cost based planning approach, this value defines how the lateral displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getForwardWeight()}
            */
   public double lateral_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the height change when stepping up will be
            * weighted.
            */
   public double step_up_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines how the height change when stepping down will be
            * weighted.
            */
   public double step_down_weight_ = -1.0;
   /**
            * When using a cost based planning approach this value defines the cost that is added for each step
            * taken. Setting this value to a high number will favor plans with less steps.
            */
   public double cost_per_step_ = -1.0;
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
            */
   public boolean use_quadratic_distance_cost_;
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
            */
   public boolean use_quadratic_height_cost_;
   /**
            * Gets the weight for the heuristics in the A Star planner.
            */
   public double a_star_heuristics_weight_ = -1.0;
   /**
            * Gets the weight for the heuristics in the Visibility graph with A star planner.
            */
   public double vis_graph_with_a_star_heuristics_weight_ = -1.0;
   /**
            * Gets the weight for the heuristics in the Depth First planner.
            */
   public double depth_first_heuristics_weight_ = -1.0;
   /**
            * Gets the weight for the heuristics in the Body path based planner.
            */
   public double body_path_based_heuristics_weight_ = -1.0;
   /**
            * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
            * @see FootstepPlannerCostParameters#getBoundingBoxCost
            */
   public double maximum_2d_distance_from_bounding_box_to_penalize_ = -1.0;
   /**
            * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
            * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
            * {@code c * (1 - d / d_max)}, where d_max is this value.
            */
   public double bounding_box_cost_ = -1.0;

   public FootstepPlannerCostParametersPacket()
   {
   }

   public FootstepPlannerCostParametersPacket(FootstepPlannerCostParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(FootstepPlannerCostParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      yaw_weight_ = other.yaw_weight_;

      pitch_weight_ = other.pitch_weight_;

      roll_weight_ = other.roll_weight_;

      forward_weight_ = other.forward_weight_;

      lateral_weight_ = other.lateral_weight_;

      step_up_weight_ = other.step_up_weight_;

      step_down_weight_ = other.step_down_weight_;

      cost_per_step_ = other.cost_per_step_;

      use_quadratic_distance_cost_ = other.use_quadratic_distance_cost_;

      use_quadratic_height_cost_ = other.use_quadratic_height_cost_;

      a_star_heuristics_weight_ = other.a_star_heuristics_weight_;

      vis_graph_with_a_star_heuristics_weight_ = other.vis_graph_with_a_star_heuristics_weight_;

      depth_first_heuristics_weight_ = other.depth_first_heuristics_weight_;

      body_path_based_heuristics_weight_ = other.body_path_based_heuristics_weight_;

      maximum_2d_distance_from_bounding_box_to_penalize_ = other.maximum_2d_distance_from_bounding_box_to_penalize_;

      bounding_box_cost_ = other.bounding_box_cost_;

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
            * When using a cost based planning approach this value defined how the yaw of a footstep will be
            * weighted in comparison to its position.
            */
   public void setYawWeight(double yaw_weight)
   {
      yaw_weight_ = yaw_weight;
   }
   /**
            * When using a cost based planning approach this value defined how the yaw of a footstep will be
            * weighted in comparison to its position.
            */
   public double getYawWeight()
   {
      return yaw_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the pitch will be weighted.
            */
   public void setPitchWeight(double pitch_weight)
   {
      pitch_weight_ = pitch_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the pitch will be weighted.
            */
   public double getPitchWeight()
   {
      return pitch_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the roll will be weighted.
            */
   public void setRollWeight(double roll_weight)
   {
      roll_weight_ = roll_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the roll will be weighted.
            */
   public double getRollWeight()
   {
      return roll_weight_;
   }

   /**
            * When using a cost based planning approach, this value defines how the forward (or backward) displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getLateralWeight()}
            */
   public void setForwardWeight(double forward_weight)
   {
      forward_weight_ = forward_weight;
   }
   /**
            * When using a cost based planning approach, this value defines how the forward (or backward) displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getLateralWeight()}
            */
   public double getForwardWeight()
   {
      return forward_weight_;
   }

   /**
            * When using a cost based planning approach, this value defines how the lateral displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getForwardWeight()}
            */
   public void setLateralWeight(double lateral_weight)
   {
      lateral_weight_ = lateral_weight;
   }
   /**
            * When using a cost based planning approach, this value defines how the lateral displacement
            * of a footstep will be weighted in comparison to its position. Note that when using a Euclidean distance, this
            * weight is averaged with the value returned by {@link #getForwardWeight()}
            */
   public double getLateralWeight()
   {
      return lateral_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the height change when stepping up will be
            * weighted.
            */
   public void setStepUpWeight(double step_up_weight)
   {
      step_up_weight_ = step_up_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the height change when stepping up will be
            * weighted.
            */
   public double getStepUpWeight()
   {
      return step_up_weight_;
   }

   /**
            * When using a cost based planning approach this value defines how the height change when stepping down will be
            * weighted.
            */
   public void setStepDownWeight(double step_down_weight)
   {
      step_down_weight_ = step_down_weight;
   }
   /**
            * When using a cost based planning approach this value defines how the height change when stepping down will be
            * weighted.
            */
   public double getStepDownWeight()
   {
      return step_down_weight_;
   }

   /**
            * When using a cost based planning approach this value defines the cost that is added for each step
            * taken. Setting this value to a high number will favor plans with less steps.
            */
   public void setCostPerStep(double cost_per_step)
   {
      cost_per_step_ = cost_per_step;
   }
   /**
            * When using a cost based planning approach this value defines the cost that is added for each step
            * taken. Setting this value to a high number will favor plans with less steps.
            */
   public double getCostPerStep()
   {
      return cost_per_step_;
   }

   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
            */
   public void setUseQuadraticDistanceCost(boolean use_quadratic_distance_cost)
   {
      use_quadratic_distance_cost_ = use_quadratic_distance_cost;
   }
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link EuclideanDistanceAndYawBasedCost}
            */
   public boolean getUseQuadraticDistanceCost()
   {
      return use_quadratic_distance_cost_;
   }

   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
            */
   public void setUseQuadraticHeightCost(boolean use_quadratic_height_cost)
   {
      use_quadratic_height_cost_ = use_quadratic_height_cost;
   }
   /**
            * Determines which cost function for distance and yaw to use, between {@link QuadraticDistanceAndYawCost} and {@link LinearHeightCost}
            */
   public boolean getUseQuadraticHeightCost()
   {
      return use_quadratic_height_cost_;
   }

   /**
            * Gets the weight for the heuristics in the A Star planner.
            */
   public void setAStarHeuristicsWeight(double a_star_heuristics_weight)
   {
      a_star_heuristics_weight_ = a_star_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the A Star planner.
            */
   public double getAStarHeuristicsWeight()
   {
      return a_star_heuristics_weight_;
   }

   /**
            * Gets the weight for the heuristics in the Visibility graph with A star planner.
            */
   public void setVisGraphWithAStarHeuristicsWeight(double vis_graph_with_a_star_heuristics_weight)
   {
      vis_graph_with_a_star_heuristics_weight_ = vis_graph_with_a_star_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the Visibility graph with A star planner.
            */
   public double getVisGraphWithAStarHeuristicsWeight()
   {
      return vis_graph_with_a_star_heuristics_weight_;
   }

   /**
            * Gets the weight for the heuristics in the Depth First planner.
            */
   public void setDepthFirstHeuristicsWeight(double depth_first_heuristics_weight)
   {
      depth_first_heuristics_weight_ = depth_first_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the Depth First planner.
            */
   public double getDepthFirstHeuristicsWeight()
   {
      return depth_first_heuristics_weight_;
   }

   /**
            * Gets the weight for the heuristics in the Body path based planner.
            */
   public void setBodyPathBasedHeuristicsWeight(double body_path_based_heuristics_weight)
   {
      body_path_based_heuristics_weight_ = body_path_based_heuristics_weight;
   }
   /**
            * Gets the weight for the heuristics in the Body path based planner.
            */
   public double getBodyPathBasedHeuristicsWeight()
   {
      return body_path_based_heuristics_weight_;
   }

   /**
            * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
            * @see FootstepPlannerCostParameters#getBoundingBoxCost
            */
   public void setMaximum2dDistanceFromBoundingBoxToPenalize(double maximum_2d_distance_from_bounding_box_to_penalize)
   {
      maximum_2d_distance_from_bounding_box_to_penalize_ = maximum_2d_distance_from_bounding_box_to_penalize;
   }
   /**
            * If this value is non-zero, nodes will be given cost if the bounding box is within this xy distance of a planar region
            * @see FootstepPlannerCostParameters#getBoundingBoxCost
            */
   public double getMaximum2dDistanceFromBoundingBoxToPenalize()
   {
      return maximum_2d_distance_from_bounding_box_to_penalize_;
   }

   /**
            * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
            * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
            * {@code c * (1 - d / d_max)}, where d_max is this value.
            */
   public void setBoundingBoxCost(double bounding_box_cost)
   {
      bounding_box_cost_ = bounding_box_cost;
   }
   /**
            * If a node doesn't have bounding box collisions at the default dimensions, but does when increasing the xy dimensions by d,
            * where d < getMaximum2DDistanceFromBoundingBoxToPenalize, there will be a cost given to the node of:
            * {@code c * (1 - d / d_max)}, where d_max is this value.
            */
   public double getBoundingBoxCost()
   {
      return bounding_box_cost_;
   }


   public static Supplier<FootstepPlannerCostParametersPacketPubSubType> getPubSubType()
   {
      return FootstepPlannerCostParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepPlannerCostParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepPlannerCostParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.yaw_weight_, other.yaw_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pitch_weight_, other.pitch_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.roll_weight_, other.roll_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.forward_weight_, other.forward_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.lateral_weight_, other.lateral_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_up_weight_, other.step_up_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_down_weight_, other.step_down_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cost_per_step_, other.cost_per_step_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_quadratic_distance_cost_, other.use_quadratic_distance_cost_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_quadratic_height_cost_, other.use_quadratic_height_cost_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.a_star_heuristics_weight_, other.a_star_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.vis_graph_with_a_star_heuristics_weight_, other.vis_graph_with_a_star_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.depth_first_heuristics_weight_, other.depth_first_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.body_path_based_heuristics_weight_, other.body_path_based_heuristics_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.maximum_2d_distance_from_bounding_box_to_penalize_, other.maximum_2d_distance_from_bounding_box_to_penalize_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.bounding_box_cost_, other.bounding_box_cost_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepPlannerCostParametersPacket)) return false;

      FootstepPlannerCostParametersPacket otherMyClass = (FootstepPlannerCostParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.yaw_weight_ != otherMyClass.yaw_weight_) return false;

      if(this.pitch_weight_ != otherMyClass.pitch_weight_) return false;

      if(this.roll_weight_ != otherMyClass.roll_weight_) return false;

      if(this.forward_weight_ != otherMyClass.forward_weight_) return false;

      if(this.lateral_weight_ != otherMyClass.lateral_weight_) return false;

      if(this.step_up_weight_ != otherMyClass.step_up_weight_) return false;

      if(this.step_down_weight_ != otherMyClass.step_down_weight_) return false;

      if(this.cost_per_step_ != otherMyClass.cost_per_step_) return false;

      if(this.use_quadratic_distance_cost_ != otherMyClass.use_quadratic_distance_cost_) return false;

      if(this.use_quadratic_height_cost_ != otherMyClass.use_quadratic_height_cost_) return false;

      if(this.a_star_heuristics_weight_ != otherMyClass.a_star_heuristics_weight_) return false;

      if(this.vis_graph_with_a_star_heuristics_weight_ != otherMyClass.vis_graph_with_a_star_heuristics_weight_) return false;

      if(this.depth_first_heuristics_weight_ != otherMyClass.depth_first_heuristics_weight_) return false;

      if(this.body_path_based_heuristics_weight_ != otherMyClass.body_path_based_heuristics_weight_) return false;

      if(this.maximum_2d_distance_from_bounding_box_to_penalize_ != otherMyClass.maximum_2d_distance_from_bounding_box_to_penalize_) return false;

      if(this.bounding_box_cost_ != otherMyClass.bounding_box_cost_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepPlannerCostParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("yaw_weight=");
      builder.append(this.yaw_weight_);      builder.append(", ");
      builder.append("pitch_weight=");
      builder.append(this.pitch_weight_);      builder.append(", ");
      builder.append("roll_weight=");
      builder.append(this.roll_weight_);      builder.append(", ");
      builder.append("forward_weight=");
      builder.append(this.forward_weight_);      builder.append(", ");
      builder.append("lateral_weight=");
      builder.append(this.lateral_weight_);      builder.append(", ");
      builder.append("step_up_weight=");
      builder.append(this.step_up_weight_);      builder.append(", ");
      builder.append("step_down_weight=");
      builder.append(this.step_down_weight_);      builder.append(", ");
      builder.append("cost_per_step=");
      builder.append(this.cost_per_step_);      builder.append(", ");
      builder.append("use_quadratic_distance_cost=");
      builder.append(this.use_quadratic_distance_cost_);      builder.append(", ");
      builder.append("use_quadratic_height_cost=");
      builder.append(this.use_quadratic_height_cost_);      builder.append(", ");
      builder.append("a_star_heuristics_weight=");
      builder.append(this.a_star_heuristics_weight_);      builder.append(", ");
      builder.append("vis_graph_with_a_star_heuristics_weight=");
      builder.append(this.vis_graph_with_a_star_heuristics_weight_);      builder.append(", ");
      builder.append("depth_first_heuristics_weight=");
      builder.append(this.depth_first_heuristics_weight_);      builder.append(", ");
      builder.append("body_path_based_heuristics_weight=");
      builder.append(this.body_path_based_heuristics_weight_);      builder.append(", ");
      builder.append("maximum_2d_distance_from_bounding_box_to_penalize=");
      builder.append(this.maximum_2d_distance_from_bounding_box_to_penalize_);      builder.append(", ");
      builder.append("bounding_box_cost=");
      builder.append(this.bounding_box_cost_);
      builder.append("}");
      return builder.toString();
   }
}
