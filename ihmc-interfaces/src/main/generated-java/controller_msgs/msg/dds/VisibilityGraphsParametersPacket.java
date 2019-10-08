package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC footstep planning module.
       */
public class VisibilityGraphsParametersPacket extends Packet<VisibilityGraphsParametersPacket> implements Settable<VisibilityGraphsParametersPacket>, EpsilonComparable<VisibilityGraphsParametersPacket>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   public double max_inter_region_connection_length_ = -1.0;
   public double normal_z_threshold_for_accessible_regions_ = -1.0;
   public double navigable_extrusion_distance_ = -1.0;
   public double obstacle_extrusion_distance_ = -1.0;
   public double preferred_obstacle_extrusion_distance_ = -1.0;
   public double obstacle_extrusion_distance_if_not_too_high_to_step_ = -1.0;
   public double too_high_to_step_distance_ = -1.0;
   public double cluster_resolution_ = -1.0;
   public double exploration_distance_from_start_goal_ = -1.0;
   public double planar_region_min_area_;
   public long planar_region_min_size_;
   /**
            * Defines the angle from which two regions are considered orthogonal.
            * It is used to determine if a region should be projected onto another as a polygon or a line.
            * It should be close to 90 degrees.
            * Returns the angle threshold to use to determine if a line or polygon projection method should be used.
            */
   public double region_orthogonal_angle_ = -1.0;
   /**
            * This epsilon is is used when searching to which region the start/goal belongs to.
            * A positive value corresponds to growing all the regions before testing if the start/goal is inside.
            * Returns the value of the epsilon to use.
            */
   public double search_host_region_epsilon_ = 0.03;
   public double can_duck_under_height_ = -1.0;
   public double can_easily_step_over_height_ = -1.0;
   public double length_for_long_inter_region_edge_ = -1.0;
   public boolean perform_post_processing_node_shifting_;
   public boolean introduce_midpoints_in_post_processing_ = true;
   public boolean compute_orientations_to_avoid_obstacles_;
   public double heuristic_weight_ = -1.0;
   public double distance_weight_ = -1.0;
   public double elevation_weight_ = -1.0;
   public double occluded_goal_edge_weight_ = -1.0;
   public double weight_for_inter_region_edge_ = -1.0;
   public boolean return_best_effort_solution_;

   public VisibilityGraphsParametersPacket()
   {
   }

   public VisibilityGraphsParametersPacket(VisibilityGraphsParametersPacket other)
   {
      this();
      set(other);
   }

   public void set(VisibilityGraphsParametersPacket other)
   {
      sequence_id_ = other.sequence_id_;

      max_inter_region_connection_length_ = other.max_inter_region_connection_length_;

      normal_z_threshold_for_accessible_regions_ = other.normal_z_threshold_for_accessible_regions_;

      navigable_extrusion_distance_ = other.navigable_extrusion_distance_;

      obstacle_extrusion_distance_ = other.obstacle_extrusion_distance_;

      preferred_obstacle_extrusion_distance_ = other.preferred_obstacle_extrusion_distance_;

      obstacle_extrusion_distance_if_not_too_high_to_step_ = other.obstacle_extrusion_distance_if_not_too_high_to_step_;

      too_high_to_step_distance_ = other.too_high_to_step_distance_;

      cluster_resolution_ = other.cluster_resolution_;

      exploration_distance_from_start_goal_ = other.exploration_distance_from_start_goal_;

      planar_region_min_area_ = other.planar_region_min_area_;

      planar_region_min_size_ = other.planar_region_min_size_;

      region_orthogonal_angle_ = other.region_orthogonal_angle_;

      search_host_region_epsilon_ = other.search_host_region_epsilon_;

      can_duck_under_height_ = other.can_duck_under_height_;

      can_easily_step_over_height_ = other.can_easily_step_over_height_;

      length_for_long_inter_region_edge_ = other.length_for_long_inter_region_edge_;

      perform_post_processing_node_shifting_ = other.perform_post_processing_node_shifting_;

      introduce_midpoints_in_post_processing_ = other.introduce_midpoints_in_post_processing_;

      compute_orientations_to_avoid_obstacles_ = other.compute_orientations_to_avoid_obstacles_;

      heuristic_weight_ = other.heuristic_weight_;

      distance_weight_ = other.distance_weight_;

      elevation_weight_ = other.elevation_weight_;

      occluded_goal_edge_weight_ = other.occluded_goal_edge_weight_;

      weight_for_inter_region_edge_ = other.weight_for_inter_region_edge_;

      return_best_effort_solution_ = other.return_best_effort_solution_;

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

   public void setMaxInterRegionConnectionLength(double max_inter_region_connection_length)
   {
      max_inter_region_connection_length_ = max_inter_region_connection_length;
   }
   public double getMaxInterRegionConnectionLength()
   {
      return max_inter_region_connection_length_;
   }

   public void setNormalZThresholdForAccessibleRegions(double normal_z_threshold_for_accessible_regions)
   {
      normal_z_threshold_for_accessible_regions_ = normal_z_threshold_for_accessible_regions;
   }
   public double getNormalZThresholdForAccessibleRegions()
   {
      return normal_z_threshold_for_accessible_regions_;
   }

   public void setNavigableExtrusionDistance(double navigable_extrusion_distance)
   {
      navigable_extrusion_distance_ = navigable_extrusion_distance;
   }
   public double getNavigableExtrusionDistance()
   {
      return navigable_extrusion_distance_;
   }

   public void setObstacleExtrusionDistance(double obstacle_extrusion_distance)
   {
      obstacle_extrusion_distance_ = obstacle_extrusion_distance;
   }
   public double getObstacleExtrusionDistance()
   {
      return obstacle_extrusion_distance_;
   }

   public void setPreferredObstacleExtrusionDistance(double preferred_obstacle_extrusion_distance)
   {
      preferred_obstacle_extrusion_distance_ = preferred_obstacle_extrusion_distance;
   }
   public double getPreferredObstacleExtrusionDistance()
   {
      return preferred_obstacle_extrusion_distance_;
   }

   public void setObstacleExtrusionDistanceIfNotTooHighToStep(double obstacle_extrusion_distance_if_not_too_high_to_step)
   {
      obstacle_extrusion_distance_if_not_too_high_to_step_ = obstacle_extrusion_distance_if_not_too_high_to_step;
   }
   public double getObstacleExtrusionDistanceIfNotTooHighToStep()
   {
      return obstacle_extrusion_distance_if_not_too_high_to_step_;
   }

   public void setTooHighToStepDistance(double too_high_to_step_distance)
   {
      too_high_to_step_distance_ = too_high_to_step_distance;
   }
   public double getTooHighToStepDistance()
   {
      return too_high_to_step_distance_;
   }

   public void setClusterResolution(double cluster_resolution)
   {
      cluster_resolution_ = cluster_resolution;
   }
   public double getClusterResolution()
   {
      return cluster_resolution_;
   }

   public void setExplorationDistanceFromStartGoal(double exploration_distance_from_start_goal)
   {
      exploration_distance_from_start_goal_ = exploration_distance_from_start_goal;
   }
   public double getExplorationDistanceFromStartGoal()
   {
      return exploration_distance_from_start_goal_;
   }

   public void setPlanarRegionMinArea(double planar_region_min_area)
   {
      planar_region_min_area_ = planar_region_min_area;
   }
   public double getPlanarRegionMinArea()
   {
      return planar_region_min_area_;
   }

   public void setPlanarRegionMinSize(long planar_region_min_size)
   {
      planar_region_min_size_ = planar_region_min_size;
   }
   public long getPlanarRegionMinSize()
   {
      return planar_region_min_size_;
   }

   /**
            * Defines the angle from which two regions are considered orthogonal.
            * It is used to determine if a region should be projected onto another as a polygon or a line.
            * It should be close to 90 degrees.
            * Returns the angle threshold to use to determine if a line or polygon projection method should be used.
            */
   public void setRegionOrthogonalAngle(double region_orthogonal_angle)
   {
      region_orthogonal_angle_ = region_orthogonal_angle;
   }
   /**
            * Defines the angle from which two regions are considered orthogonal.
            * It is used to determine if a region should be projected onto another as a polygon or a line.
            * It should be close to 90 degrees.
            * Returns the angle threshold to use to determine if a line or polygon projection method should be used.
            */
   public double getRegionOrthogonalAngle()
   {
      return region_orthogonal_angle_;
   }

   /**
            * This epsilon is is used when searching to which region the start/goal belongs to.
            * A positive value corresponds to growing all the regions before testing if the start/goal is inside.
            * Returns the value of the epsilon to use.
            */
   public void setSearchHostRegionEpsilon(double search_host_region_epsilon)
   {
      search_host_region_epsilon_ = search_host_region_epsilon;
   }
   /**
            * This epsilon is is used when searching to which region the start/goal belongs to.
            * A positive value corresponds to growing all the regions before testing if the start/goal is inside.
            * Returns the value of the epsilon to use.
            */
   public double getSearchHostRegionEpsilon()
   {
      return search_host_region_epsilon_;
   }

   public void setCanDuckUnderHeight(double can_duck_under_height)
   {
      can_duck_under_height_ = can_duck_under_height;
   }
   public double getCanDuckUnderHeight()
   {
      return can_duck_under_height_;
   }

   public void setCanEasilyStepOverHeight(double can_easily_step_over_height)
   {
      can_easily_step_over_height_ = can_easily_step_over_height;
   }
   public double getCanEasilyStepOverHeight()
   {
      return can_easily_step_over_height_;
   }

   public void setLengthForLongInterRegionEdge(double length_for_long_inter_region_edge)
   {
      length_for_long_inter_region_edge_ = length_for_long_inter_region_edge;
   }
   public double getLengthForLongInterRegionEdge()
   {
      return length_for_long_inter_region_edge_;
   }

   public void setPerformPostProcessingNodeShifting(boolean perform_post_processing_node_shifting)
   {
      perform_post_processing_node_shifting_ = perform_post_processing_node_shifting;
   }
   public boolean getPerformPostProcessingNodeShifting()
   {
      return perform_post_processing_node_shifting_;
   }

   public void setIntroduceMidpointsInPostProcessing(boolean introduce_midpoints_in_post_processing)
   {
      introduce_midpoints_in_post_processing_ = introduce_midpoints_in_post_processing;
   }
   public boolean getIntroduceMidpointsInPostProcessing()
   {
      return introduce_midpoints_in_post_processing_;
   }

   public void setComputeOrientationsToAvoidObstacles(boolean compute_orientations_to_avoid_obstacles)
   {
      compute_orientations_to_avoid_obstacles_ = compute_orientations_to_avoid_obstacles;
   }
   public boolean getComputeOrientationsToAvoidObstacles()
   {
      return compute_orientations_to_avoid_obstacles_;
   }

   public void setHeuristicWeight(double heuristic_weight)
   {
      heuristic_weight_ = heuristic_weight;
   }
   public double getHeuristicWeight()
   {
      return heuristic_weight_;
   }

   public void setDistanceWeight(double distance_weight)
   {
      distance_weight_ = distance_weight;
   }
   public double getDistanceWeight()
   {
      return distance_weight_;
   }

   public void setElevationWeight(double elevation_weight)
   {
      elevation_weight_ = elevation_weight;
   }
   public double getElevationWeight()
   {
      return elevation_weight_;
   }

   public void setOccludedGoalEdgeWeight(double occluded_goal_edge_weight)
   {
      occluded_goal_edge_weight_ = occluded_goal_edge_weight;
   }
   public double getOccludedGoalEdgeWeight()
   {
      return occluded_goal_edge_weight_;
   }

   public void setWeightForInterRegionEdge(double weight_for_inter_region_edge)
   {
      weight_for_inter_region_edge_ = weight_for_inter_region_edge;
   }
   public double getWeightForInterRegionEdge()
   {
      return weight_for_inter_region_edge_;
   }

   public void setReturnBestEffortSolution(boolean return_best_effort_solution)
   {
      return_best_effort_solution_ = return_best_effort_solution;
   }
   public boolean getReturnBestEffortSolution()
   {
      return return_best_effort_solution_;
   }


   public static Supplier<VisibilityGraphsParametersPacketPubSubType> getPubSubType()
   {
      return VisibilityGraphsParametersPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return VisibilityGraphsParametersPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(VisibilityGraphsParametersPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_inter_region_connection_length_, other.max_inter_region_connection_length_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.normal_z_threshold_for_accessible_regions_, other.normal_z_threshold_for_accessible_regions_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.navigable_extrusion_distance_, other.navigable_extrusion_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.obstacle_extrusion_distance_, other.obstacle_extrusion_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preferred_obstacle_extrusion_distance_, other.preferred_obstacle_extrusion_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.obstacle_extrusion_distance_if_not_too_high_to_step_, other.obstacle_extrusion_distance_if_not_too_high_to_step_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.too_high_to_step_distance_, other.too_high_to_step_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.cluster_resolution_, other.cluster_resolution_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.exploration_distance_from_start_goal_, other.exploration_distance_from_start_goal_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planar_region_min_area_, other.planar_region_min_area_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.planar_region_min_size_, other.planar_region_min_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.region_orthogonal_angle_, other.region_orthogonal_angle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.search_host_region_epsilon_, other.search_host_region_epsilon_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.can_duck_under_height_, other.can_duck_under_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.can_easily_step_over_height_, other.can_easily_step_over_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.length_for_long_inter_region_edge_, other.length_for_long_inter_region_edge_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.perform_post_processing_node_shifting_, other.perform_post_processing_node_shifting_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.introduce_midpoints_in_post_processing_, other.introduce_midpoints_in_post_processing_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.compute_orientations_to_avoid_obstacles_, other.compute_orientations_to_avoid_obstacles_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.heuristic_weight_, other.heuristic_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_weight_, other.distance_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.elevation_weight_, other.elevation_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.occluded_goal_edge_weight_, other.occluded_goal_edge_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.weight_for_inter_region_edge_, other.weight_for_inter_region_edge_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.return_best_effort_solution_, other.return_best_effort_solution_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof VisibilityGraphsParametersPacket)) return false;

      VisibilityGraphsParametersPacket otherMyClass = (VisibilityGraphsParametersPacket) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.max_inter_region_connection_length_ != otherMyClass.max_inter_region_connection_length_) return false;

      if(this.normal_z_threshold_for_accessible_regions_ != otherMyClass.normal_z_threshold_for_accessible_regions_) return false;

      if(this.navigable_extrusion_distance_ != otherMyClass.navigable_extrusion_distance_) return false;

      if(this.obstacle_extrusion_distance_ != otherMyClass.obstacle_extrusion_distance_) return false;

      if(this.preferred_obstacle_extrusion_distance_ != otherMyClass.preferred_obstacle_extrusion_distance_) return false;

      if(this.obstacle_extrusion_distance_if_not_too_high_to_step_ != otherMyClass.obstacle_extrusion_distance_if_not_too_high_to_step_) return false;

      if(this.too_high_to_step_distance_ != otherMyClass.too_high_to_step_distance_) return false;

      if(this.cluster_resolution_ != otherMyClass.cluster_resolution_) return false;

      if(this.exploration_distance_from_start_goal_ != otherMyClass.exploration_distance_from_start_goal_) return false;

      if(this.planar_region_min_area_ != otherMyClass.planar_region_min_area_) return false;

      if(this.planar_region_min_size_ != otherMyClass.planar_region_min_size_) return false;

      if(this.region_orthogonal_angle_ != otherMyClass.region_orthogonal_angle_) return false;

      if(this.search_host_region_epsilon_ != otherMyClass.search_host_region_epsilon_) return false;

      if(this.can_duck_under_height_ != otherMyClass.can_duck_under_height_) return false;

      if(this.can_easily_step_over_height_ != otherMyClass.can_easily_step_over_height_) return false;

      if(this.length_for_long_inter_region_edge_ != otherMyClass.length_for_long_inter_region_edge_) return false;

      if(this.perform_post_processing_node_shifting_ != otherMyClass.perform_post_processing_node_shifting_) return false;

      if(this.introduce_midpoints_in_post_processing_ != otherMyClass.introduce_midpoints_in_post_processing_) return false;

      if(this.compute_orientations_to_avoid_obstacles_ != otherMyClass.compute_orientations_to_avoid_obstacles_) return false;

      if(this.heuristic_weight_ != otherMyClass.heuristic_weight_) return false;

      if(this.distance_weight_ != otherMyClass.distance_weight_) return false;

      if(this.elevation_weight_ != otherMyClass.elevation_weight_) return false;

      if(this.occluded_goal_edge_weight_ != otherMyClass.occluded_goal_edge_weight_) return false;

      if(this.weight_for_inter_region_edge_ != otherMyClass.weight_for_inter_region_edge_) return false;

      if(this.return_best_effort_solution_ != otherMyClass.return_best_effort_solution_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("VisibilityGraphsParametersPacket {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("max_inter_region_connection_length=");
      builder.append(this.max_inter_region_connection_length_);      builder.append(", ");
      builder.append("normal_z_threshold_for_accessible_regions=");
      builder.append(this.normal_z_threshold_for_accessible_regions_);      builder.append(", ");
      builder.append("navigable_extrusion_distance=");
      builder.append(this.navigable_extrusion_distance_);      builder.append(", ");
      builder.append("obstacle_extrusion_distance=");
      builder.append(this.obstacle_extrusion_distance_);      builder.append(", ");
      builder.append("preferred_obstacle_extrusion_distance=");
      builder.append(this.preferred_obstacle_extrusion_distance_);      builder.append(", ");
      builder.append("obstacle_extrusion_distance_if_not_too_high_to_step=");
      builder.append(this.obstacle_extrusion_distance_if_not_too_high_to_step_);      builder.append(", ");
      builder.append("too_high_to_step_distance=");
      builder.append(this.too_high_to_step_distance_);      builder.append(", ");
      builder.append("cluster_resolution=");
      builder.append(this.cluster_resolution_);      builder.append(", ");
      builder.append("exploration_distance_from_start_goal=");
      builder.append(this.exploration_distance_from_start_goal_);      builder.append(", ");
      builder.append("planar_region_min_area=");
      builder.append(this.planar_region_min_area_);      builder.append(", ");
      builder.append("planar_region_min_size=");
      builder.append(this.planar_region_min_size_);      builder.append(", ");
      builder.append("region_orthogonal_angle=");
      builder.append(this.region_orthogonal_angle_);      builder.append(", ");
      builder.append("search_host_region_epsilon=");
      builder.append(this.search_host_region_epsilon_);      builder.append(", ");
      builder.append("can_duck_under_height=");
      builder.append(this.can_duck_under_height_);      builder.append(", ");
      builder.append("can_easily_step_over_height=");
      builder.append(this.can_easily_step_over_height_);      builder.append(", ");
      builder.append("length_for_long_inter_region_edge=");
      builder.append(this.length_for_long_inter_region_edge_);      builder.append(", ");
      builder.append("perform_post_processing_node_shifting=");
      builder.append(this.perform_post_processing_node_shifting_);      builder.append(", ");
      builder.append("introduce_midpoints_in_post_processing=");
      builder.append(this.introduce_midpoints_in_post_processing_);      builder.append(", ");
      builder.append("compute_orientations_to_avoid_obstacles=");
      builder.append(this.compute_orientations_to_avoid_obstacles_);      builder.append(", ");
      builder.append("heuristic_weight=");
      builder.append(this.heuristic_weight_);      builder.append(", ");
      builder.append("distance_weight=");
      builder.append(this.distance_weight_);      builder.append(", ");
      builder.append("elevation_weight=");
      builder.append(this.elevation_weight_);      builder.append(", ");
      builder.append("occluded_goal_edge_weight=");
      builder.append(this.occluded_goal_edge_weight_);      builder.append(", ");
      builder.append("weight_for_inter_region_edge=");
      builder.append(this.weight_for_inter_region_edge_);      builder.append(", ");
      builder.append("return_best_effort_solution=");
      builder.append(this.return_best_effort_solution_);
      builder.append("}");
      return builder.toString();
   }
}
