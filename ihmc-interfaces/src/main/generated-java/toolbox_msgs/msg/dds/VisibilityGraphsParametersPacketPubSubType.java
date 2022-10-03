package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "VisibilityGraphsParametersPacket" defined in "VisibilityGraphsParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from VisibilityGraphsParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit VisibilityGraphsParametersPacket_.idl instead.
*
*/
public class VisibilityGraphsParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::VisibilityGraphsParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data) throws java.io.IOException
   {
      deserializeCDR.deserialize(serializedPayload);
      read(data, deserializeCDR);
      deserializeCDR.finishDeserialize();
   }

   public static int getMaxCdrSerializedSize()
   {
      return getMaxCdrSerializedSize(0);
   }

   public static int getMaxCdrSerializedSize(int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_6(data.getMaxInterRegionConnectionLength());

      cdr.write_type_6(data.getNormalZThresholdForAccessibleRegions());

      cdr.write_type_6(data.getNavigableExtrusionDistance());

      cdr.write_type_6(data.getObstacleExtrusionDistance());

      cdr.write_type_6(data.getObstacleExtrusionDistanceIfNotTooHighToStep());

      cdr.write_type_6(data.getTooHighToStepDistance());

      cdr.write_type_6(data.getHeightForMaxAvoidance());

      cdr.write_type_6(data.getClusterResolution());

      cdr.write_type_6(data.getExplorationDistanceFromStartGoal());

      cdr.write_type_6(data.getPlanarRegionMinArea());

      cdr.write_type_4(data.getPlanarRegionMinSize());

      cdr.write_type_6(data.getRegionOrthogonalAngle());

      cdr.write_type_6(data.getSearchHostRegionEpsilon());

      cdr.write_type_6(data.getCanDuckUnderHeight());

      cdr.write_type_6(data.getCanEasilyStepOverHeight());

      cdr.write_type_6(data.getLengthForLongInterRegionEdge());

      cdr.write_type_7(data.getPerformPostProcessingNodeShifting());

      cdr.write_type_7(data.getIntroduceMidpointsInPostProcessing());

      cdr.write_type_7(data.getComputeOrientationsToAvoidObstacles());

      cdr.write_type_6(data.getHeuristicWeight());

      cdr.write_type_6(data.getDistanceWeight());

      cdr.write_type_6(data.getElevationWeight());

      cdr.write_type_6(data.getOccludedGoalEdgeWeight());

      cdr.write_type_6(data.getWeightForInterRegionEdge());

      cdr.write_type_7(data.getReturnBestEffortSolution());

      cdr.write_type_7(data.getOptimizeForNarrowPassage());

   }

   public static void read(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setMaxInterRegionConnectionLength(cdr.read_type_6());
      	
      data.setNormalZThresholdForAccessibleRegions(cdr.read_type_6());
      	
      data.setNavigableExtrusionDistance(cdr.read_type_6());
      	
      data.setObstacleExtrusionDistance(cdr.read_type_6());
      	
      data.setObstacleExtrusionDistanceIfNotTooHighToStep(cdr.read_type_6());
      	
      data.setTooHighToStepDistance(cdr.read_type_6());
      	
      data.setHeightForMaxAvoidance(cdr.read_type_6());
      	
      data.setClusterResolution(cdr.read_type_6());
      	
      data.setExplorationDistanceFromStartGoal(cdr.read_type_6());
      	
      data.setPlanarRegionMinArea(cdr.read_type_6());
      	
      data.setPlanarRegionMinSize(cdr.read_type_4());
      	
      data.setRegionOrthogonalAngle(cdr.read_type_6());
      	
      data.setSearchHostRegionEpsilon(cdr.read_type_6());
      	
      data.setCanDuckUnderHeight(cdr.read_type_6());
      	
      data.setCanEasilyStepOverHeight(cdr.read_type_6());
      	
      data.setLengthForLongInterRegionEdge(cdr.read_type_6());
      	
      data.setPerformPostProcessingNodeShifting(cdr.read_type_7());
      	
      data.setIntroduceMidpointsInPostProcessing(cdr.read_type_7());
      	
      data.setComputeOrientationsToAvoidObstacles(cdr.read_type_7());
      	
      data.setHeuristicWeight(cdr.read_type_6());
      	
      data.setDistanceWeight(cdr.read_type_6());
      	
      data.setElevationWeight(cdr.read_type_6());
      	
      data.setOccludedGoalEdgeWeight(cdr.read_type_6());
      	
      data.setWeightForInterRegionEdge(cdr.read_type_6());
      	
      data.setReturnBestEffortSolution(cdr.read_type_7());
      	
      data.setOptimizeForNarrowPassage(cdr.read_type_7());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_6("max_inter_region_connection_length", data.getMaxInterRegionConnectionLength());
      ser.write_type_6("normal_z_threshold_for_accessible_regions", data.getNormalZThresholdForAccessibleRegions());
      ser.write_type_6("navigable_extrusion_distance", data.getNavigableExtrusionDistance());
      ser.write_type_6("obstacle_extrusion_distance", data.getObstacleExtrusionDistance());
      ser.write_type_6("obstacle_extrusion_distance_if_not_too_high_to_step", data.getObstacleExtrusionDistanceIfNotTooHighToStep());
      ser.write_type_6("too_high_to_step_distance", data.getTooHighToStepDistance());
      ser.write_type_6("height_for_max_avoidance", data.getHeightForMaxAvoidance());
      ser.write_type_6("cluster_resolution", data.getClusterResolution());
      ser.write_type_6("exploration_distance_from_start_goal", data.getExplorationDistanceFromStartGoal());
      ser.write_type_6("planar_region_min_area", data.getPlanarRegionMinArea());
      ser.write_type_4("planar_region_min_size", data.getPlanarRegionMinSize());
      ser.write_type_6("region_orthogonal_angle", data.getRegionOrthogonalAngle());
      ser.write_type_6("search_host_region_epsilon", data.getSearchHostRegionEpsilon());
      ser.write_type_6("can_duck_under_height", data.getCanDuckUnderHeight());
      ser.write_type_6("can_easily_step_over_height", data.getCanEasilyStepOverHeight());
      ser.write_type_6("length_for_long_inter_region_edge", data.getLengthForLongInterRegionEdge());
      ser.write_type_7("perform_post_processing_node_shifting", data.getPerformPostProcessingNodeShifting());
      ser.write_type_7("introduce_midpoints_in_post_processing", data.getIntroduceMidpointsInPostProcessing());
      ser.write_type_7("compute_orientations_to_avoid_obstacles", data.getComputeOrientationsToAvoidObstacles());
      ser.write_type_6("heuristic_weight", data.getHeuristicWeight());
      ser.write_type_6("distance_weight", data.getDistanceWeight());
      ser.write_type_6("elevation_weight", data.getElevationWeight());
      ser.write_type_6("occluded_goal_edge_weight", data.getOccludedGoalEdgeWeight());
      ser.write_type_6("weight_for_inter_region_edge", data.getWeightForInterRegionEdge());
      ser.write_type_7("return_best_effort_solution", data.getReturnBestEffortSolution());
      ser.write_type_7("optimize_for_narrow_passage", data.getOptimizeForNarrowPassage());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setMaxInterRegionConnectionLength(ser.read_type_6("max_inter_region_connection_length"));
      data.setNormalZThresholdForAccessibleRegions(ser.read_type_6("normal_z_threshold_for_accessible_regions"));
      data.setNavigableExtrusionDistance(ser.read_type_6("navigable_extrusion_distance"));
      data.setObstacleExtrusionDistance(ser.read_type_6("obstacle_extrusion_distance"));
      data.setObstacleExtrusionDistanceIfNotTooHighToStep(ser.read_type_6("obstacle_extrusion_distance_if_not_too_high_to_step"));
      data.setTooHighToStepDistance(ser.read_type_6("too_high_to_step_distance"));
      data.setHeightForMaxAvoidance(ser.read_type_6("height_for_max_avoidance"));
      data.setClusterResolution(ser.read_type_6("cluster_resolution"));
      data.setExplorationDistanceFromStartGoal(ser.read_type_6("exploration_distance_from_start_goal"));
      data.setPlanarRegionMinArea(ser.read_type_6("planar_region_min_area"));
      data.setPlanarRegionMinSize(ser.read_type_4("planar_region_min_size"));
      data.setRegionOrthogonalAngle(ser.read_type_6("region_orthogonal_angle"));
      data.setSearchHostRegionEpsilon(ser.read_type_6("search_host_region_epsilon"));
      data.setCanDuckUnderHeight(ser.read_type_6("can_duck_under_height"));
      data.setCanEasilyStepOverHeight(ser.read_type_6("can_easily_step_over_height"));
      data.setLengthForLongInterRegionEdge(ser.read_type_6("length_for_long_inter_region_edge"));
      data.setPerformPostProcessingNodeShifting(ser.read_type_7("perform_post_processing_node_shifting"));
      data.setIntroduceMidpointsInPostProcessing(ser.read_type_7("introduce_midpoints_in_post_processing"));
      data.setComputeOrientationsToAvoidObstacles(ser.read_type_7("compute_orientations_to_avoid_obstacles"));
      data.setHeuristicWeight(ser.read_type_6("heuristic_weight"));
      data.setDistanceWeight(ser.read_type_6("distance_weight"));
      data.setElevationWeight(ser.read_type_6("elevation_weight"));
      data.setOccludedGoalEdgeWeight(ser.read_type_6("occluded_goal_edge_weight"));
      data.setWeightForInterRegionEdge(ser.read_type_6("weight_for_inter_region_edge"));
      data.setReturnBestEffortSolution(ser.read_type_7("return_best_effort_solution"));
      data.setOptimizeForNarrowPassage(ser.read_type_7("optimize_for_narrow_passage"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket src, toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket createData()
   {
      return new toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket();
   }
   @Override
   public int getTypeSize()
   {
      return us.ihmc.idl.CDR.getTypeSize(getMaxCdrSerializedSize());
   }

   @Override
   public java.lang.String getName()
   {
      return name;
   }
   
   public void serialize(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket src, toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public VisibilityGraphsParametersPacketPubSubType newInstance()
   {
      return new VisibilityGraphsParametersPacketPubSubType();
   }
}
