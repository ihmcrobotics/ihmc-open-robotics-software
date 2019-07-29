package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "FootstepPlannerParametersPacket" defined in "FootstepPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from FootstepPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit FootstepPlannerParametersPacket_.idl instead.
*
*/
public class FootstepPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.FootstepPlannerParametersPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::FootstepPlannerParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.FootstepPlannerParametersPacket data) throws java.io.IOException
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

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

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

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


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


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getCheckForBodyBoxCollisions());

      cdr.write_type_7(data.getPerformHeuristicSearchPolicies());

      cdr.write_type_6(data.getIdealFootstepWidth());

      cdr.write_type_6(data.getIdealFootstepLength());

      cdr.write_type_6(data.getWiggleInsideDelta());

      cdr.write_type_6(data.getMaximumStepReach());

      cdr.write_type_6(data.getMaximumStepYaw());

      cdr.write_type_6(data.getMinimumStepWidth());

      cdr.write_type_6(data.getMinimumStepLength());

      cdr.write_type_6(data.getMinimumStepYaw());

      cdr.write_type_6(data.getMaximumStepReachWhenSteppingUp());

      cdr.write_type_6(data.getMaximumStepZWhenSteppingUp());

      cdr.write_type_6(data.getMaximumStepXWhenForwardAndDown());

      cdr.write_type_6(data.getMaximumStepZWhenForwardAndDown());

      cdr.write_type_6(data.getMaximumStepZ());

      cdr.write_type_6(data.getMinimumFootholdPercent());

      cdr.write_type_6(data.getMinimumSurfaceInclineRadians());

      cdr.write_type_7(data.getWiggleIntoConvexHullOfPlanarRegions());

      cdr.write_type_7(data.getRejectIfCannotFullyWiggleInside());

      cdr.write_type_6(data.getMaximumXyWiggleDistance());

      cdr.write_type_6(data.getMaximumYawWiggle());

      cdr.write_type_6(data.getMaximumZPenetrationOnValleyRegions());

      cdr.write_type_6(data.getMaximumStepWidth());

      cdr.write_type_6(data.getCliffHeightToAvoid());

      cdr.write_type_6(data.getMinimumDistanceFromCliffBottoms());

      cdr.write_type_7(data.getReturnBestEffortPlan());

      cdr.write_type_4(data.getMinimumStepsForBestEffortPlan());

      cdr.write_type_6(data.getBodyGroundClearance());

      cdr.write_type_6(data.getBodyBoxHeight());

      cdr.write_type_6(data.getBodyBoxDepth());

      cdr.write_type_6(data.getBodyBoxWidth());

      cdr.write_type_6(data.getBodyBoxBaseX());

      cdr.write_type_6(data.getBodyBoxBaseY());

      cdr.write_type_6(data.getBodyBoxBaseZ());

      cdr.write_type_6(data.getMinXClearanceFromStance());

      cdr.write_type_6(data.getMinYClearanceFromStance());

      cdr.write_type_6(data.getYawWeight());

      cdr.write_type_6(data.getPitchWeight());

      cdr.write_type_6(data.getRollWeight());

      cdr.write_type_6(data.getForwardWeight());

      cdr.write_type_6(data.getLateralWeight());

      cdr.write_type_6(data.getStepUpWeight());

      cdr.write_type_6(data.getStepDownWeight());

      cdr.write_type_6(data.getCostPerStep());

      cdr.write_type_7(data.getUseQuadraticDistanceCost());

      cdr.write_type_7(data.getUseQuadraticHeightCost());

      cdr.write_type_6(data.getAStarHeuristicsWeight());

      cdr.write_type_6(data.getVisGraphWithAStarHeuristicsWeight());

      cdr.write_type_6(data.getDepthFirstHeuristicsWeight());

      cdr.write_type_6(data.getBodyPathBasedHeuristicsWeight());

      cdr.write_type_6(data.getMaximum2dDistanceFromBoundingBoxToPenalize());

      cdr.write_type_6(data.getBoundingBoxCost());

   }

   public static void read(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCheckForBodyBoxCollisions(cdr.read_type_7());
      	
      data.setPerformHeuristicSearchPolicies(cdr.read_type_7());
      	
      data.setIdealFootstepWidth(cdr.read_type_6());
      	
      data.setIdealFootstepLength(cdr.read_type_6());
      	
      data.setWiggleInsideDelta(cdr.read_type_6());
      	
      data.setMaximumStepReach(cdr.read_type_6());
      	
      data.setMaximumStepYaw(cdr.read_type_6());
      	
      data.setMinimumStepWidth(cdr.read_type_6());
      	
      data.setMinimumStepLength(cdr.read_type_6());
      	
      data.setMinimumStepYaw(cdr.read_type_6());
      	
      data.setMaximumStepReachWhenSteppingUp(cdr.read_type_6());
      	
      data.setMaximumStepZWhenSteppingUp(cdr.read_type_6());
      	
      data.setMaximumStepXWhenForwardAndDown(cdr.read_type_6());
      	
      data.setMaximumStepZWhenForwardAndDown(cdr.read_type_6());
      	
      data.setMaximumStepZ(cdr.read_type_6());
      	
      data.setMinimumFootholdPercent(cdr.read_type_6());
      	
      data.setMinimumSurfaceInclineRadians(cdr.read_type_6());
      	
      data.setWiggleIntoConvexHullOfPlanarRegions(cdr.read_type_7());
      	
      data.setRejectIfCannotFullyWiggleInside(cdr.read_type_7());
      	
      data.setMaximumXyWiggleDistance(cdr.read_type_6());
      	
      data.setMaximumYawWiggle(cdr.read_type_6());
      	
      data.setMaximumZPenetrationOnValleyRegions(cdr.read_type_6());
      	
      data.setMaximumStepWidth(cdr.read_type_6());
      	
      data.setCliffHeightToAvoid(cdr.read_type_6());
      	
      data.setMinimumDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setReturnBestEffortPlan(cdr.read_type_7());
      	
      data.setMinimumStepsForBestEffortPlan(cdr.read_type_4());
      	
      data.setBodyGroundClearance(cdr.read_type_6());
      	
      data.setBodyBoxHeight(cdr.read_type_6());
      	
      data.setBodyBoxDepth(cdr.read_type_6());
      	
      data.setBodyBoxWidth(cdr.read_type_6());
      	
      data.setBodyBoxBaseX(cdr.read_type_6());
      	
      data.setBodyBoxBaseY(cdr.read_type_6());
      	
      data.setBodyBoxBaseZ(cdr.read_type_6());
      	
      data.setMinXClearanceFromStance(cdr.read_type_6());
      	
      data.setMinYClearanceFromStance(cdr.read_type_6());
      	
      data.setYawWeight(cdr.read_type_6());
      	
      data.setPitchWeight(cdr.read_type_6());
      	
      data.setRollWeight(cdr.read_type_6());
      	
      data.setForwardWeight(cdr.read_type_6());
      	
      data.setLateralWeight(cdr.read_type_6());
      	
      data.setStepUpWeight(cdr.read_type_6());
      	
      data.setStepDownWeight(cdr.read_type_6());
      	
      data.setCostPerStep(cdr.read_type_6());
      	
      data.setUseQuadraticDistanceCost(cdr.read_type_7());
      	
      data.setUseQuadraticHeightCost(cdr.read_type_7());
      	
      data.setAStarHeuristicsWeight(cdr.read_type_6());
      	
      data.setVisGraphWithAStarHeuristicsWeight(cdr.read_type_6());
      	
      data.setDepthFirstHeuristicsWeight(cdr.read_type_6());
      	
      data.setBodyPathBasedHeuristicsWeight(cdr.read_type_6());
      	
      data.setMaximum2dDistanceFromBoundingBoxToPenalize(cdr.read_type_6());
      	
      data.setBoundingBoxCost(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("check_for_body_box_collisions", data.getCheckForBodyBoxCollisions());
      ser.write_type_7("perform_heuristic_search_policies", data.getPerformHeuristicSearchPolicies());
      ser.write_type_6("ideal_footstep_width", data.getIdealFootstepWidth());
      ser.write_type_6("ideal_footstep_length", data.getIdealFootstepLength());
      ser.write_type_6("wiggle_inside_delta", data.getWiggleInsideDelta());
      ser.write_type_6("maximum_step_reach", data.getMaximumStepReach());
      ser.write_type_6("maximum_step_yaw", data.getMaximumStepYaw());
      ser.write_type_6("minimum_step_width", data.getMinimumStepWidth());
      ser.write_type_6("minimum_step_length", data.getMinimumStepLength());
      ser.write_type_6("minimum_step_yaw", data.getMinimumStepYaw());
      ser.write_type_6("maximum_step_reach_when_stepping_up", data.getMaximumStepReachWhenSteppingUp());
      ser.write_type_6("maximum_step_z_when_stepping_up", data.getMaximumStepZWhenSteppingUp());
      ser.write_type_6("maximum_step_x_when_forward_and_down", data.getMaximumStepXWhenForwardAndDown());
      ser.write_type_6("maximum_step_z_when_forward_and_down", data.getMaximumStepZWhenForwardAndDown());
      ser.write_type_6("maximum_step_z", data.getMaximumStepZ());
      ser.write_type_6("minimum_foothold_percent", data.getMinimumFootholdPercent());
      ser.write_type_6("minimum_surface_incline_radians", data.getMinimumSurfaceInclineRadians());
      ser.write_type_7("wiggle_into_convex_hull_of_planar_regions", data.getWiggleIntoConvexHullOfPlanarRegions());
      ser.write_type_7("reject_if_cannot_fully_wiggle_inside", data.getRejectIfCannotFullyWiggleInside());
      ser.write_type_6("maximum_xy_wiggle_distance", data.getMaximumXyWiggleDistance());
      ser.write_type_6("maximum_yaw_wiggle", data.getMaximumYawWiggle());
      ser.write_type_6("maximum_z_penetration_on_valley_regions", data.getMaximumZPenetrationOnValleyRegions());
      ser.write_type_6("maximum_step_width", data.getMaximumStepWidth());
      ser.write_type_6("cliff_height_to_avoid", data.getCliffHeightToAvoid());
      ser.write_type_6("minimum_distance_from_cliff_bottoms", data.getMinimumDistanceFromCliffBottoms());
      ser.write_type_7("return_best_effort_plan", data.getReturnBestEffortPlan());
      ser.write_type_4("minimum_steps_for_best_effort_plan", data.getMinimumStepsForBestEffortPlan());
      ser.write_type_6("body_ground_clearance", data.getBodyGroundClearance());
      ser.write_type_6("body_box_height", data.getBodyBoxHeight());
      ser.write_type_6("body_box_depth", data.getBodyBoxDepth());
      ser.write_type_6("body_box_width", data.getBodyBoxWidth());
      ser.write_type_6("body_box_base_x", data.getBodyBoxBaseX());
      ser.write_type_6("body_box_base_y", data.getBodyBoxBaseY());
      ser.write_type_6("body_box_base_z", data.getBodyBoxBaseZ());
      ser.write_type_6("min_x_clearance_from_stance", data.getMinXClearanceFromStance());
      ser.write_type_6("min_y_clearance_from_stance", data.getMinYClearanceFromStance());
      ser.write_type_6("yaw_weight", data.getYawWeight());
      ser.write_type_6("pitch_weight", data.getPitchWeight());
      ser.write_type_6("roll_weight", data.getRollWeight());
      ser.write_type_6("forward_weight", data.getForwardWeight());
      ser.write_type_6("lateral_weight", data.getLateralWeight());
      ser.write_type_6("step_up_weight", data.getStepUpWeight());
      ser.write_type_6("step_down_weight", data.getStepDownWeight());
      ser.write_type_6("cost_per_step", data.getCostPerStep());
      ser.write_type_7("use_quadratic_distance_cost", data.getUseQuadraticDistanceCost());
      ser.write_type_7("use_quadratic_height_cost", data.getUseQuadraticHeightCost());
      ser.write_type_6("a_star_heuristics_weight", data.getAStarHeuristicsWeight());
      ser.write_type_6("vis_graph_with_a_star_heuristics_weight", data.getVisGraphWithAStarHeuristicsWeight());
      ser.write_type_6("depth_first_heuristics_weight", data.getDepthFirstHeuristicsWeight());
      ser.write_type_6("body_path_based_heuristics_weight", data.getBodyPathBasedHeuristicsWeight());
      ser.write_type_6("maximum_2d_distance_from_bounding_box_to_penalize", data.getMaximum2dDistanceFromBoundingBoxToPenalize());
      ser.write_type_6("bounding_box_cost", data.getBoundingBoxCost());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.FootstepPlannerParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCheckForBodyBoxCollisions(ser.read_type_7("check_for_body_box_collisions"));
      data.setPerformHeuristicSearchPolicies(ser.read_type_7("perform_heuristic_search_policies"));
      data.setIdealFootstepWidth(ser.read_type_6("ideal_footstep_width"));
      data.setIdealFootstepLength(ser.read_type_6("ideal_footstep_length"));
      data.setWiggleInsideDelta(ser.read_type_6("wiggle_inside_delta"));
      data.setMaximumStepReach(ser.read_type_6("maximum_step_reach"));
      data.setMaximumStepYaw(ser.read_type_6("maximum_step_yaw"));
      data.setMinimumStepWidth(ser.read_type_6("minimum_step_width"));
      data.setMinimumStepLength(ser.read_type_6("minimum_step_length"));
      data.setMinimumStepYaw(ser.read_type_6("minimum_step_yaw"));
      data.setMaximumStepReachWhenSteppingUp(ser.read_type_6("maximum_step_reach_when_stepping_up"));
      data.setMaximumStepZWhenSteppingUp(ser.read_type_6("maximum_step_z_when_stepping_up"));
      data.setMaximumStepXWhenForwardAndDown(ser.read_type_6("maximum_step_x_when_forward_and_down"));
      data.setMaximumStepZWhenForwardAndDown(ser.read_type_6("maximum_step_z_when_forward_and_down"));
      data.setMaximumStepZ(ser.read_type_6("maximum_step_z"));
      data.setMinimumFootholdPercent(ser.read_type_6("minimum_foothold_percent"));
      data.setMinimumSurfaceInclineRadians(ser.read_type_6("minimum_surface_incline_radians"));
      data.setWiggleIntoConvexHullOfPlanarRegions(ser.read_type_7("wiggle_into_convex_hull_of_planar_regions"));
      data.setRejectIfCannotFullyWiggleInside(ser.read_type_7("reject_if_cannot_fully_wiggle_inside"));
      data.setMaximumXyWiggleDistance(ser.read_type_6("maximum_xy_wiggle_distance"));
      data.setMaximumYawWiggle(ser.read_type_6("maximum_yaw_wiggle"));
      data.setMaximumZPenetrationOnValleyRegions(ser.read_type_6("maximum_z_penetration_on_valley_regions"));
      data.setMaximumStepWidth(ser.read_type_6("maximum_step_width"));
      data.setCliffHeightToAvoid(ser.read_type_6("cliff_height_to_avoid"));
      data.setMinimumDistanceFromCliffBottoms(ser.read_type_6("minimum_distance_from_cliff_bottoms"));
      data.setReturnBestEffortPlan(ser.read_type_7("return_best_effort_plan"));
      data.setMinimumStepsForBestEffortPlan(ser.read_type_4("minimum_steps_for_best_effort_plan"));
      data.setBodyGroundClearance(ser.read_type_6("body_ground_clearance"));
      data.setBodyBoxHeight(ser.read_type_6("body_box_height"));
      data.setBodyBoxDepth(ser.read_type_6("body_box_depth"));
      data.setBodyBoxWidth(ser.read_type_6("body_box_width"));
      data.setBodyBoxBaseX(ser.read_type_6("body_box_base_x"));
      data.setBodyBoxBaseY(ser.read_type_6("body_box_base_y"));
      data.setBodyBoxBaseZ(ser.read_type_6("body_box_base_z"));
      data.setMinXClearanceFromStance(ser.read_type_6("min_x_clearance_from_stance"));
      data.setMinYClearanceFromStance(ser.read_type_6("min_y_clearance_from_stance"));
      data.setYawWeight(ser.read_type_6("yaw_weight"));
      data.setPitchWeight(ser.read_type_6("pitch_weight"));
      data.setRollWeight(ser.read_type_6("roll_weight"));
      data.setForwardWeight(ser.read_type_6("forward_weight"));
      data.setLateralWeight(ser.read_type_6("lateral_weight"));
      data.setStepUpWeight(ser.read_type_6("step_up_weight"));
      data.setStepDownWeight(ser.read_type_6("step_down_weight"));
      data.setCostPerStep(ser.read_type_6("cost_per_step"));
      data.setUseQuadraticDistanceCost(ser.read_type_7("use_quadratic_distance_cost"));
      data.setUseQuadraticHeightCost(ser.read_type_7("use_quadratic_height_cost"));
      data.setAStarHeuristicsWeight(ser.read_type_6("a_star_heuristics_weight"));
      data.setVisGraphWithAStarHeuristicsWeight(ser.read_type_6("vis_graph_with_a_star_heuristics_weight"));
      data.setDepthFirstHeuristicsWeight(ser.read_type_6("depth_first_heuristics_weight"));
      data.setBodyPathBasedHeuristicsWeight(ser.read_type_6("body_path_based_heuristics_weight"));
      data.setMaximum2dDistanceFromBoundingBoxToPenalize(ser.read_type_6("maximum_2d_distance_from_bounding_box_to_penalize"));
      data.setBoundingBoxCost(ser.read_type_6("bounding_box_cost"));
   }

   public static void staticCopy(controller_msgs.msg.dds.FootstepPlannerParametersPacket src, controller_msgs.msg.dds.FootstepPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.FootstepPlannerParametersPacket createData()
   {
      return new controller_msgs.msg.dds.FootstepPlannerParametersPacket();
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
   
   public void serialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.FootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.FootstepPlannerParametersPacket src, controller_msgs.msg.dds.FootstepPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public FootstepPlannerParametersPacketPubSubType newInstance()
   {
      return new FootstepPlannerParametersPacketPubSubType();
   }
}
