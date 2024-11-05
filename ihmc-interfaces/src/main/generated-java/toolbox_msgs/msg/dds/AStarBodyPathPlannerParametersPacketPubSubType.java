package toolbox_msgs.msg.dds;

/**
* 
* Topic data type of the struct "AStarBodyPathPlannerParametersPacket" defined in "AStarBodyPathPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from AStarBodyPathPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit AStarBodyPathPlannerParametersPacket_.idl instead.
*
*/
public class AStarBodyPathPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket>
{
   public static final java.lang.String name = "toolbox_msgs::msg::dds_::AStarBodyPathPlannerParametersPacket_";
   
   @Override
   public final java.lang.String getDefinitionChecksum()
   {
   		return "fc7bc927cf2bd0aadef7fc3b35e035c01609d70cc507bdacf9b2bb6325bf1c30";
   }
   
   @Override
   public final java.lang.String getDefinitionVersion()
   {
   		return "local";
   }

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data) throws java.io.IOException
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


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


      current_alignment += 1 + us.ihmc.idl.CDR.alignment(current_alignment, 1);


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



      return current_alignment - initial_alignment;
   }

   public static void write(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_4(data.getSequenceId());

      cdr.write_type_7(data.getCheckForCollisions());

      cdr.write_type_7(data.getComputeSurfaceNormalCost());

      cdr.write_type_7(data.getComputeTraversibility());

      cdr.write_type_7(data.getPerformSmoothing());

      cdr.write_type_6(data.getRollCostWeight());

      cdr.write_type_6(data.getRollCostDeadband());

      cdr.write_type_6(data.getMaxPenalizedRollAngle());

      cdr.write_type_6(data.getSnapRadius());

      cdr.write_type_6(data.getMinSnapHeightThreshold());

      cdr.write_type_6(data.getInclineCostWeight());

      cdr.write_type_6(data.getInclineCostDeadband());

      cdr.write_type_6(data.getMaxIncline());

      cdr.write_type_6(data.getCollisionBoxSizeY());

      cdr.write_type_6(data.getCollisionBoxSizeX());

      cdr.write_type_6(data.getCollisionBoxGroundClearance());

      cdr.write_type_6(data.getTraversibilityWeight());

      cdr.write_type_6(data.getTraversibilityStanceWeight());

      cdr.write_type_6(data.getTraversibilityStepWeight());

      cdr.write_type_6(data.getMinTraversibilityScore());

      cdr.write_type_6(data.getMinNormalAngleToPenalizeForTraversibility());

      cdr.write_type_6(data.getMaxNormalAngleToPenalizeForTraversibility());

      cdr.write_type_6(data.getTraversibilityInclineWeight());

      cdr.write_type_6(data.getTraversibilitySearchWidth());

      cdr.write_type_2(data.getMinOccupiedNeighborsForTraversibility());

      cdr.write_type_6(data.getHalfStanceWidth());

      cdr.write_type_6(data.getTraversibilityHeightWindowWidth());

      cdr.write_type_6(data.getTraversibilityHeightWindowDeadband());

      cdr.write_type_6(data.getHeightProximityForSayingWalkingOnGround());

      cdr.write_type_6(data.getTraversibilityNonGroundDiscountWhenWalkingOnGround());

      cdr.write_type_6(data.getSmootherCollisionWeight());

      cdr.write_type_6(data.getSmootherSmoothnessWeight());

      cdr.write_type_6(data.getSmootherTurnPointSmoothnessDiscount());

      cdr.write_type_6(data.getSmootherMinCurvatureToPenalize());

      cdr.write_type_6(data.getSmootherEqualSpacingWeight());

      cdr.write_type_6(data.getSmootherRollWeight());

      cdr.write_type_6(data.getSmootherDisplacementWeight());

      cdr.write_type_6(data.getSmootherTraversibilityWeight());

      cdr.write_type_6(data.getSmootherGroundPlaneWeight());

      cdr.write_type_6(data.getSmootherMinimumTraversibilityToSearchFor());

      cdr.write_type_6(data.getSmootherTraversibilityThresholdForNoDiscount());

      cdr.write_type_6(data.getSmootherHillClimbGain());

      cdr.write_type_6(data.getSmootherGradientThresholdToTerminate());

   }

   public static void read(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setSequenceId(cdr.read_type_4());
      	
      data.setCheckForCollisions(cdr.read_type_7());
      	
      data.setComputeSurfaceNormalCost(cdr.read_type_7());
      	
      data.setComputeTraversibility(cdr.read_type_7());
      	
      data.setPerformSmoothing(cdr.read_type_7());
      	
      data.setRollCostWeight(cdr.read_type_6());
      	
      data.setRollCostDeadband(cdr.read_type_6());
      	
      data.setMaxPenalizedRollAngle(cdr.read_type_6());
      	
      data.setSnapRadius(cdr.read_type_6());
      	
      data.setMinSnapHeightThreshold(cdr.read_type_6());
      	
      data.setInclineCostWeight(cdr.read_type_6());
      	
      data.setInclineCostDeadband(cdr.read_type_6());
      	
      data.setMaxIncline(cdr.read_type_6());
      	
      data.setCollisionBoxSizeY(cdr.read_type_6());
      	
      data.setCollisionBoxSizeX(cdr.read_type_6());
      	
      data.setCollisionBoxGroundClearance(cdr.read_type_6());
      	
      data.setTraversibilityWeight(cdr.read_type_6());
      	
      data.setTraversibilityStanceWeight(cdr.read_type_6());
      	
      data.setTraversibilityStepWeight(cdr.read_type_6());
      	
      data.setMinTraversibilityScore(cdr.read_type_6());
      	
      data.setMinNormalAngleToPenalizeForTraversibility(cdr.read_type_6());
      	
      data.setMaxNormalAngleToPenalizeForTraversibility(cdr.read_type_6());
      	
      data.setTraversibilityInclineWeight(cdr.read_type_6());
      	
      data.setTraversibilitySearchWidth(cdr.read_type_6());
      	
      data.setMinOccupiedNeighborsForTraversibility(cdr.read_type_2());
      	
      data.setHalfStanceWidth(cdr.read_type_6());
      	
      data.setTraversibilityHeightWindowWidth(cdr.read_type_6());
      	
      data.setTraversibilityHeightWindowDeadband(cdr.read_type_6());
      	
      data.setHeightProximityForSayingWalkingOnGround(cdr.read_type_6());
      	
      data.setTraversibilityNonGroundDiscountWhenWalkingOnGround(cdr.read_type_6());
      	
      data.setSmootherCollisionWeight(cdr.read_type_6());
      	
      data.setSmootherSmoothnessWeight(cdr.read_type_6());
      	
      data.setSmootherTurnPointSmoothnessDiscount(cdr.read_type_6());
      	
      data.setSmootherMinCurvatureToPenalize(cdr.read_type_6());
      	
      data.setSmootherEqualSpacingWeight(cdr.read_type_6());
      	
      data.setSmootherRollWeight(cdr.read_type_6());
      	
      data.setSmootherDisplacementWeight(cdr.read_type_6());
      	
      data.setSmootherTraversibilityWeight(cdr.read_type_6());
      	
      data.setSmootherGroundPlaneWeight(cdr.read_type_6());
      	
      data.setSmootherMinimumTraversibilityToSearchFor(cdr.read_type_6());
      	
      data.setSmootherTraversibilityThresholdForNoDiscount(cdr.read_type_6());
      	
      data.setSmootherHillClimbGain(cdr.read_type_6());
      	
      data.setSmootherGradientThresholdToTerminate(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_4("sequence_id", data.getSequenceId());
      ser.write_type_7("check_for_collisions", data.getCheckForCollisions());
      ser.write_type_7("compute_surface_normal_cost", data.getComputeSurfaceNormalCost());
      ser.write_type_7("compute_traversibility", data.getComputeTraversibility());
      ser.write_type_7("perform_smoothing", data.getPerformSmoothing());
      ser.write_type_6("roll_cost_weight", data.getRollCostWeight());
      ser.write_type_6("roll_cost_deadband", data.getRollCostDeadband());
      ser.write_type_6("max_penalized_roll_angle", data.getMaxPenalizedRollAngle());
      ser.write_type_6("snap_radius", data.getSnapRadius());
      ser.write_type_6("min_snap_height_threshold", data.getMinSnapHeightThreshold());
      ser.write_type_6("incline_cost_weight", data.getInclineCostWeight());
      ser.write_type_6("incline_cost_deadband", data.getInclineCostDeadband());
      ser.write_type_6("max_incline", data.getMaxIncline());
      ser.write_type_6("collision_box_size_y", data.getCollisionBoxSizeY());
      ser.write_type_6("collision_box_size_x", data.getCollisionBoxSizeX());
      ser.write_type_6("collision_box_ground_clearance", data.getCollisionBoxGroundClearance());
      ser.write_type_6("traversibility_weight", data.getTraversibilityWeight());
      ser.write_type_6("traversibility_stance_weight", data.getTraversibilityStanceWeight());
      ser.write_type_6("traversibility_step_weight", data.getTraversibilityStepWeight());
      ser.write_type_6("min_traversibility_score", data.getMinTraversibilityScore());
      ser.write_type_6("min_normal_angle_to_penalize_for_traversibility", data.getMinNormalAngleToPenalizeForTraversibility());
      ser.write_type_6("max_normal_angle_to_penalize_for_traversibility", data.getMaxNormalAngleToPenalizeForTraversibility());
      ser.write_type_6("traversibility_incline_weight", data.getTraversibilityInclineWeight());
      ser.write_type_6("traversibility_search_width", data.getTraversibilitySearchWidth());
      ser.write_type_2("min_occupied_neighbors_for_traversibility", data.getMinOccupiedNeighborsForTraversibility());
      ser.write_type_6("half_stance_width", data.getHalfStanceWidth());
      ser.write_type_6("traversibility_height_window_width", data.getTraversibilityHeightWindowWidth());
      ser.write_type_6("traversibility_height_window_deadband", data.getTraversibilityHeightWindowDeadband());
      ser.write_type_6("height_proximity_for_saying_walking_on_ground", data.getHeightProximityForSayingWalkingOnGround());
      ser.write_type_6("traversibility_non_ground_discount_when_walking_on_ground", data.getTraversibilityNonGroundDiscountWhenWalkingOnGround());
      ser.write_type_6("smoother_collision_weight", data.getSmootherCollisionWeight());
      ser.write_type_6("smoother_smoothness_weight", data.getSmootherSmoothnessWeight());
      ser.write_type_6("smoother_turn_point_smoothness_discount", data.getSmootherTurnPointSmoothnessDiscount());
      ser.write_type_6("smoother_min_curvature_to_penalize", data.getSmootherMinCurvatureToPenalize());
      ser.write_type_6("smoother_equal_spacing_weight", data.getSmootherEqualSpacingWeight());
      ser.write_type_6("smoother_roll_weight", data.getSmootherRollWeight());
      ser.write_type_6("smoother_displacement_weight", data.getSmootherDisplacementWeight());
      ser.write_type_6("smoother_traversibility_weight", data.getSmootherTraversibilityWeight());
      ser.write_type_6("smoother_ground_plane_weight", data.getSmootherGroundPlaneWeight());
      ser.write_type_6("smoother_minimum_traversibility_to_search_for", data.getSmootherMinimumTraversibilityToSearchFor());
      ser.write_type_6("smoother_traversibility_threshold_for_no_discount", data.getSmootherTraversibilityThresholdForNoDiscount());
      ser.write_type_6("smoother_hill_climb_gain", data.getSmootherHillClimbGain());
      ser.write_type_6("smoother_gradient_threshold_to_terminate", data.getSmootherGradientThresholdToTerminate());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data)
   {
      data.setSequenceId(ser.read_type_4("sequence_id"));
      data.setCheckForCollisions(ser.read_type_7("check_for_collisions"));
      data.setComputeSurfaceNormalCost(ser.read_type_7("compute_surface_normal_cost"));
      data.setComputeTraversibility(ser.read_type_7("compute_traversibility"));
      data.setPerformSmoothing(ser.read_type_7("perform_smoothing"));
      data.setRollCostWeight(ser.read_type_6("roll_cost_weight"));
      data.setRollCostDeadband(ser.read_type_6("roll_cost_deadband"));
      data.setMaxPenalizedRollAngle(ser.read_type_6("max_penalized_roll_angle"));
      data.setSnapRadius(ser.read_type_6("snap_radius"));
      data.setMinSnapHeightThreshold(ser.read_type_6("min_snap_height_threshold"));
      data.setInclineCostWeight(ser.read_type_6("incline_cost_weight"));
      data.setInclineCostDeadband(ser.read_type_6("incline_cost_deadband"));
      data.setMaxIncline(ser.read_type_6("max_incline"));
      data.setCollisionBoxSizeY(ser.read_type_6("collision_box_size_y"));
      data.setCollisionBoxSizeX(ser.read_type_6("collision_box_size_x"));
      data.setCollisionBoxGroundClearance(ser.read_type_6("collision_box_ground_clearance"));
      data.setTraversibilityWeight(ser.read_type_6("traversibility_weight"));
      data.setTraversibilityStanceWeight(ser.read_type_6("traversibility_stance_weight"));
      data.setTraversibilityStepWeight(ser.read_type_6("traversibility_step_weight"));
      data.setMinTraversibilityScore(ser.read_type_6("min_traversibility_score"));
      data.setMinNormalAngleToPenalizeForTraversibility(ser.read_type_6("min_normal_angle_to_penalize_for_traversibility"));
      data.setMaxNormalAngleToPenalizeForTraversibility(ser.read_type_6("max_normal_angle_to_penalize_for_traversibility"));
      data.setTraversibilityInclineWeight(ser.read_type_6("traversibility_incline_weight"));
      data.setTraversibilitySearchWidth(ser.read_type_6("traversibility_search_width"));
      data.setMinOccupiedNeighborsForTraversibility(ser.read_type_2("min_occupied_neighbors_for_traversibility"));
      data.setHalfStanceWidth(ser.read_type_6("half_stance_width"));
      data.setTraversibilityHeightWindowWidth(ser.read_type_6("traversibility_height_window_width"));
      data.setTraversibilityHeightWindowDeadband(ser.read_type_6("traversibility_height_window_deadband"));
      data.setHeightProximityForSayingWalkingOnGround(ser.read_type_6("height_proximity_for_saying_walking_on_ground"));
      data.setTraversibilityNonGroundDiscountWhenWalkingOnGround(ser.read_type_6("traversibility_non_ground_discount_when_walking_on_ground"));
      data.setSmootherCollisionWeight(ser.read_type_6("smoother_collision_weight"));
      data.setSmootherSmoothnessWeight(ser.read_type_6("smoother_smoothness_weight"));
      data.setSmootherTurnPointSmoothnessDiscount(ser.read_type_6("smoother_turn_point_smoothness_discount"));
      data.setSmootherMinCurvatureToPenalize(ser.read_type_6("smoother_min_curvature_to_penalize"));
      data.setSmootherEqualSpacingWeight(ser.read_type_6("smoother_equal_spacing_weight"));
      data.setSmootherRollWeight(ser.read_type_6("smoother_roll_weight"));
      data.setSmootherDisplacementWeight(ser.read_type_6("smoother_displacement_weight"));
      data.setSmootherTraversibilityWeight(ser.read_type_6("smoother_traversibility_weight"));
      data.setSmootherGroundPlaneWeight(ser.read_type_6("smoother_ground_plane_weight"));
      data.setSmootherMinimumTraversibilityToSearchFor(ser.read_type_6("smoother_minimum_traversibility_to_search_for"));
      data.setSmootherTraversibilityThresholdForNoDiscount(ser.read_type_6("smoother_traversibility_threshold_for_no_discount"));
      data.setSmootherHillClimbGain(ser.read_type_6("smoother_hill_climb_gain"));
      data.setSmootherGradientThresholdToTerminate(ser.read_type_6("smoother_gradient_threshold_to_terminate"));
   }

   public static void staticCopy(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket src, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket createData()
   {
      return new toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket();
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
   
   public void serialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket src, toolbox_msgs.msg.dds.AStarBodyPathPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public AStarBodyPathPlannerParametersPacketPubSubType newInstance()
   {
      return new AStarBodyPathPlannerParametersPacketPubSubType();
   }
}
