package controller_msgs.msg.dds;

/**
* 
* Topic data type of the struct "ValkyrieFootstepPlannerParametersPacket" defined in "ValkyrieFootstepPlannerParametersPacket_.idl". Use this class to provide the TopicDataType to a Participant. 
*
* This file was automatically generated from ValkyrieFootstepPlannerParametersPacket_.idl by us.ihmc.idl.generator.IDLGenerator. 
* Do not update this file directly, edit ValkyrieFootstepPlannerParametersPacket_.idl instead.
*
*/
public class ValkyrieFootstepPlannerParametersPacketPubSubType implements us.ihmc.pubsub.TopicDataType<controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket>
{
   public static final java.lang.String name = "controller_msgs::msg::dds_::ValkyrieFootstepPlannerParametersPacket_";

   private final us.ihmc.idl.CDR serializeCDR = new us.ihmc.idl.CDR();
   private final us.ihmc.idl.CDR deserializeCDR = new us.ihmc.idl.CDR();

   @Override
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data, us.ihmc.pubsub.common.SerializedPayload serializedPayload) throws java.io.IOException
   {
      serializeCDR.serialize(serializedPayload);
      write(data, serializeCDR);
      serializeCDR.finishSerialize();
   }

   @Override
   public void deserialize(us.ihmc.pubsub.common.SerializedPayload serializedPayload, controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data) throws java.io.IOException
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

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getMaxCdrSerializedSize(current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      return current_alignment - initial_alignment;
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data)
   {
      return getCdrSerializedSize(data, 0);
   }

   public final static int getCdrSerializedSize(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data, int current_alignment)
   {
      int initial_alignment = current_alignment;

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


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getBodyBoxDimensions(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getBodyBoxOffset(), current_alignment);

      current_alignment += 4 + us.ihmc.idl.CDR.alignment(current_alignment, 4);


      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getTranslationWeight(), current_alignment);

      current_alignment += geometry_msgs.msg.dds.Vector3PubSubType.getCdrSerializedSize(data.getOrientationWeight(), current_alignment);

      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);


      current_alignment += 8 + us.ihmc.idl.CDR.alignment(current_alignment, 8);



      return current_alignment - initial_alignment;
   }

   public static void write(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      cdr.write_type_6(data.getIdealFootstepWidth());

      cdr.write_type_6(data.getMinimumStepLength());

      cdr.write_type_6(data.getIdealFootstepLength());

      cdr.write_type_6(data.getMinimumStepWidth());

      cdr.write_type_6(data.getMaximumStepWidth());

      cdr.write_type_6(data.getMaximumStepReach());

      cdr.write_type_6(data.getMinXClearanceFromStance());

      cdr.write_type_6(data.getMinYClearanceFromStance());

      cdr.write_type_6(data.getMinimumStepYaw());

      cdr.write_type_6(data.getMaximumStepYaw());

      cdr.write_type_6(data.getStepYawReductionFactorAtMaxReach());

      cdr.write_type_6(data.getMaximumStepZ());

      cdr.write_type_6(data.getMinimumFootholdPercent());

      cdr.write_type_6(data.getMaximumSurfaceInclineRadians());

      cdr.write_type_7(data.getWiggleWhilePlanning());

      cdr.write_type_6(data.getWiggleInsideDelta());

      cdr.write_type_6(data.getMaximumXyWiggleDistance());

      cdr.write_type_6(data.getMaximumYawWiggle());

      cdr.write_type_6(data.getCliffHeightToAvoid());

      cdr.write_type_6(data.getMinimumDistanceFromCliffBottoms());

      cdr.write_type_6(data.getFlatGroundLowerThreshold());

      cdr.write_type_6(data.getFlatGroundUpperThreshold());

      cdr.write_type_6(data.getMaximumStepWidthWhenSteppingDown());

      cdr.write_type_6(data.getMaximumStepReachWhenSteppingDown());

      cdr.write_type_6(data.getMaximumStepWidthWhenSteppingUp());

      cdr.write_type_6(data.getMaximumStepReachWhenSteppingUp());

      cdr.write_type_6(data.getTranslationScaleFromGrandparentNode());

      cdr.write_type_6(data.getFinalTurnProximity());

      cdr.write_type_7(data.getCheckForPathCollisions());

      cdr.write_type_7(data.getCheckForBodyBoxCollisions());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getBodyBoxDimensions(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getBodyBoxOffset(), cdr);
      cdr.write_type_4(data.getNumberOfBoundingBoxChecks());

      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getTranslationWeight(), cdr);
      geometry_msgs.msg.dds.Vector3PubSubType.write(data.getOrientationWeight(), cdr);
      cdr.write_type_6(data.getCostPerStep());

      cdr.write_type_6(data.getFootholdAreaWeight());

      cdr.write_type_6(data.getAStarHeuristicsWeight());

      cdr.write_type_6(data.getWaypointCost());

   }

   public static void read(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      data.setIdealFootstepWidth(cdr.read_type_6());
      	
      data.setMinimumStepLength(cdr.read_type_6());
      	
      data.setIdealFootstepLength(cdr.read_type_6());
      	
      data.setMinimumStepWidth(cdr.read_type_6());
      	
      data.setMaximumStepWidth(cdr.read_type_6());
      	
      data.setMaximumStepReach(cdr.read_type_6());
      	
      data.setMinXClearanceFromStance(cdr.read_type_6());
      	
      data.setMinYClearanceFromStance(cdr.read_type_6());
      	
      data.setMinimumStepYaw(cdr.read_type_6());
      	
      data.setMaximumStepYaw(cdr.read_type_6());
      	
      data.setStepYawReductionFactorAtMaxReach(cdr.read_type_6());
      	
      data.setMaximumStepZ(cdr.read_type_6());
      	
      data.setMinimumFootholdPercent(cdr.read_type_6());
      	
      data.setMaximumSurfaceInclineRadians(cdr.read_type_6());
      	
      data.setWiggleWhilePlanning(cdr.read_type_7());
      	
      data.setWiggleInsideDelta(cdr.read_type_6());
      	
      data.setMaximumXyWiggleDistance(cdr.read_type_6());
      	
      data.setMaximumYawWiggle(cdr.read_type_6());
      	
      data.setCliffHeightToAvoid(cdr.read_type_6());
      	
      data.setMinimumDistanceFromCliffBottoms(cdr.read_type_6());
      	
      data.setFlatGroundLowerThreshold(cdr.read_type_6());
      	
      data.setFlatGroundUpperThreshold(cdr.read_type_6());
      	
      data.setMaximumStepWidthWhenSteppingDown(cdr.read_type_6());
      	
      data.setMaximumStepReachWhenSteppingDown(cdr.read_type_6());
      	
      data.setMaximumStepWidthWhenSteppingUp(cdr.read_type_6());
      	
      data.setMaximumStepReachWhenSteppingUp(cdr.read_type_6());
      	
      data.setTranslationScaleFromGrandparentNode(cdr.read_type_6());
      	
      data.setFinalTurnProximity(cdr.read_type_6());
      	
      data.setCheckForPathCollisions(cdr.read_type_7());
      	
      data.setCheckForBodyBoxCollisions(cdr.read_type_7());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getBodyBoxDimensions(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getBodyBoxOffset(), cdr);	
      data.setNumberOfBoundingBoxChecks(cdr.read_type_4());
      	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getTranslationWeight(), cdr);	
      geometry_msgs.msg.dds.Vector3PubSubType.read(data.getOrientationWeight(), cdr);	
      data.setCostPerStep(cdr.read_type_6());
      	
      data.setFootholdAreaWeight(cdr.read_type_6());
      	
      data.setAStarHeuristicsWeight(cdr.read_type_6());
      	
      data.setWaypointCost(cdr.read_type_6());
      	

   }

   @Override
   public final void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data, us.ihmc.idl.InterchangeSerializer ser)
   {
      ser.write_type_6("ideal_footstep_width", data.getIdealFootstepWidth());
      ser.write_type_6("minimum_step_length", data.getMinimumStepLength());
      ser.write_type_6("ideal_footstep_length", data.getIdealFootstepLength());
      ser.write_type_6("minimum_step_width", data.getMinimumStepWidth());
      ser.write_type_6("maximum_step_width", data.getMaximumStepWidth());
      ser.write_type_6("maximum_step_reach", data.getMaximumStepReach());
      ser.write_type_6("min_x_clearance_from_stance", data.getMinXClearanceFromStance());
      ser.write_type_6("min_y_clearance_from_stance", data.getMinYClearanceFromStance());
      ser.write_type_6("minimum_step_yaw", data.getMinimumStepYaw());
      ser.write_type_6("maximum_step_yaw", data.getMaximumStepYaw());
      ser.write_type_6("step_yaw_reduction_factor_at_max_reach", data.getStepYawReductionFactorAtMaxReach());
      ser.write_type_6("maximum_step_z", data.getMaximumStepZ());
      ser.write_type_6("minimum_foothold_percent", data.getMinimumFootholdPercent());
      ser.write_type_6("maximum_surface_incline_radians", data.getMaximumSurfaceInclineRadians());
      ser.write_type_7("wiggle_while_planning", data.getWiggleWhilePlanning());
      ser.write_type_6("wiggle_inside_delta", data.getWiggleInsideDelta());
      ser.write_type_6("maximum_xy_wiggle_distance", data.getMaximumXyWiggleDistance());
      ser.write_type_6("maximum_yaw_wiggle", data.getMaximumYawWiggle());
      ser.write_type_6("cliff_height_to_avoid", data.getCliffHeightToAvoid());
      ser.write_type_6("minimum_distance_from_cliff_bottoms", data.getMinimumDistanceFromCliffBottoms());
      ser.write_type_6("flat_ground_lower_threshold", data.getFlatGroundLowerThreshold());
      ser.write_type_6("flat_ground_upper_threshold", data.getFlatGroundUpperThreshold());
      ser.write_type_6("maximum_step_width_when_stepping_down", data.getMaximumStepWidthWhenSteppingDown());
      ser.write_type_6("maximum_step_reach_when_stepping_down", data.getMaximumStepReachWhenSteppingDown());
      ser.write_type_6("maximum_step_width_when_stepping_up", data.getMaximumStepWidthWhenSteppingUp());
      ser.write_type_6("maximum_step_reach_when_stepping_up", data.getMaximumStepReachWhenSteppingUp());
      ser.write_type_6("translation_scale_from_grandparent_node", data.getTranslationScaleFromGrandparentNode());
      ser.write_type_6("final_turn_proximity", data.getFinalTurnProximity());
      ser.write_type_7("check_for_path_collisions", data.getCheckForPathCollisions());
      ser.write_type_7("check_for_body_box_collisions", data.getCheckForBodyBoxCollisions());
      ser.write_type_a("body_box_dimensions", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBodyBoxDimensions());

      ser.write_type_a("body_box_offset", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBodyBoxOffset());

      ser.write_type_4("number_of_bounding_box_checks", data.getNumberOfBoundingBoxChecks());
      ser.write_type_a("translation_weight", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTranslationWeight());

      ser.write_type_a("orientation_weight", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getOrientationWeight());

      ser.write_type_6("cost_per_step", data.getCostPerStep());
      ser.write_type_6("foothold_area_weight", data.getFootholdAreaWeight());
      ser.write_type_6("a_star_heuristics_weight", data.getAStarHeuristicsWeight());
      ser.write_type_6("waypoint_cost", data.getWaypointCost());
   }

   @Override
   public final void deserialize(us.ihmc.idl.InterchangeSerializer ser, controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data)
   {
      data.setIdealFootstepWidth(ser.read_type_6("ideal_footstep_width"));
      data.setMinimumStepLength(ser.read_type_6("minimum_step_length"));
      data.setIdealFootstepLength(ser.read_type_6("ideal_footstep_length"));
      data.setMinimumStepWidth(ser.read_type_6("minimum_step_width"));
      data.setMaximumStepWidth(ser.read_type_6("maximum_step_width"));
      data.setMaximumStepReach(ser.read_type_6("maximum_step_reach"));
      data.setMinXClearanceFromStance(ser.read_type_6("min_x_clearance_from_stance"));
      data.setMinYClearanceFromStance(ser.read_type_6("min_y_clearance_from_stance"));
      data.setMinimumStepYaw(ser.read_type_6("minimum_step_yaw"));
      data.setMaximumStepYaw(ser.read_type_6("maximum_step_yaw"));
      data.setStepYawReductionFactorAtMaxReach(ser.read_type_6("step_yaw_reduction_factor_at_max_reach"));
      data.setMaximumStepZ(ser.read_type_6("maximum_step_z"));
      data.setMinimumFootholdPercent(ser.read_type_6("minimum_foothold_percent"));
      data.setMaximumSurfaceInclineRadians(ser.read_type_6("maximum_surface_incline_radians"));
      data.setWiggleWhilePlanning(ser.read_type_7("wiggle_while_planning"));
      data.setWiggleInsideDelta(ser.read_type_6("wiggle_inside_delta"));
      data.setMaximumXyWiggleDistance(ser.read_type_6("maximum_xy_wiggle_distance"));
      data.setMaximumYawWiggle(ser.read_type_6("maximum_yaw_wiggle"));
      data.setCliffHeightToAvoid(ser.read_type_6("cliff_height_to_avoid"));
      data.setMinimumDistanceFromCliffBottoms(ser.read_type_6("minimum_distance_from_cliff_bottoms"));
      data.setFlatGroundLowerThreshold(ser.read_type_6("flat_ground_lower_threshold"));
      data.setFlatGroundUpperThreshold(ser.read_type_6("flat_ground_upper_threshold"));
      data.setMaximumStepWidthWhenSteppingDown(ser.read_type_6("maximum_step_width_when_stepping_down"));
      data.setMaximumStepReachWhenSteppingDown(ser.read_type_6("maximum_step_reach_when_stepping_down"));
      data.setMaximumStepWidthWhenSteppingUp(ser.read_type_6("maximum_step_width_when_stepping_up"));
      data.setMaximumStepReachWhenSteppingUp(ser.read_type_6("maximum_step_reach_when_stepping_up"));
      data.setTranslationScaleFromGrandparentNode(ser.read_type_6("translation_scale_from_grandparent_node"));
      data.setFinalTurnProximity(ser.read_type_6("final_turn_proximity"));
      data.setCheckForPathCollisions(ser.read_type_7("check_for_path_collisions"));
      data.setCheckForBodyBoxCollisions(ser.read_type_7("check_for_body_box_collisions"));
      ser.read_type_a("body_box_dimensions", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBodyBoxDimensions());

      ser.read_type_a("body_box_offset", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getBodyBoxOffset());

      data.setNumberOfBoundingBoxChecks(ser.read_type_4("number_of_bounding_box_checks"));
      ser.read_type_a("translation_weight", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getTranslationWeight());

      ser.read_type_a("orientation_weight", new geometry_msgs.msg.dds.Vector3PubSubType(), data.getOrientationWeight());

      data.setCostPerStep(ser.read_type_6("cost_per_step"));
      data.setFootholdAreaWeight(ser.read_type_6("foothold_area_weight"));
      data.setAStarHeuristicsWeight(ser.read_type_6("a_star_heuristics_weight"));
      data.setWaypointCost(ser.read_type_6("waypoint_cost"));
   }

   public static void staticCopy(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket src, controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket dest)
   {
      dest.set(src);
   }

   @Override
   public controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket createData()
   {
      return new controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket();
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
   
   public void serialize(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      write(data, cdr);
   }

   public void deserialize(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket data, us.ihmc.idl.CDR cdr)
   {
      read(data, cdr);
   }
   
   public void copy(controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket src, controller_msgs.msg.dds.ValkyrieFootstepPlannerParametersPacket dest)
   {
      staticCopy(src, dest);
   }

   @Override
   public ValkyrieFootstepPlannerParametersPacketPubSubType newInstance()
   {
      return new ValkyrieFootstepPlannerParametersPacketPubSubType();
   }
}
